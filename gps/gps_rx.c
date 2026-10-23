/*
 * gps_rx.c - standalone GPS L1 C/A software receiver
 *
 * Supports two modes:
 *   File:    ./gps_rx capture.iq          (batch, same as before)
 *   Stream:  ./gps_rx                     (reads complex float32 IQ from stdin)
 *            ./gps_rx -                   (explicit stdin)
 *
 * Streaming lets you pipe live IQ from GNURadio for real-time GPS fixes:
 *
 *   Linux (named pipe):
 *     mkfifo /tmp/gps_iq
 *     # set GNURadio File Sink path to /tmp/gps_iq
 *     ./gps_rx < /tmp/gps_iq
 *
 *   Linux (direct pipe from GRC-generated Python):
 *     python3 gps_recv.py --output -  |  ./gps_rx
 *
 *   Any OS (TCP):
 *     # In GNURadio: add a TCP Sink on port 5555
 *     ncat localhost 5555 | ./gps_rx
 *
 * Input format: complex float32 (interleaved I/Q, 8 bytes per sample),
 *               4.096 MS/s, centered on 1575.42 MHz.
 *
 * Build: gcc -O3 -march=native -o gps_rx gps_rx.c -lm
 */
#define _FILE_OFFSET_BITS 64
#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE /* usleep() on glibc */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <limits.h>

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#include <windows.h>
#define FSEEK _fseeki64
#define FTELL _ftelli64
#else
#include <unistd.h>
#define FSEEK fseeko
#define FTELL ftello
#endif

/* ------------------------- configuration ------------------------- */
#define FS            4096000.0
#define NS            4096
#define F_L1          1575.42e6
#define CODE_RATE     1.023e6
#define C_LIGHT       299792458.0
#define OMEGA_E       7.2921151467e-5
#define MU            3.986005e14
#define PI            3.14159265358979323846

#define ACQ_MS        20
#define ACQ_DOPP_MAX  20000
#define ACQ_DOPP_STEP 250
#define ACQ_THRESH    1.8
#define ACQ_START_S   0.5
#define MAX_CH        12

#define FLL_MS        400
#define PLL_BN        20.0
#define ZETA          0.7071
#define DLL_GAIN      0.01

#define TRACK_MS_MAX  300000       /* 5 minutes; for streaming, effectively unlimited */

#define STREAM_INIT_CAP (1 << 22) /* ~4M samples initial buffer */

/* ------------------------- basic types --------------------------- */
typedef struct { double r, i; } cpx;
typedef struct { float i, q; } sample_t;

static inline cpx cmul(cpx a, cpx b){ cpx c={a.r*b.r-a.i*b.i, a.r*b.i+a.i*b.r}; return c; }

/* ------------------------- stream -------------------------------- */
#define STREAM_MODE_FILE   0   /* batch: read completed file                */
#define STREAM_MODE_STDIN  1   /* pipe: read stdin                         */
#define STREAM_MODE_TAIL   2   /* tail: read growing file, poll for growth */

typedef struct {
    FILE *f;
    sample_t *buf;
    long long base;      /* absolute sample index of buf[0] */
    long long len;       /* valid samples currently in buf   */
    long long capacity;  /* allocated buf size in samples    */
    long long total;     /* file mode: total samples in file; stream: -1 */
    int mode;
    int eof;
} stream_t;

static int stream_open_file(stream_t *s, const char *path)
{
    memset(s, 0, sizeof(*s));
    s->f = fopen(path, "rb");
    if (!s->f) return 0;
    FSEEK(s->f, 0, SEEK_END);
    s->total = (long long)FTELL(s->f) / (long long)sizeof(sample_t);
    s->capacity = STREAM_INIT_CAP;
    s->buf = (sample_t *)malloc((size_t)s->capacity * sizeof(sample_t));
    s->mode = STREAM_MODE_FILE;
    return s->buf != NULL;
}

static int stream_open_stdin(stream_t *s)
{
    memset(s, 0, sizeof(*s));
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
#endif
    s->f = stdin;
    s->mode = STREAM_MODE_STDIN;
    s->total = -1;
    s->capacity = STREAM_INIT_CAP;
    s->buf = (sample_t *)malloc((size_t)s->capacity * sizeof(sample_t));
    return s->buf != NULL;
}

static int stream_open_tail(stream_t *s, const char *path)
{
    memset(s, 0, sizeof(*s));
    s->f = fopen(path, "rb");
    if (!s->f) return 0;
    s->mode = STREAM_MODE_TAIL;
    s->total = -1;
    s->capacity = STREAM_INIT_CAP;
    s->buf = (sample_t *)malloc((size_t)s->capacity * sizeof(sample_t));
    return s->buf != NULL;
}

static void stream_sleep_ms(int ms)
{
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

/* try to read more samples; for tail mode, retries with sleep */
static int stream_read_more(stream_t *s, long long need_total)
{
    int max_retries = (s->mode == STREAM_MODE_TAIL) ? 200 : 0; /* 200 * 50ms = 10s */
    int attempt = 0;

    while (s->base + s->len < need_total) {
        if (s->len >= s->capacity) {
            s->capacity *= 2;
            s->buf = (sample_t *)realloc(s->buf,
                        (size_t)s->capacity * sizeof(sample_t));
        }
        long long space = s->capacity - s->len;
        size_t got = fread(s->buf + s->len, sizeof(sample_t),
                           (size_t)space, s->f);
        if (got > 0) {
            s->len += (long long)got;
            attempt = 0;
        } else if (s->mode == STREAM_MODE_TAIL && attempt < max_retries) {
            clearerr(s->f);
            stream_sleep_ms(50);
            attempt++;
        } else {
            if (s->mode != STREAM_MODE_TAIL) s->eof = 1;
            break;
        }
    }
    return s->base + s->len >= need_total;
}

static const sample_t *stream_get(stream_t *s, long long n, long long need)
{
    if (s->mode == STREAM_MODE_STDIN || s->mode == STREAM_MODE_TAIL) {
        /* streamed samples before base were trimmed and can't be re-read */
        if (n < s->base) return NULL;
        if (!stream_read_more(s, n + need)) return NULL;
        return s->buf + (n - s->base);
    } else {
        /* file mode: chunk-based random access (original behavior) */
        if (n < 0 || n + need > s->total) return NULL;
        if (n < s->base || n + need > s->base + s->len) {
            FSEEK(s->f, (long long)n * (long long)sizeof(sample_t), SEEK_SET);
            long long want = s->capacity;
            if (want > s->total - n) want = s->total - n;
            s->len = (long long)fread(s->buf, sizeof(sample_t),
                                       (size_t)want, s->f);
            s->base = n;
            if (s->len < need) return NULL;
        }
        return s->buf + (n - s->base);
    }
}

/* discard samples before min_needed to free memory (streaming only) */
static void stream_trim(stream_t *s, long long min_needed)
{
    if (s->mode == STREAM_MODE_FILE) return;
    if (min_needed <= s->base) return;
    long long discard = min_needed - s->base;
    if (discard >= s->len) {
        s->base += s->len;
        s->len = 0;
    } else {
        memmove(s->buf, s->buf + discard,
                (size_t)(s->len - discard) * sizeof(sample_t));
        s->base += discard;
        s->len -= discard;
    }
}

/* ensure at least `need` total samples are available (blocking read) */
static int stream_fill(stream_t *s, long long need)
{
    return stream_get(s, need - 1, 1) != NULL;
}

/* ------------------------- C/A code ------------------------------ */
static const int g2tap[33][2] = {
    {0,0},{2,6},{3,7},{4,8},{5,9},{1,9},{2,10},{1,8},{2,9},{3,10},{2,3},{3,4},
    {5,6},{6,7},{7,8},{8,9},{9,10},{1,4},{2,5},{3,6},{4,7},{5,8},{6,9},{1,3},
    {4,6},{5,7},{6,8},{7,9},{8,10},{1,6},{2,7},{3,8},{4,9}};

static void gen_ca(int prn, int8_t *out)
{
    int g1[10], g2[10], k;
    for (k = 0; k < 10; k++) g1[k] = g2[k] = 1;
    for (int i = 0; i < 1023; i++) {
        int bit = g1[9] ^ g2[g2tap[prn][0]-1] ^ g2[g2tap[prn][1]-1];
        out[i] = bit ? -1 : 1;
        int f1 = g1[2] ^ g1[9];
        int f2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];
        for (k = 9; k > 0; k--) { g1[k] = g1[k-1]; g2[k] = g2[k-1]; }
        g1[0] = f1; g2[0] = f2;
    }
}

static inline int cidx(double p)
{
    int i = (int)floor(p);
    if (i < 0) i += 1023; else if (i >= 1023) i -= 1023;
    return i;
}

/* ------------------------- FFT ----------------------------------- */
static void fft(cpx *x, int n, int inv)
{
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { cpx t = x[i]; x[i] = x[j]; x[j] = t; }
    }
    for (int len = 2; len <= n; len <<= 1) {
        double ang = 2.0 * PI / len * (inv ? 1.0 : -1.0);
        cpx wl = { cos(ang), sin(ang) };
        int h = len >> 1;
        for (int i = 0; i < n; i += len) {
            cpx w = { 1.0, 0.0 };
            for (int j = 0; j < h; j++) {
                cpx u = x[i+j], v = cmul(x[i+j+h], w);
                x[i+j].r = u.r + v.r;   x[i+j].i = u.i + v.i;
                x[i+j+h].r = u.r - v.r; x[i+j+h].i = u.i - v.i;
                w = cmul(w, wl);
            }
        }
    }
    if (inv) for (int i = 0; i < n; i++) { x[i].r /= n; x[i].i /= n; }
}

/* ------------------------- acquisition --------------------------- */
typedef struct { int prn; double dop; int cp; double ratio; } acq_t;

static int cmp_acq(const void *a, const void *b)
{
    double d = ((const acq_t *)b)->ratio - ((const acq_t *)a)->ratio;
    return (d > 0) - (d < 0);
}

static int acquire(stream_t *s, long long start, acq_t *out)
{
    cpx *blk  = (cpx *)malloc(sizeof(cpx) * NS * ACQ_MS);
    cpx *spec = (cpx *)malloc(sizeof(cpx) * NS * ACQ_MS);
    cpx *cf   = (cpx *)malloc(sizeof(cpx) * NS * 33);
    cpx *car  = (cpx *)malloc(sizeof(cpx) * NS);
    cpx *prod = (cpx *)malloc(sizeof(cpx) * NS);
    cpx *tmp  = (cpx *)malloc(sizeof(cpx) * NS);
    double *acc = (double *)malloc(sizeof(double) * NS);
    double bpeak[33] = {0}, bratio[33] = {0}, bdop[33] = {0};
    int bcp[33] = {0};
    int8_t code[1023];
    int nout = 0;

    for (int k = 0; k < ACQ_MS; k++) {
        const sample_t *p = stream_get(s, start + (long long)k * NS, NS);
        if (!p) { fprintf(stderr, "not enough data for acquisition\n"); return 0; }
        for (int n = 0; n < NS; n++) { blk[k*NS+n].r = p[n].i; blk[k*NS+n].i = p[n].q; }
    }
    for (int prn = 1; prn <= 32; prn++) {
        gen_ca(prn, code);
        for (int n = 0; n < NS; n++) { tmp[n].r = code[(int)(n * 1023.0 / NS)]; tmp[n].i = 0; }
        fft(tmp, NS, 0);
        for (int n = 0; n < NS; n++) { cf[prn*NS+n].r = tmp[n].r; cf[prn*NS+n].i = -tmp[n].i; }
    }

    fprintf(stderr, "Acquisition: searching 32 PRNs, +/-%d Hz ...\n", ACQ_DOPP_MAX);
    for (int dop = -ACQ_DOPP_MAX; dop <= ACQ_DOPP_MAX; dop += ACQ_DOPP_STEP) {
        for (int n = 0; n < NS; n++) {
            double a = -2.0 * PI * dop * n / FS;
            car[n].r = cos(a); car[n].i = sin(a);
        }
        for (int k = 0; k < ACQ_MS; k++) {
            for (int n = 0; n < NS; n++) spec[k*NS+n] = cmul(blk[k*NS+n], car[n]);
            fft(spec + k*NS, NS, 0);
        }
        for (int prn = 1; prn <= 32; prn++) {
            memset(acc, 0, sizeof(double) * NS);
            for (int k = 0; k < ACQ_MS; k++) {
                for (int n = 0; n < NS; n++) prod[n] = cmul(spec[k*NS+n], cf[prn*NS+n]);
                fft(prod, NS, 1);
                for (int n = 0; n < NS; n++) acc[n] += prod[n].r*prod[n].r + prod[n].i*prod[n].i;
            }
            int pk = 0; double pv = 0;
            for (int n = 0; n < NS; n++) if (acc[n] > pv) { pv = acc[n]; pk = n; }
            if (pv > bpeak[prn]) {
                double second = 0;
                for (int n = 0; n < NS; n++) {
                    int d = abs(n - pk); if (d > NS/2) d = NS - d;
                    if (d > 8 && acc[n] > second) second = acc[n];
                }
                bpeak[prn] = pv; bcp[prn] = pk; bdop[prn] = dop;
                bratio[prn] = second > 0 ? pv / second : 0;
            }
        }
    }

    fprintf(stderr, "\n--- Acquisition results ---\n");
    for (int prn = 1; prn <= 32; prn++) {
        fprintf(stderr, "PRN %2d: peak=%12.3e  ratio=%6.2f  dop=%7.0f Hz  cp=%4d\n",
                prn, bpeak[prn], bratio[prn], bdop[prn], bcp[prn]);
    }
    fprintf(stderr, "--- End acquisition results ---\n");

    for (int prn = 1; prn <= 32; prn++) {
        if (bratio[prn] >= ACQ_THRESH && nout < MAX_CH * 2) {
            out[nout].prn = prn;
            out[nout].dop = bdop[prn];
            out[nout].cp = bcp[prn];
            out[nout].ratio = bratio[prn];
            nout++;
        }
    }

    qsort(out, nout, sizeof(acq_t), cmp_acq);
    if (nout > MAX_CH) nout = MAX_CH;
    free(blk); free(spec); free(cf); free(car); free(prod); free(tmp); free(acc);
    return nout;
}

/* ------------------------- ephemeris ----------------------------- */
typedef struct {
    int have1, have2, have3, valid;
    int iodc, iode2, iode3, week, health;
    double tgd, toc, af0, af1, af2;
    double crs, dn, m0, cuc, ecc, cus, sqrta, toe;
    double cic, omg0, cis, i0, crc, omg, omgd, idot;
} eph_t;

/* ------------------------- channel ------------------------------- */
typedef struct {
    int prn;
    int8_t code[1023];

    /* correlation history (grows during tracking) */
    double *ip, *qp, *epos;
    int nep, maxep;

    /* tracking NCO state */
    long long n;
    double p;            /* code phase within current epoch */
    double phase;        /* carrier phase */
    double fcar;         /* carrier frequency estimate */
    double wint;         /* PLL integrator */
    double ipv, qpv;     /* previous epoch prompt I/Q (for FLL) */
    int have_prev;

    /* status */
    int active;
    int locked;

    /* navigation */
    int have_t0, e0;
    double t0;
    eph_t eph;
    int nsf;
} chan_t;

/* ------------------------- tracking ------------------------------ */
static void track_init(chan_t *c, long long n0, double dop0)
{
    c->maxep = 8192;
    c->ip   = (double *)malloc(sizeof(double) * c->maxep);
    c->qp   = (double *)malloc(sizeof(double) * c->maxep);
    c->epos = (double *)malloc(sizeof(double) * c->maxep);
    gen_ca(c->prn, c->code);
    c->n = n0;
    c->p = 0.0;
    c->phase = 0.0;
    c->fcar = dop0;
    c->wint = 0.0;
    c->ipv = c->qpv = 0.0;
    c->have_prev = 0;
    c->nep = 0;
    c->active = 1;
    c->locked = 0;
    memset(&c->eph, 0, sizeof(eph_t));
    c->have_t0 = 0;
    c->nsf = 0;
}

/* process one code epoch (~1 ms); returns 1=ok, 0=no data, -1=lock fail */
static int track_epoch(stream_t *s, chan_t *c)
{
    if (!c->active) return 0;
    int i = c->nep;

    /* grow arrays if needed */
    if (i >= c->maxep) {
        c->maxep *= 2;
        c->ip   = (double *)realloc(c->ip,   sizeof(double) * c->maxep);
        c->qp   = (double *)realloc(c->qp,   sizeof(double) * c->maxep);
        c->epos = (double *)realloc(c->epos,  sizeof(double) * c->maxep);
    }

    double step = CODE_RATE * (1.0 + c->fcar / F_L1) / FS;
    double dphi = 2.0 * PI * c->fcar / FS;
    const sample_t *x = stream_get(s, c->n, NS + 16);
    if (!x) return 0;

    c->epos[i] = (double)c->n - c->p / step;

    double er = cos(c->phase), ei = -sin(c->phase);
    double rr = cos(dphi),     ri = -sin(dphi);
    double Pi = 0, Pq = 0, Ei = 0, Eq = 0, Li = 0, Lq = 0;
    int k = 0;
    double p = c->p;
    while (p < 1023.0 && k < NS + 8) {
        double xr = x[k].i, xi = x[k].q;
        double sr = xr * er - xi * ei, si = xr * ei + xi * er;
        double pe = p + 0.5, pl = p - 0.5;
        double cp = c->code[cidx(p)], ce = c->code[cidx(pe)], cl = c->code[cidx(pl)];
        Pi += sr * cp; Pq += si * cp;
        Ei += sr * ce; Eq += si * ce;
        Li += sr * cl; Lq += si * cl;
        double t = er * rr - ei * ri; ei = er * ri + ei * rr; er = t;
        p += step; k++;
    }
    c->n += k;
    p -= 1023.0;
    c->phase = fmod(c->phase + dphi * k, 2.0 * PI);
    c->ip[i] = Pi;
    c->qp[i] = Pq;

    /* DLL: non-coherent early-late */
    double Em = hypot(Ei, Eq), Lm = hypot(Li, Lq);
    if (Em + Lm > 0) p += DLL_GAIN * 0.5 * (Em - Lm) / (Em + Lm);
    c->p = p;

    /* carrier loop */
    double wn = PLL_BN / 0.53;
    if (i < FLL_MS) {
        if (c->have_prev) {
            double cross = c->ipv * Pq - c->qpv * Pi;
            double dot   = c->ipv * Pi + c->qpv * Pq;
            double fe = atan2(cross * (dot < 0 ? -1.0 : 1.0),
                              fabs(dot)) / (2.0 * PI * 0.001);
            c->fcar += 0.25 * fe;
        }
        if (i == FLL_MS - 1) c->wint = 2.0 * PI * c->fcar;
    } else {
        double perr = atan2(Pq * (Pi < 0 ? -1.0 : 1.0), fabs(Pi));
        c->wint += wn * wn * 0.001 * perr;
        c->fcar = (c->wint + 2.0 * ZETA * wn * perr) / (2.0 * PI);
    }
    c->ipv = Pi; c->qpv = Pq; c->have_prev = 1;
    c->nep = i + 1;

    /* lock check at epoch FLL_MS + 200 */
    if (i == FLL_MS + 199) {
        double si = 0, sq = 0;
        for (int j = FLL_MS; j <= i; j++) {
            si += fabs(c->ip[j]);
            sq += fabs(c->qp[j]);
        }
        if (si < 2.0 * sq) {
            c->active = 0;
            return -1;
        }
        c->locked = 1;
        fprintf(stderr, "  PRN %2d: carrier locked, freq %.1f Hz\n",
                c->prn, c->fcar);
    }

    return 1;
}

/* ------------------------- nav message --------------------------- */
static uint64_t getu(const int *d, int start, int len)
{
    uint64_t v = 0;
    for (int i = 0; i < len; i++) v = (v << 1) | (uint64_t)d[start + i];
    return v;
}
static int64_t signext(uint64_t v, int len)
{
    if ((v >> (len - 1)) & 1) return (int64_t)v - (int64_t)(1ULL << len);
    return (int64_t)v;
}
static int64_t gets_(const int *d, int start, int len) { return signext(getu(d, start, len), len); }
static uint64_t getu2(const int *d, int s1, int l1, int s2, int l2)
{ return (getu(d, s1, l1) << l2) | getu(d, s2, l2); }
static int64_t gets2(const int *d, int s1, int l1, int s2, int l2)
{ return signext(getu2(d, s1, l1, s2, l2), l1 + l2); }

static const int pm[6][16] = {
    {1,2,3,5,6,10,11,12,13,14,17,18,20,23,0},
    {2,3,4,6,7,11,12,13,14,15,18,19,21,24,0},
    {1,3,4,5,7,8,12,13,14,15,16,19,20,22,0},
    {2,4,5,6,8,9,13,14,15,16,17,20,21,23,0},
    {1,3,5,6,7,9,10,14,15,16,17,18,21,22,24,0},
    {3,5,6,8,9,10,11,13,15,19,22,23,24,0}};
static const int pstar[6] = {29, 30, 29, 30, 30, 29};

static int parity_subframe(const char *bits, int d29, int d30, int *d)
{
    for (int w = 0; w < 10; w++) {
        const char *W = bits + 30 * w;
        int dd[25];
        for (int i = 1; i <= 24; i++) dd[i] = W[i-1] ^ d30;
        for (int k = 0; k < 6; k++) {
            int p = (pstar[k] == 29) ? d29 : d30;
            for (int j = 0; pm[k][j]; j++) p ^= dd[pm[k][j]];
            if (p != W[24 + k]) return 0;
        }
        for (int i = 1; i <= 24; i++) d[30 * w + i] = dd[i];
        d29 = W[28]; d30 = W[29];
    }
    return 1;
}

static void parse_subframe(const int *d, int id, eph_t *e)
{
    if (id == 1) {
        e->week   = (int)getu(d, 61, 10);
        e->health = (int)getu(d, 77, 6);
        e->iodc   = (int)((getu(d, 83, 2) << 8) | getu(d, 211, 8));
        e->tgd    = gets_(d, 197, 8)  * pow(2, -31);
        e->toc    = getu(d, 219, 16)  * 16.0;
        e->af2    = gets_(d, 241, 8)  * pow(2, -55);
        e->af1    = gets_(d, 249, 16) * pow(2, -43);
        e->af0    = gets_(d, 271, 22) * pow(2, -31);
        e->have1  = 1;
    } else if (id == 2) {
        e->iode2 = (int)getu(d, 61, 8);
        e->crs   = gets_(d, 69, 16)                * pow(2, -5);
        e->dn    = gets_(d, 91, 16)                * pow(2, -43) * PI;
        e->m0    = gets2(d, 107, 8, 121, 24)       * pow(2, -31) * PI;
        e->cuc   = gets_(d, 151, 16)               * pow(2, -29);
        e->ecc   = getu2(d, 167, 8, 181, 24)       * pow(2, -33);
        e->cus   = gets_(d, 211, 16)               * pow(2, -29);
        e->sqrta = getu2(d, 227, 8, 241, 24)       * pow(2, -19);
        e->toe   = getu(d, 271, 16)                * 16.0;
        e->have2 = 1;
    } else if (id == 3) {
        e->cic   = gets_(d, 61, 16)                * pow(2, -29);
        e->omg0  = gets2(d, 77, 8, 91, 24)         * pow(2, -31) * PI;
        e->cis   = gets_(d, 121, 16)               * pow(2, -29);
        e->i0    = gets2(d, 137, 8, 151, 24)       * pow(2, -31) * PI;
        e->crc   = gets_(d, 181, 16)               * pow(2, -5);
        e->omg   = gets2(d, 197, 8, 211, 24)       * pow(2, -31) * PI;
        e->omgd  = gets_(d, 241, 24)               * pow(2, -43) * PI;
        e->iode3 = (int)getu(d, 271, 8);
        e->idot  = gets_(d, 279, 14)               * pow(2, -43) * PI;
        e->have3 = 1;
    }
    if (e->have1 && e->have2 && e->have3 &&
        (e->iodc & 0xFF) == e->iode2 && e->iode2 == e->iode3 && e->sqrta > 4000.0)
        e->valid = 1;
}

static void decode_nav(chan_t *c)
{
    int start = FLL_MS + 50, hist[20] = {0}, off = 0;
    for (int i = start + 1; i < c->nep; i++)
        if ((c->ip[i] > 0) != (c->ip[i-1] > 0)) hist[i % 20]++;
    for (int k = 1; k < 20; k++) if (hist[k] > hist[off]) off = k;
    int e_first = start; while (e_first % 20 != off) e_first++;
    int nb = (c->nep - e_first) / 20;
    if (nb < 310) return;
    char *bits = (char *)malloc(nb);
    for (int k = 0; k < nb; k++) {
        double sum = 0;
        for (int j = 0; j < 20; j++) sum += c->ip[e_first + 20 * k + j];
        bits[k] = sum > 0;
    }
    static const int pre[8] = {1,0,0,0,1,0,1,1};
    int d[301];
    c->nsf = 0;
    for (int b = 2; b + 300 <= nb; b++) {
        int m = 1, v = 1;
        for (int j = 0; j < 8; j++) {
            if (bits[b+j] != pre[j]) m = 0;
            if (bits[b+j] == pre[j]) v = 0;
        }
        if (!m && !v) continue;
        if (!parity_subframe(bits + b, bits[b-2], bits[b-1], d)) continue;
        int id = (int)getu(d, 50, 3);
        long tow = (long)getu(d, 31, 17);
        if (id < 1 || id > 5) continue;
        double tsf = tow * 6.0 - 6.0;
        int esf = e_first + 20 * b;
        if (!c->have_t0) { c->t0 = tsf; c->e0 = esf; c->have_t0 = 1; }
        else if (fabs(c->t0 + (esf - c->e0) * 0.001 - tsf) > 0.0005) continue;
        c->nsf++;
        if (id <= 3) parse_subframe(d, id, &c->eph);
    }
    free(bits);
}

/* ------------------------- satellite position -------------------- */
static double wrapw(double t)
{
    while (t > 302400.0) t -= 604800.0;
    while (t < -302400.0) t += 604800.0;
    return t;
}

static double kepler(const eph_t *e, double tk, double *Eout)
{
    double A = e->sqrta * e->sqrta;
    double n = sqrt(MU / (A * A * A)) + e->dn;
    double M = e->m0 + n * tk, E = M;
    for (int i = 0; i < 15; i++) E = M + e->ecc * sin(E);
    *Eout = E;
    return E;
}

static void sat_state(const eph_t *e, double ttx, double pos[3], double *dtsv)
{
    const double F = -4.442807633e-10;
    double E;
    kepler(e, wrapw(ttx - e->toe), &E);
    double tc = wrapw(ttx - e->toc);
    double dt = e->af0 + e->af1 * tc + e->af2 * tc * tc
              + F * e->ecc * e->sqrta * sin(E) - e->tgd;
    double t = ttx - dt;
    double tk = wrapw(t - e->toe);
    kepler(e, tk, &E);
    double A = e->sqrta * e->sqrta;
    double nu = atan2(sqrt(1.0 - e->ecc * e->ecc) * sin(E), cos(E) - e->ecc);
    double phi = nu + e->omg;
    double s2 = sin(2 * phi), c2 = cos(2 * phi);
    double u = phi + e->cus * s2 + e->cuc * c2;
    double r = A * (1.0 - e->ecc * cos(E)) + e->crs * s2 + e->crc * c2;
    double inc = e->i0 + e->cis * s2 + e->cic * c2 + e->idot * tk;
    double xp = r * cos(u), yp = r * sin(u);
    double om = e->omg0 + (e->omgd - OMEGA_E) * tk - OMEGA_E * e->toe;
    pos[0] = xp * cos(om) - yp * cos(inc) * sin(om);
    pos[1] = xp * sin(om) + yp * cos(inc) * cos(om);
    pos[2] = yp * sin(inc);
    *dtsv = dt;
}

/* ------------------------- position solution --------------------- */
static int solve4(double A[4][5])
{
    for (int c = 0; c < 4; c++) {
        int pv = c;
        for (int r = c + 1; r < 4; r++) if (fabs(A[r][c]) > fabs(A[pv][c])) pv = r;
        if (fabs(A[pv][c]) < 1e-12) return 0;
        for (int k = 0; k < 5; k++) { double t = A[c][k]; A[c][k] = A[pv][k]; A[pv][k] = t; }
        for (int r = 0; r < 4; r++) if (r != c) {
            double f = A[r][c] / A[c][c];
            for (int k = c; k < 5; k++) A[r][k] -= f * A[c][k];
        }
    }
    for (int r = 0; r < 4; r++) A[r][4] /= A[r][r];
    return 1;
}

static int lsq_fix(int n, double sp[][3], double rho[], double x[4], double *rms)
{
    x[0] = x[1] = x[2] = x[3] = 0.0;
    for (int it = 0; it < 20; it++) {
        double N[4][5] = {{0}}, H[16][4], y[16];
        for (int i = 0; i < n; i++) {
            double rx = x[0], ry = x[1], rz = x[2];
            double dx = sp[i][0] - rx, dy = sp[i][1] - ry, dz = sp[i][2] - rz;
            double r0 = sqrt(dx*dx + dy*dy + dz*dz);
            double th = OMEGA_E * r0 / C_LIGHT, ct = cos(th), st = sin(th);
            double sx = sp[i][0] * ct + sp[i][1] * st;
            double sy = -sp[i][0] * st + sp[i][1] * ct;
            double sz = sp[i][2];
            dx = sx - rx; dy = sy - ry; dz = sz - rz;
            double r = sqrt(dx*dx + dy*dy + dz*dz);
            H[i][0] = -dx / r; H[i][1] = -dy / r; H[i][2] = -dz / r; H[i][3] = 1.0;
            y[i] = rho[i] - (r + x[3]);
        }
        for (int i = 0; i < n; i++)
            for (int a = 0; a < 4; a++) {
                for (int b = 0; b < 4; b++) N[a][b] += H[i][a] * H[i][b];
                N[a][4] += H[i][a] * y[i];
            }
        if (!solve4(N)) return 0;
        for (int a = 0; a < 4; a++) x[a] += N[a][4];
        if (fabs(N[0][4]) + fabs(N[1][4]) + fabs(N[2][4]) < 1e-3) {
            double ss = 0;
            for (int i = 0; i < n; i++) {
                double dx = sp[i][0] - x[0], dy = sp[i][1] - x[1], dz = sp[i][2] - x[2];
                double r0 = sqrt(dx*dx + dy*dy + dz*dz);
                double th = OMEGA_E * r0 / C_LIGHT;
                double sx = sp[i][0] * cos(th) + sp[i][1] * sin(th);
                double sy = -sp[i][0] * sin(th) + sp[i][1] * cos(th);
                dx = sx - x[0]; dy = sy - x[1]; dz = sp[i][2] - x[2];
                double e = rho[i] - (sqrt(dx*dx + dy*dy + dz*dz) + x[3]);
                ss += e * e;
            }
            *rms = sqrt(ss / n);
            return 1;
        }
    }
    return 0;
}

static void ecef2llh(const double x[3], double *lat, double *lon, double *h)
{
    const double a = 6378137.0, f = 1.0 / 298.257223563, e2 = f * (2 - f);
    double p = hypot(x[0], x[1]);
    *lon = atan2(x[1], x[0]);
    double la = atan2(x[2], p * (1 - e2)), N = a;
    for (int i = 0; i < 10; i++) {
        N = a / sqrt(1 - e2 * sin(la) * sin(la));
        *h = p / cos(la) - N;
        la = atan2(x[2], p * (1 - e2 * N / (N + *h)));
    }
    *lat = la;
}

static int find_epoch(const chan_t *c, double S)
{
    int lo = 0, hi = c->nep - 2;
    while (lo < hi) {
        int mid = (lo + hi + 1) / 2;
        if (c->epos[mid] <= S) lo = mid; else hi = mid - 1;
    }
    return lo;
}

/* compute a single PVT fix from the current channel states; returns 1 on success */
static int pvt_snapshot(chan_t *ch, int nch, int *nfix,
                        double *sumlat, double *sumlon, double *sumh)
{
    chan_t *use[MAX_CH]; int nuse = 0;
    for (int i = 0; i < nch; i++)
        if (ch[i].active && ch[i].eph.valid && ch[i].have_t0 && nuse < MAX_CH)
            use[nuse++] = &ch[i];
    if (nuse < 4) return 0;

    /* find common sample-position window */
    double S = 0;
    int min_nep = INT_MAX;
    for (int i = 0; i < nuse; i++)
        if (use[i]->nep < min_nep) min_nep = use[i]->nep;
    if (min_nep < FLL_MS + 200) return 0;
    S = use[0]->epos[min_nep - 2];

    double ttx[MAX_CH], tmax = 0;
    for (int i = 0; i < nuse; i++) {
        chan_t *c = use[i];
        int e = find_epoch(c, S);
        if (e < 0 || e + 1 >= c->nep) return 0;
        double frac = (S - c->epos[e]) / (c->epos[e+1] - c->epos[e]);
        ttx[i] = c->t0 + ((e - c->e0) + frac) * 0.001;
        if (ttx[i] > tmax) tmax = ttx[i];
    }

    double trx = tmax + 0.070, sp[MAX_CH][3], rho[MAX_CH];
    for (int i = 0; i < nuse; i++) {
        double dtsv;
        sat_state(&use[i]->eph, ttx[i], sp[i], &dtsv);
        rho[i] = C_LIGHT * (trx - ttx[i]) + C_LIGHT * dtsv;
    }

    double x[4], rms, lat, lon, h;
    if (!lsq_fix(nuse, sp, rho, x, &rms)) return 0;
    ecef2llh(x, &lat, &lon, &h);
    if (h < -1000 || h > 20000) return 0;

    printf("%12.6f %12.6f %9.1f  %4d  %6.1f  %10.1f\n",
           lat * 180 / PI, lon * 180 / PI, h, nuse, rms, x[3]);
    fflush(stdout);

    *sumlat += lat; *sumlon += lon; *sumh += h; (*nfix)++;
    return 1;
}

/* batch PVT (original behavior for file mode) */
static int run_pvt(chan_t *ch, int nch)
{
    chan_t *use[MAX_CH]; int nuse = 0;
    for (int i = 0; i < nch; i++)
        if (ch[i].active && ch[i].eph.valid && ch[i].have_t0 && nuse < MAX_CH)
            use[nuse++] = &ch[i];
    if (nuse < 4) {
        fprintf(stderr, "Only %d satellites with ephemeris (need 4). Record a longer "
                        "file (90+ s) or improve signal.\n", nuse);
        return 3;
    }

    double slo = 0, shi = 1e30;
    for (int i = 0; i < nuse; i++) {
        double lo = use[i]->epos[FLL_MS + 100], hi = use[i]->epos[use[i]->nep - 2];
        if (lo > slo) slo = lo;
        if (hi < shi) shi = hi;
    }
    double sumlat = 0, sumlon = 0, sumh = 0; int nfix = 0;
    printf("\n time(s)      lat(deg)     lon(deg)    alt(m)  nsat  rms(m)  clkbias(m)\n");
    for (double S = slo; S < shi && nfix < 60; S += FS) {
        double ttx[MAX_CH], tmax = 0;
        for (int i = 0; i < nuse; i++) {
            chan_t *c = use[i];
            int e = find_epoch(c, S);
            double frac = (S - c->epos[e]) / (c->epos[e+1] - c->epos[e]);
            ttx[i] = c->t0 + ((e - c->e0) + frac) * 0.001;
            if (ttx[i] > tmax) tmax = ttx[i];
        }
        double trx = tmax + 0.070, sp[MAX_CH][3], rho[MAX_CH];
        for (int i = 0; i < nuse; i++) {
            double dtsv;
            sat_state(&use[i]->eph, ttx[i], sp[i], &dtsv);
            rho[i] = C_LIGHT * (trx - ttx[i]) + C_LIGHT * dtsv;
        }
        double x[4], rms, lat, lon, h;
        if (!lsq_fix(nuse, sp, rho, x, &rms)) continue;
        ecef2llh(x, &lat, &lon, &h);
        if (h < -1000 || h > 20000) continue;
        printf("%8.1f  %12.6f %12.6f %9.1f  %4d  %6.1f  %10.1f\n",
               (S - slo) / FS, lat * 180 / PI, lon * 180 / PI, h, nuse, rms, x[3]);
        sumlat += lat; sumlon += lon; sumh += h; nfix++;
    }
    if (nfix == 0) { printf("No valid fix.\n"); return 4; }
    printf("\nMean of %d fixes: lat %.6f  lon %.6f  alt %.1f m\n",
           nfix, sumlat / nfix * 180 / PI, sumlon / nfix * 180 / PI, sumh / nfix);
    printf("Google Maps: https://www.google.com/maps?q=%.6f,%.6f\n",
           sumlat / nfix * 180 / PI, sumlon / nfix * 180 / PI);
    return 0;
}

/* ------------------------- main ---------------------------------- */
int main(int argc, char **argv)
{
    stream_t s;
    int streaming;

    if (argc >= 3 && strcmp(argv[1], "--tail") == 0) {
        /* --- tail mode: read growing file written by GNURadio --- */
        if (!stream_open_tail(&s, argv[2])) {
            fprintf(stderr, "cannot open %s\n", argv[2]);
            return 1;
        }
        fprintf(stderr, "Tail mode: reading growing file %s at %.3f MS/s\n"
                        "  (start GNURadio recording, then this will follow)\n",
                argv[2], FS / 1e6);
        streaming = 1;
    } else if (argc >= 2 && strcmp(argv[1], "-") != 0) {
        /* --- file mode (original batch behavior) --- */
        if (!stream_open_file(&s, argv[1])) {
            fprintf(stderr, "cannot open %s\n", argv[1]);
            return 1;
        }
        fprintf(stderr, "File: %.1f s of data at %.3f MS/s assumed\n",
                s.total / FS, FS / 1e6);
        streaming = 0;
    } else {
        /* --- streaming mode (stdin) --- */
        if (!stream_open_stdin(&s)) {
            fprintf(stderr, "cannot allocate stream buffer\n");
            return 1;
        }
        fprintf(stderr, "Streaming mode: reading complex float32 IQ from stdin "
                        "at %.3f MS/s\n", FS / 1e6);
        streaming = 1;
    }

    /* ---- Phase 1: wait for enough data and acquire ---- */
    long long acq_need = (long long)(ACQ_START_S * FS) + (long long)ACQ_MS * NS;
    fprintf(stderr, "Waiting for %.1f s of data for acquisition...\n",
            acq_need / FS);
    if (!stream_fill(&s, acq_need)) {
        fprintf(stderr, "not enough data for acquisition (need %.1f s)\n",
                acq_need / FS);
        return 1;
    }

    long long start = (long long)(ACQ_START_S * FS);
    acq_t acq[64];
    int nacq = acquire(&s, start, acq);
    fprintf(stderr, "Acquired %d satellites:\n", nacq);
    for (int i = 0; i < nacq; i++)
        fprintf(stderr, "  PRN %2d  ratio %5.1f  doppler %7.0f Hz  codephase %4d\n",
                acq[i].prn, acq[i].ratio, acq[i].dop, acq[i].cp);
    if (nacq < 4) {
        fprintf(stderr, "Fewer than 4 satellites found.\n");
        return 2;
    }

    /* ---- Phase 2: initialize tracking channels ---- */
    int nch = 0;
    chan_t ch[MAX_CH];
    memset(ch, 0, sizeof(ch));
    for (int i = 0; i < nacq && nch < MAX_CH; i++) {
        ch[nch].prn = acq[i].prn;
        track_init(&ch[nch], start + acq[i].cp, acq[i].dop);
        nch++;
    }

    if (!streaming) {
        /* ---- File mode: batch track, then decode, then PVT ---- */
        int good = 0;
        for (int i = 0; i < nch; i++) {
            while (ch[i].active) {
                int r = track_epoch(&s, &ch[i]);
                if (r <= 0) {
                    if (r == -1)
                        fprintf(stderr, "  PRN %2d: no carrier lock, dropped\n",
                                ch[i].prn);
                    break;
                }
            }
            if (!ch[i].active && !ch[i].locked) continue;
            fprintf(stderr, "  PRN %2d: tracked %d ms, final carrier %.1f Hz\n",
                    ch[i].prn, ch[i].nep, ch[i].fcar);
            decode_nav(&ch[i]);
            fprintf(stderr, "  PRN %2d: %d valid subframes, ephemeris %s\n",
                    ch[i].prn, ch[i].nsf, ch[i].eph.valid ? "OK" : "incomplete");
            if (ch[i].locked) good++;
        }
        return run_pvt(ch, nch);
    }

    /* ---- Phase 3: streaming processing loop ---- */
    fprintf(stderr, "\nStreaming: tracking %d satellites, waiting for ephemeris...\n"
                    "  (first fix typically takes 30-90 s)\n\n", nch);

    printf("     lat(deg)     lon(deg)    alt(m)  nsat  rms(m)  clkbias(m)\n");
    fflush(stdout);

    int last_nav_epoch = 0;
    int last_pvt_epoch = 0;
    int last_status_epoch = 0;
    int nfixes = 0;
    double sumlat = 0, sumlon = 0, sumh = 0;
    int had_first_fix = 0;

    for (;;) {
        int any_active = 0;
        long long min_n = LLONG_MAX;

        /* advance each channel by one epoch */
        for (int i = 0; i < nch; i++) {
            if (!ch[i].active) continue;
            int r = track_epoch(&s, &ch[i]);
            if (r <= 0) {
                if (r == -1)
                    fprintf(stderr, "  PRN %2d: no carrier lock, dropped\n",
                            ch[i].prn);
                if (r == 0 && s.eof) ch[i].active = 0;
                continue;
            }
            any_active = 1;
            if (ch[i].n < min_n) min_n = ch[i].n;
        }

        if (!any_active) break;

        /* use first active channel's epoch count as time reference */
        int ref_epoch = 0;
        for (int i = 0; i < nch; i++)
            if (ch[i].active) { ref_epoch = ch[i].nep; break; }

        /* trim old samples to keep memory bounded */
        if (min_n > NS * 4)
            stream_trim(&s, min_n - NS * 4);

        /* periodic status update (every 5 seconds) */
        if (ref_epoch - last_status_epoch >= 5000) {
            last_status_epoch = ref_epoch;
            int nactive = 0, nlocked = 0, neph = 0;
            for (int i = 0; i < nch; i++) {
                if (ch[i].active) nactive++;
                if (ch[i].active && ch[i].locked) nlocked++;
                if (ch[i].active && ch[i].eph.valid) neph++;
            }
            fprintf(stderr, "[%5.1f s] active=%d locked=%d ephemeris=%d fixes=%d\n",
                    ref_epoch / 1000.0, nactive, nlocked, neph, nfixes);
        }

        /* try nav decode every 2 seconds (only for channels without ephemeris) */
        if (ref_epoch - last_nav_epoch >= 2000) {
            last_nav_epoch = ref_epoch;
            for (int i = 0; i < nch; i++) {
                if (!ch[i].active || !ch[i].locked || ch[i].eph.valid) continue;
                if (ch[i].nep < FLL_MS + 7000) continue;
                decode_nav(&ch[i]);
                if (ch[i].eph.valid)
                    fprintf(stderr, "  PRN %2d: ephemeris complete (%d subframes)\n",
                            ch[i].prn, ch[i].nsf);
            }
        }

        /* try PVT every second */
        if (ref_epoch - last_pvt_epoch >= 1000) {
            last_pvt_epoch = ref_epoch;
            if (pvt_snapshot(ch, nch, &nfixes, &sumlat, &sumlon, &sumh)) {
                if (!had_first_fix) {
                    fprintf(stderr, ">>> First fix at %.1f s <<<\n",
                            ref_epoch / 1000.0);
                    had_first_fix = 1;
                }
            }
        }
    }

    /* final summary */
    if (nfixes > 0) {
        printf("\nMean of %d fixes: lat %.6f  lon %.6f  alt %.1f m\n",
               nfixes,
               sumlat / nfixes * 180 / PI,
               sumlon / nfixes * 180 / PI,
               sumh / nfixes);
        printf("Google Maps: https://www.google.com/maps?q=%.6f,%.6f\n",
               sumlat / nfixes * 180 / PI,
               sumlon / nfixes * 180 / PI);
    } else {
        fprintf(stderr, "No valid fixes obtained.\n");
    }

    return nfixes > 0 ? 0 : 4;
}
