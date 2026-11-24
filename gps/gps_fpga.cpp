/*
 * gps_fpga.cpp - Host controller for FPGA GPS L1 C/A correlator
 *
 * Drives the gps_correlator RTL on LimeSDR Mini v2 via LiteX CSR registers.
 * The FPGA handles correlation, acquisition, and tracking loop closure.
 * This program manages channel assignment, nav data demodulation, and PVT.
 *
 * Access method: LimeSuiteNG ICSR (direct USB, no CLI tools).
 * Device init configures RX at 1575.42 MHz / 4.096 MS/s and starts streaming
 * to keep the LMS7002M MCLK1 alive (GPS correlator runs in that clock domain).
 *
 * Build (on Pi):
 *   g++ -O2 -o gps_fpga gps_fpga.cpp -llimesuiteng -lm -lpthread
 *
 * Usage:
 *   ./gps_fpga [-v|-vv]
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>

#include <limesuiteng/DeviceHandle.h>
#include <limesuiteng/DeviceRegistry.h>
#include <limesuiteng/SDRDevice.h>
#include <limesuiteng/SDRConfig.h>
#include <limesuiteng/StreamConfig.h>
#include <limesuiteng/StreamMeta.h>
#include <limesuiteng/RFStream.h>
#include <limesuiteng/ICSR.h>
#include <limesuiteng/complex.h>
#include <limesuiteng/types.h>

#include <memory>

/* ======================== Configuration ======================== */

#define FS            4096000.0
#define NS            4096
#define F_L1          1575.42e6
#define CODE_RATE     1.023e6
#define C_LIGHT       299792458.0
#define OMEGA_E       7.2921151467e-5
#define MU            3.986005e14
#define PI            3.14159265358979323846

#define MAX_CH        8
#define MAX_PRN       32

/* Acquisition: Doppler search range and step (Hz) */
#define ACQ_DOPP_MAX  40000
#define ACQ_DOPP_STEP 500
#define ACQ_THRESH    500       /* minimum peak power to declare detection */

/* Two-tier acquisition: coarse sweep finds candidates, fine sweep refines */
#define ACQ_COARSE_EPOCHS  20
#define ACQ_COARSE_STEP    1000
#define ACQ_COARSE_THRESH  100   /* ~linear with epochs: 500 * 20/40 = 250, lowered for weak signals */
#define ACQ_FINE_EPOCHS    40
#define ACQ_FINE_STEP      250
#define ACQ_FINE_RANGE     500   /* ±Hz around coarse hit */

/* Tracking */
#define LOCK_WINDOW   200       /* epochs to accumulate for lock check */
#define NAV_POLL_MS   20        /* poll IP every 20 ms (one nav bit) */

/* Coasting and reacquisition */
#define COAST_TIMEOUT_MS  2000  /* coast this long before assisted reacq */
#define REACQ_DOPP_MAX    1000  /* narrow Doppler search for assisted reacq (Hz) */
#define REACQ_DOPP_STEP   250   /* finer step for narrow search */
#define COLD_REACQ_INTERVAL_MS 10000  /* try cold reacq on free channels every 10s */

/* NCO frequency words: carrier_freq = doppler_hz * 2^32 / Fs */
#define HZ_TO_NCO(hz)   ((int32_t)((double)(hz) * 4294967296.0 / FS))
#define NCO_TO_HZ(nco)  ((double)(nco) * FS / 4294967296.0)

/* Code NCO nominal: 1023 chips / 4096 samples → code_freq = 1023/4096 * 2^32 */
#define CODE_FREQ_NOM 0x3FF00000u

/* Loop filter gains (Q16.16 fixed-point × normalized discriminator)
 * PLL: 2nd order, BN=15 Hz, zeta=0.707 → wn ≈ 28.3 rad/s
 *   proportional: 2*zeta*wn / (2*pi) * T * 2^16 ≈ 418
 *   integral:     wn^2 / (2*pi) * T^2 * 2^16 ≈ 8
 * FLL: cross/dot discriminator, applied during first FLL_EPOCHS epochs
 *   gain = 0.05 * 2^16 / Fs ≈ 800 (conservative pull-in)
 * DLL: 1st order, cod_gain = 84000
 */
#define DEFAULT_CAR_GAIN_P  420
#define DEFAULT_CAR_GAIN_I  8
#define DEFAULT_COD_GAIN    84000
#define DEFAULT_FLL_GAIN    800
#define DEFAULT_FLL_EPOCHS  400

/* ======================== LimeSDR Device Layer ======================== */

static lime::SDRDevice *sdr = nullptr;
static std::unique_ptr<lime::RFStream> rfstream;
static pthread_t drain_tid;
static volatile int running = 1;
static int verbose = 0;

/* LiteX CSR addresses from gateware build (v3.11 + GPS correlator) */
static const uint64_t CSR_GPS_ADDR  = 0xf0001800;
static const uint64_t CSR_GPS_WDATA = 0xf0001804;
static const uint64_t CSR_GPS_RDATA = 0xf0001808;
static const uint64_t CSR_GPS_WEN   = 0xf000180c;

static void csr_write(uint64_t addr, uint32_t value)
{
    if (verbose > 1) fprintf(stderr, "  CSR W [%08lx] = %08x\n", (unsigned long)addr, value);
    lime::ICSR *icsr = sdr->getICSR();
    if (icsr) icsr->ioWrite64(addr, (uint64_t)value);
}

static uint32_t csr_read(uint64_t addr)
{
    lime::ICSR *icsr = sdr->getICSR();
    uint32_t val = 0;
    if (icsr) val = (uint32_t)icsr->ioRead64(addr);
    if (verbose > 1) fprintf(stderr, "  CSR R [%08lx] = %08x\n", (unsigned long)addr, val);
    return val;
}

/* Background thread: drain RX samples to keep MCLK1 alive */
static void *rx_drain_thread(void *arg)
{
    (void)arg;
    lime::complex16_t buf[4096];
    lime::complex16_t *chptrs[1] = { buf };
    lime::StreamMeta meta;
    while (running) {
        rfstream->StreamRx(chptrs, 4096, &meta);
    }
    return nullptr;
}

static int device_init(void)
{
    auto handles = lime::DeviceRegistry::enumerate();
    if (handles.empty()) {
        fprintf(stderr, "Error: no LimeSDR devices found\n");
        return -1;
    }
    fprintf(stderr, "Found %zu LimeSDR device(s)\n", handles.size());

    sdr = lime::DeviceRegistry::makeDevice(handles.at(0));
    if (!sdr) {
        fprintf(stderr, "Error: failed to open LimeSDR\n");
        return -1;
    }

    if (sdr->Init() != lime::OpStatus::Success) {
        fprintf(stderr, "Error: device init failed\n");
        lime::DeviceRegistry::freeDevice(sdr);
        sdr = nullptr;
        return -1;
    }

    /* Configure RX channel 0: GPS L1 C/A */
    lime::SDRConfig config;
    config.channel[0].rx.enabled = true;
    config.channel[0].rx.centerFrequency = F_L1;
    config.channel[0].rx.sampleRate = FS;
    config.channel[0].rx.lpf = 2.5e6;
    config.channel[0].rx.oversample = 0; /* auto */
    config.channel[0].rx.path = 3;       /* LNAW (1=LNAH, 2=LNAL, 3=LNAW) */

    if (sdr->Configure(config, 0) != lime::OpStatus::Success) {
        fprintf(stderr, "Error: RX configuration failed\n");
        lime::DeviceRegistry::freeDevice(sdr);
        sdr = nullptr;
        return -1;
    }

    /* Create RX stream — this enables MCLK1 from LMS7002M */
    lime::StreamConfig scfg;
    scfg.channels[lime::TRXDir::Rx] = {0};
    scfg.format = lime::DataFormat::I16;
    scfg.bufferSize = 4096 * 16;

    rfstream = sdr->StreamCreate(scfg, 0);
    if (!rfstream) {
        fprintf(stderr, "Error: stream creation failed\n");
        lime::DeviceRegistry::freeDevice(sdr);
        sdr = nullptr;
        return -1;
    }

    if (rfstream->Start() != lime::OpStatus::Success) {
        fprintf(stderr, "Error: stream start failed\n");
        rfstream.reset();
        lime::DeviceRegistry::freeDevice(sdr);
        sdr = nullptr;
        return -1;
    }

    /* Drain thread keeps USB flowing so MCLK stays alive */
    pthread_create(&drain_tid, nullptr, rx_drain_thread, nullptr);

    fprintf(stderr, "LimeSDR ready: RX @ %.2f MHz, %.3f MS/s, MCLK1 active\n",
            F_L1 / 1e6, FS / 1e6);
    return 0;
}

static void device_close(void)
{
    if (!sdr) return;
    if (rfstream) {
        rfstream->Stop();
        pthread_join(drain_tid, nullptr);
        rfstream.reset();
    }
    lime::DeviceRegistry::freeDevice(sdr);
    sdr = nullptr;
}

/* ======================== GPS Register Interface ======================== */

/* GPS correlator register map:
 * Address = {channel[3:0], register[2:0]}
 * Channels 0-7: per-channel, 12: gains, 15: global/acquisition */

#define GPS_REG(ch, reg)  (((ch) << 3) | (reg))

/* Per-channel registers */
#define REG_CTRL      0
#define REG_CAR_FREQ  1
#define REG_COD_FREQ  2
#define REG_COD_PHASE 3  /* write: code phase preset */
#define REG_IP        3  /* read: I-prompt */
#define REG_QP        4
#define REG_IE_QE     5
#define REG_IL_QL     6
#define REG_STATUS    7

/* Channel 12: loop filter gains */
#define CH_GAIN       12
#define REG_CAR_GAIN_P 0
#define REG_CAR_GAIN_I 1
#define REG_COD_GAIN   2
#define REG_FLL_GAIN   3

/* Channel 13: FLL/aiding config */
#define CH_FLL_CFG    13
#define REG_FLL_EPOCHS 0

/* Channel 15: global / acquisition */
#define CH_GLOBAL     15
#define REG_GLOB_CTRL 0
#define REG_EPOCH_CNT 1
#define REG_VERSION   2
#define REG_ACQ_CTRL  3
#define REG_ACQ_CFG   4
#define REG_ACQ_CAR   5
#define REG_ACQ_POWER 6

static void gps_write(int ch, int reg, uint32_t value)
{
    uint32_t addr = GPS_REG(ch, reg);
    csr_write(CSR_GPS_ADDR, addr);
    csr_write(CSR_GPS_WDATA, value);
    csr_write(CSR_GPS_WEN, 1);
    usleep(200);  /* CDC settling */
}

static uint32_t gps_read(int ch, int reg)
{
    uint32_t addr = GPS_REG(ch, reg);
    csr_write(CSR_GPS_ADDR, addr);
    usleep(500);  /* CDC + MultiReg settling */
    return csr_read(CSR_GPS_RDATA);
}

/* ======================== Channel State ======================== */

typedef struct {
    double tgd, toc, af0, af1, af2;
    double m0, ecc, sqrta, toe, omg0, omg, omgd, i0, idot, dn;
    double crs, crc, cus, cuc, cis, cic;
    int week, health, iodc, iode2, iode3;
    int have1, have2, have3, valid;
} eph_t;

typedef struct {
    int prn;
    int active;
    int locked;
    int track_ms;
    int coasting;        /* 1 = lock lost, channel still running */
    int coast_ms;        /* ms since lock was lost */
    int32_t last_car_nco; /* last known carrier freq for assisted reacq */
    int was_locked;      /* had lock at some point (vs never locked) */

    /* nav data demodulation */
    int32_t ip_hist[20];     /* IP sign accumulator for current nav bit */
    int     ip_idx;          /* index within 20-epoch nav bit */
    int     bit_count;       /* total nav bits demodulated */
    char    bits[2000];      /* raw nav bit stream */

    /* lock detection */
    int32_t sum_abs_ip;
    int32_t sum_abs_qp;
    int     lock_cnt;

    /* timing */
    int     have_t0;
    double  t0;              /* GPS time at bit_ref epoch */
    int     bit_ref;         /* bit index where t0 was established */
    uint32_t epoch_start;    /* global epoch counter when tracking started */
    uint32_t epoch_at_t0;    /* global epoch counter at the t0 reference point */

    /* ephemeris */
    eph_t   eph;
    int     nsf;
} chan_t;

static chan_t channels[MAX_CH];

/* PRNs waiting for cold reacquisition on a free channel */
static int reacq_queue[MAX_PRN];
static int reacq_count = 0;

static void reacq_enqueue(int prn)
{
    for (int i = 0; i < reacq_count; i++)
        if (reacq_queue[i] == prn) return;
    if (reacq_count < MAX_PRN)
        reacq_queue[reacq_count++] = prn;
}

static int reacq_dequeue(void)
{
    if (reacq_count == 0) return 0;
    int prn = reacq_queue[0];
    reacq_count--;
    for (int i = 0; i < reacq_count; i++)
        reacq_queue[i] = reacq_queue[i + 1];
    return prn;
}

static void sighandler(int sig) { (void)sig; running = 0; }

/* ======================== Acquisition ======================== */

typedef struct {
    int prn;
    int code_phase;
    int32_t doppler_nco;
    uint32_t power;
} acq_result_t;

static int acquire_prn(int prn, int32_t doppler_nco, int num_epochs,
                       acq_result_t *result)
{
    /* Configure acquisition engine */
    gps_write(CH_GLOBAL, REG_ACQ_CFG,
              ((uint32_t)prn << 16) | (num_epochs & 0x3F));
    gps_write(CH_GLOBAL, REG_ACQ_CAR, (uint32_t)doppler_nco);

    /* Start */
    gps_write(CH_GLOBAL, REG_ACQ_CTRL, 1);

    /* Poll for completion */
    for (int i = 0; i < 200; i++) {
        usleep(20000);
        uint32_t status = gps_read(CH_GLOBAL, REG_ACQ_CTRL);
        if (status & 0x80000000u) {  /* done bit */
            result->prn = prn;
            result->code_phase = status & 0x3FF;
            result->doppler_nco = doppler_nco;
            result->power = gps_read(CH_GLOBAL, REG_ACQ_POWER);
            return 1;
        }
    }
    return 0;  /* timeout */
}

static int acquire_all(acq_result_t results[], int max_results)
{
    typedef struct { int prn; int32_t doppler_nco; uint32_t power; } coarse_hit_t;
    coarse_hit_t coarse_hits[MAX_PRN];
    int nhits = 0, n = 0;

    /* Tier 1: coarse sweep — all PRNs, wide step, few epochs */
    int coarse_bins = 2 * ACQ_DOPP_MAX / ACQ_COARSE_STEP + 1;
    fprintf(stderr, "  Tier 1: coarse (%d epochs, %d Hz step, %d bins/PRN)\n",
            ACQ_COARSE_EPOCHS, ACQ_COARSE_STEP, coarse_bins);

    for (int prn = 1; prn <= MAX_PRN; prn++) {
        uint32_t best_power = 0;
        int32_t best_nco = 0;

        fprintf(stderr, "    PRN %2d: ", prn);

        for (int d = -ACQ_DOPP_MAX; d <= ACQ_DOPP_MAX; d += ACQ_COARSE_STEP) {
            int32_t nco = HZ_TO_NCO(d);
            acq_result_t r;
            if (acquire_prn(prn, nco, ACQ_COARSE_EPOCHS, &r)
                && r.power > best_power) {
                best_power = r.power;
                best_nco = nco;
            }
        }

        if (best_power > ACQ_COARSE_THRESH) {
            fprintf(stderr, "HIT  power=%u  doppler=%.0f Hz\n",
                    best_power, NCO_TO_HZ(best_nco));
            coarse_hits[nhits].prn = prn;
            coarse_hits[nhits].doppler_nco = best_nco;
            coarse_hits[nhits].power = best_power;
            nhits++;
        } else {
            fprintf(stderr, "---  (peak=%u)\n", best_power);
        }
    }

    fprintf(stderr, "  Tier 1 done: %d candidate(s)\n\n", nhits);

    /* Tier 2: fine sweep — detected PRNs only, narrow range, full epochs */
    int fine_bins = 2 * ACQ_FINE_RANGE / ACQ_FINE_STEP + 1;
    fprintf(stderr, "  Tier 2: fine (%d epochs, %d Hz step, +/-%d Hz, %d bins/PRN)\n",
            ACQ_FINE_EPOCHS, ACQ_FINE_STEP, ACQ_FINE_RANGE, fine_bins);

    for (int i = 0; i < nhits && n < max_results; i++) {
        uint32_t best_power = 0;
        acq_result_t best = {};

        fprintf(stderr, "    PRN %2d: ", coarse_hits[i].prn);

        for (int d = -ACQ_FINE_RANGE; d <= ACQ_FINE_RANGE; d += ACQ_FINE_STEP) {
            int32_t nco = coarse_hits[i].doppler_nco + HZ_TO_NCO(d);
            acq_result_t r;
            if (acquire_prn(coarse_hits[i].prn, nco, ACQ_FINE_EPOCHS, &r)
                && r.power > best_power) {
                best_power = r.power;
                best = r;
            }
        }

        if (best_power > ACQ_THRESH) {
            fprintf(stderr, "FOUND  power=%u  doppler=%.0f Hz  phase=%d\n",
                    best_power, NCO_TO_HZ(best.doppler_nco), best.code_phase);
            results[n++] = best;
        } else {
            fprintf(stderr, "LOST  (fine peak=%u)\n", best_power);
        }
    }

    return n;
}

/* Assisted reacquisition: narrow Doppler search around last known freq */
static int acquire_assisted(int prn, int32_t last_nco, acq_result_t *result)
{
    uint32_t best_power = 0;
    acq_result_t best = {};

    for (int d = -REACQ_DOPP_MAX; d <= REACQ_DOPP_MAX; d += REACQ_DOPP_STEP) {
        int32_t nco = last_nco + HZ_TO_NCO(d);
        acq_result_t r;
        if (acquire_prn(prn, nco, ACQ_FINE_EPOCHS, &r) && r.power > best_power) {
            best_power = r.power;
            best = r;
        }
    }

    if (best_power > ACQ_THRESH) {
        *result = best;
        return 1;
    }
    return 0;
}

/* ======================== Tracking Management ======================== */

static void set_loop_gains(int32_t car_p, int32_t car_i, int32_t cod,
                           int32_t fll, int32_t fll_ep)
{
    gps_write(CH_GAIN, REG_CAR_GAIN_P, (uint32_t)car_p);
    gps_write(CH_GAIN, REG_CAR_GAIN_I, (uint32_t)car_i);
    gps_write(CH_GAIN, REG_COD_GAIN, (uint32_t)cod);
    gps_write(CH_GAIN, REG_FLL_GAIN, (uint32_t)fll);
    gps_write(CH_FLL_CFG, REG_FLL_EPOCHS, (uint32_t)fll_ep);
}

static void start_tracking(int ch, const acq_result_t *acq)
{
    chan_t *c = &channels[ch];
    memset(c, 0, sizeof(*c));
    c->prn = acq->prn;
    c->active = 1;
    c->epoch_start = gps_read(CH_GLOBAL, REG_EPOCH_CNT);

    /* Set channel: PRN, carrier freq, code freq, code phase */
    uint32_t ctrl = ((uint32_t)(acq->prn & 0x1F)) | (1 << 5);  /* enable, no track yet */
    gps_write(ch, REG_CTRL, ctrl);
    gps_write(ch, REG_CAR_FREQ, (uint32_t)acq->doppler_nco);
    gps_write(ch, REG_COD_FREQ, CODE_FREQ_NOM);
    gps_write(ch, REG_COD_PHASE, acq->code_phase & 0x3FF);

    /* Wait a few epochs for accumulators to fill */
    usleep(5000);

    /* Enable tracking mode (bit 6) */
    ctrl |= (1 << 6);
    gps_write(ch, REG_CTRL, ctrl);

    fprintf(stderr, "  Ch %2d: PRN %2d tracking started (Doppler %.0f Hz, phase %d)\n",
            ch, acq->prn, NCO_TO_HZ(acq->doppler_nco), acq->code_phase);
}

static void stop_tracking(int ch)
{
    chan_t *c = &channels[ch];
    gps_write(ch, REG_CTRL, 0);
    fprintf(stderr, "  Ch %2d: PRN %2d stopped (%d ms tracked, %d nav bits)\n",
            ch, c->prn, c->track_ms, c->bit_count);
    reacq_enqueue(c->prn);
    c->active = 0;
}

static void attempt_reacq(int ch)
{
    chan_t *c = &channels[ch];
    fprintf(stderr, "  Ch %2d: PRN %2d coast timeout, trying assisted reacq (+/-%d Hz)...\n",
            ch, c->prn, REACQ_DOPP_MAX);

    gps_write(ch, REG_CTRL, 0);

    acq_result_t r;
    if (acquire_assisted(c->prn, c->last_car_nco, &r)) {
        fprintf(stderr, "  Ch %2d: PRN %2d reacquired (power=%u, doppler=%.0f Hz)\n",
                ch, c->prn, r.power, NCO_TO_HZ(r.doppler_nco));
        int saved_bit_count = c->bit_count;
        int saved_nsf = c->nsf;
        eph_t saved_eph = c->eph;
        int saved_have_t0 = c->have_t0;
        double saved_t0 = c->t0;
        int saved_bit_ref = c->bit_ref;
        uint32_t saved_epoch_at_t0 = c->epoch_at_t0;

        start_tracking(ch, &r);

        /* Preserve nav data and ephemeris from before the outage */
        c->bit_count = saved_bit_count;
        c->nsf = saved_nsf;
        c->eph = saved_eph;
        c->have_t0 = saved_have_t0;
        c->t0 = saved_t0;
        c->bit_ref = saved_bit_ref;
        c->epoch_at_t0 = saved_epoch_at_t0;
        c->was_locked = 1;
    } else {
        fprintf(stderr, "  Ch %2d: PRN %2d assisted reacq failed, dropping\n",
                ch, c->prn);
        stop_tracking(ch);
    }
}

/* ======================== Nav Data Demodulation ======================== */

static void process_ip_sample(chan_t *c, int32_t ip)
{
    c->ip_hist[c->ip_idx] += (ip > 0) ? 1 : -1;
    c->ip_idx++;

    if (c->ip_idx >= 20) {
        /* One nav bit complete (20 epochs = 20 ms) */
        int sum = 0;
        for (int i = 0; i < 20; i++) sum += c->ip_hist[i];

        if (c->bit_count < 2000)
            c->bits[c->bit_count] = (sum > 0) ? 1 : 0;
        c->bit_count++;

        memset(c->ip_hist, 0, sizeof(c->ip_hist));
        c->ip_idx = 0;
    }
}

/* ======================== Lock Detection ======================== */

static void update_lock(chan_t *c, int32_t ip, int32_t qp)
{
    int32_t aip = ip < 0 ? -ip : ip;
    int32_t aqp = qp < 0 ? -qp : qp;
    c->sum_abs_ip += aip;
    c->sum_abs_qp += aqp;
    c->lock_cnt++;

    if (c->lock_cnt >= LOCK_WINDOW) {
        c->locked = (c->sum_abs_ip >= 2 * c->sum_abs_qp);
        c->sum_abs_ip = 0;
        c->sum_abs_qp = 0;
        c->lock_cnt = 0;
    }
}

/* ======================== Nav Message Parsing ======================== */

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

static int64_t gets_(const int *d, int start, int len)
{ return signext(getu(d, start, len), len); }

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

static void try_decode_nav(chan_t *c)
{
    if (c->bit_count < 310) return;

    static const int pre[8] = {1,0,0,0,1,0,1,1};
    int d[301];

    for (int b = 2; b + 300 <= c->bit_count; b++) {
        int m = 1, v = 1;
        for (int j = 0; j < 8; j++) {
            if (c->bits[b+j] != pre[j]) m = 0;
            if (c->bits[b+j] == pre[j]) v = 0;
        }
        if (!m && !v) continue;
        if (!parity_subframe(c->bits + b, c->bits[b-2], c->bits[b-1], d)) continue;
        int id = (int)getu(d, 50, 3);
        if (id < 1 || id > 3) continue;

        long tow = (long)getu(d, 31, 17);
        if (!c->have_t0) {
            c->t0 = tow * 6.0 - 6.0;
            c->bit_ref = b;
            c->epoch_at_t0 = c->epoch_start + (uint32_t)(b * 20);
            c->have_t0 = 1;
        }

        parse_subframe(d, id, &c->eph);
        c->nsf++;

        if (c->eph.valid) {
            fprintf(stderr, "  PRN %2d: ephemeris complete (%d subframes)\n",
                    c->prn, c->nsf);
            return;
        }
    }
}

/* ======================== Satellite Position ======================== */

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

/* ======================== Position Solution ======================== */

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
    x[0] = x[1] = x[2] = x[3] = 0;
    for (int iter = 0; iter < 10; iter++) {
        double H[MAX_CH][4], y[MAX_CH], N[4][5] = {};
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
    double la = atan2(x[2], p * (1 - e2)), N_ = a;
    for (int i = 0; i < 10; i++) {
        N_ = a / sqrt(1 - e2 * sin(la) * sin(la));
        *h = p / cos(la) - N_;
        la = atan2(x[2], p * (1 - e2 * N_ / (N_ + *h)));
    }
    *lat = la;
}

static int try_pvt(void)
{
    int nuse = 0;
    int idx[MAX_CH];
    for (int i = 0; i < MAX_CH; i++)
        if (channels[i].active && channels[i].locked &&
            channels[i].eph.valid && channels[i].have_t0)
            idx[nuse++] = i;

    if (nuse < 4) return 0;

    double sp[MAX_CH][3], rho[MAX_CH];
    double ttx[MAX_CH], tmax = 0;

    /* Read global epoch counter once — consistent across all channels */
    uint32_t epoch_now = gps_read(CH_GLOBAL, REG_EPOCH_CNT);

    for (int i = 0; i < nuse; i++) {
        chan_t *c = &channels[idx[i]];
        ttx[i] = c->t0 + (epoch_now - c->epoch_at_t0) * 0.001;
        if (ttx[i] > tmax) tmax = ttx[i];
    }

    double trx = tmax + 0.070;
    for (int i = 0; i < nuse; i++) {
        double dtsv;
        sat_state(&channels[idx[i]].eph, ttx[i], sp[i], &dtsv);
        rho[i] = C_LIGHT * (trx - ttx[i]) + C_LIGHT * dtsv;
    }

    double x[4], rms, lat, lon, h;
    if (!lsq_fix(nuse, sp, rho, x, &rms)) return 0;
    ecef2llh(x, &lat, &lon, &h);
    if (h < -1000 || h > 20000) return 0;

    printf("%12.6f %12.6f %9.1f  %4d  %6.1f  %10.1f\n",
           lat * 180 / PI, lon * 180 / PI, h, nuse, rms, x[3]);
    fflush(stdout);
    return 1;
}

/* ======================== Main ======================== */

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0)
            verbose++;
        else if (strcmp(argv[i], "-vv") == 0)
            verbose = 2;
        else {
            fprintf(stderr, "Usage: %s [-v|-vv]\n", argv[0]);
            return 1;
        }
    }

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);

    /* Open LimeSDR, configure RX, start streaming */
    if (device_init() != 0) return 1;

    fprintf(stderr, "CSR addresses (hardcoded from gateware build):\n");
    fprintf(stderr, "  gps_addr  = 0x%08lx\n", (unsigned long)CSR_GPS_ADDR);
    fprintf(stderr, "  gps_wdata = 0x%08lx\n", (unsigned long)CSR_GPS_WDATA);
    fprintf(stderr, "  gps_rdata = 0x%08lx\n", (unsigned long)CSR_GPS_RDATA);
    fprintf(stderr, "  gps_wen   = 0x%08lx\n", (unsigned long)CSR_GPS_WEN);

    /* Verify FPGA GPS correlator is present */
    uint32_t ver = gps_read(CH_GLOBAL, REG_VERSION);
    fprintf(stderr, "GPS correlator version: 0x%08x", ver);
    if ((ver & 0xFFFFFF00) == 0x47505300) {
        fprintf(stderr, " (GPS v%d)\n", ver & 0xFF);
    } else {
        fprintf(stderr, " -- unexpected! Expected 0x475053xx\n");
        device_close();
        return 1;
    }

    /* Enable global correlator */
    gps_write(CH_GLOBAL, REG_GLOB_CTRL, 1);
    fprintf(stderr, "GPS correlator enabled.\n");

    /* Set loop filter gains */
    set_loop_gains(DEFAULT_CAR_GAIN_P, DEFAULT_CAR_GAIN_I, DEFAULT_COD_GAIN,
                   DEFAULT_FLL_GAIN, DEFAULT_FLL_EPOCHS);
    fprintf(stderr, "Loop gains: car_p=%d car_i=%d cod=%d fll=%d fll_epochs=%d\n",
            DEFAULT_CAR_GAIN_P, DEFAULT_CAR_GAIN_I, DEFAULT_COD_GAIN,
            DEFAULT_FLL_GAIN, DEFAULT_FLL_EPOCHS);

    /* ---- Phase 1: Acquisition ---- */
    fprintf(stderr, "\n=== Acquisition ===\n");
    acq_result_t acq_results[MAX_CH];
    int nacq = acquire_all(acq_results, MAX_CH);

    if (nacq == 0) {
        fprintf(stderr, "No satellites found.\n");
        device_close();
        return 2;
    }
    fprintf(stderr, "Acquired %d satellite(s).\n\n", nacq);

    /* ---- Phase 2: Start tracking ---- */
    fprintf(stderr, "=== Tracking ===\n");
    for (int ch = 0; ch < nacq; ch++)
        start_tracking(ch, &acq_results[ch]);

    /* ---- Phase 3: Main loop ---- */
    fprintf(stderr, "\n=== Running (Ctrl+C to stop) ===\n");
    fprintf(stderr, "Polling channels for nav data...\n\n");
    printf("     lat(deg)     lon(deg)    alt(m)  nsat  rms(m)  clkbias(m)\n");
    fflush(stdout);

    int loop_ms = 0;
    int last_nav_ms = 0;
    int last_pvt_ms = 0;
    int last_status_ms = 0;
    int last_reacq_ms = 0;
    int nfixes = 0;

    while (running) {
        /* Poll all active channels: read IP, QP */
        for (int ch = 0; ch < MAX_CH; ch++) {
            chan_t *c = &channels[ch];
            if (!c->active) continue;

            int32_t ip = (int32_t)gps_read(ch, REG_IP);
            int32_t qp = (int32_t)gps_read(ch, REG_QP);

            /* Nav data demodulation */
            process_ip_sample(c, ip);

            /* Lock detection */
            update_lock(c, ip, qp);

            c->track_ms++;

            if (c->locked) {
                /* Save carrier freq while locked for assisted reacq */
                c->last_car_nco = (int32_t)gps_read(ch, REG_CAR_FREQ);
                c->was_locked = 1;
                if (c->coasting) {
                    fprintf(stderr, "  Ch %2d: PRN %2d lock regained after %d ms coast\n",
                            ch, c->prn, c->coast_ms);
                    c->coasting = 0;
                    c->coast_ms = 0;
                }
            } else if (c->was_locked && !c->coasting) {
                /* Just lost lock — start coasting */
                c->coasting = 1;
                c->coast_ms = 0;
                fprintf(stderr, "  Ch %2d: PRN %2d lock lost, coasting...\n",
                        ch, c->prn);
            }

            if (c->coasting) {
                c->coast_ms += NAV_POLL_MS;
                if (c->coast_ms >= COAST_TIMEOUT_MS) {
                    attempt_reacq(ch);
                    continue;
                }
            }

            /* Never locked at all after 2 seconds — initial acquisition was bad */
            if (c->track_ms > 2000 && !c->was_locked) {
                fprintf(stderr, "  PRN %2d: no lock after %d ms, dropping\n",
                        c->prn, c->track_ms);
                stop_tracking(ch);
            }
        }
        loop_ms += NAV_POLL_MS;

        /* Try nav decode every 2 seconds for channels without ephemeris */
        if (loop_ms - last_nav_ms >= 2000) {
            last_nav_ms = loop_ms;
            for (int ch = 0; ch < MAX_CH; ch++) {
                chan_t *c = &channels[ch];
                if (!c->active || !c->locked || c->eph.valid) continue;
                try_decode_nav(c);
            }
        }

        /* Try PVT every second */
        if (loop_ms - last_pvt_ms >= 1000) {
            last_pvt_ms = loop_ms;
            if (try_pvt()) nfixes++;
        }

        /* Status update every 5 seconds */
        if (loop_ms - last_status_ms >= 5000) {
            last_status_ms = loop_ms;
            int nact = 0, nlck = 0, neph = 0, ncoast = 0;
            for (int ch = 0; ch < MAX_CH; ch++) {
                if (channels[ch].active) nact++;
                if (channels[ch].active && channels[ch].locked) nlck++;
                if (channels[ch].active && channels[ch].eph.valid) neph++;
                if (channels[ch].active && channels[ch].coasting) ncoast++;
            }
            fprintf(stderr, "[%5.1f s] active=%d locked=%d coasting=%d eph=%d fixes=%d reacq_q=%d\n",
                    loop_ms / 1000.0, nact, nlck, ncoast, neph, nfixes, reacq_count);
        }

        /* Cold reacquisition on free channels for dropped PRNs */
        if (reacq_count > 0 && loop_ms - last_reacq_ms >= COLD_REACQ_INTERVAL_MS) {
            last_reacq_ms = loop_ms;
            int free_ch = -1;
            for (int ch = 0; ch < MAX_CH; ch++)
                if (!channels[ch].active) { free_ch = ch; break; }
            if (free_ch >= 0) {
                int prn = reacq_dequeue();
                fprintf(stderr, "  Cold reacq: PRN %2d on ch %d\n", prn, free_ch);
                acq_result_t r;
                uint32_t best_power = 0;
                acq_result_t best = {};
                for (int d = -ACQ_DOPP_MAX; d <= ACQ_DOPP_MAX;
                     d += ACQ_COARSE_STEP) {
                    int32_t nco = HZ_TO_NCO(d);
                    if (acquire_prn(prn, nco, ACQ_COARSE_EPOCHS, &r)
                        && r.power > best_power) {
                        best_power = r.power;
                        best = r;
                    }
                }
                if (best_power > ACQ_COARSE_THRESH) {
                    /* Refine with fine sweep */
                    uint32_t fine_power = 0;
                    acq_result_t fine = {};
                    for (int df = -ACQ_FINE_RANGE; df <= ACQ_FINE_RANGE;
                         df += ACQ_FINE_STEP) {
                        int32_t fnco = best.doppler_nco + HZ_TO_NCO(df);
                        acq_result_t fr;
                        if (acquire_prn(prn, fnco, ACQ_FINE_EPOCHS, &fr)
                            && fr.power > fine_power) {
                            fine_power = fr.power;
                            fine = fr;
                        }
                    }
                    if (fine_power > ACQ_THRESH) {
                        fprintf(stderr, "  Cold reacq: PRN %2d found (power=%u)\n",
                                prn, fine_power);
                        start_tracking(free_ch, &fine);
                    } else {
                        fprintf(stderr, "  Cold reacq: PRN %2d coarse hit but fine failed, re-queuing\n", prn);
                        reacq_enqueue(prn);
                    }
                } else {
                    fprintf(stderr, "  Cold reacq: PRN %2d not found, re-queuing\n", prn);
                    reacq_enqueue(prn);
                }
            }
        }

        usleep(1000);
    }

    /* Cleanup */
    fprintf(stderr, "\nShutting down...\n");
    for (int ch = 0; ch < MAX_CH; ch++)
        if (channels[ch].active)
            stop_tracking(ch);
    gps_write(CH_GLOBAL, REG_GLOB_CTRL, 0);
    device_close();

    if (nfixes > 0)
        fprintf(stderr, "Total fixes: %d\n", nfixes);
    else
        fprintf(stderr, "No position fixes obtained.\n");

    return nfixes > 0 ? 0 : 3;
}
