/*
 * diag_fft.cpp - Diagnose FFT acquisition engine hang
 *
 * Starts RX streaming, enables GPS correlator, starts acquisition on PRN 1,
 * and polls the debug register to see which FSM state it reaches/stalls in.
 *
 * Build: g++ -O2 -o diag_fft diag_fft.cpp -llimesuiteng -lpthread
 * Usage: ./diag_fft
 */

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <unistd.h>
#include <pthread.h>
#include <memory>

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

static const uint64_t CSR_GPS_ADDR  = 0xf0001800;
static const uint64_t CSR_GPS_WDATA = 0xf0001804;
static const uint64_t CSR_GPS_RDATA = 0xf0001808;
static const uint64_t CSR_GPS_WEN   = 0xf000180c;
static const uint64_t CSR_DBG_VALID = 0xf0001810;
static const uint64_t CSR_DBG_READY = 0xf0001814;
static const uint64_t CSR_DBG_BOTH  = 0xf0001818;
static const uint64_t CSR_DBG_RST   = 0xf000181c;

static lime::SDRDevice *sdr = nullptr;
static std::unique_ptr<lime::RFStream> rfstream;
static pthread_t drain_tid;
static volatile int running = 1;

static uint32_t csr_read(uint64_t addr)
{
    lime::ICSR *icsr = sdr->getICSR();
    if (icsr) return (uint32_t)icsr->ioRead64(addr);
    return 0;
}

static void csr_write(uint64_t addr, uint32_t value)
{
    lime::ICSR *icsr = sdr->getICSR();
    if (icsr) icsr->ioWrite64(addr, (uint64_t)value);
}

static uint32_t gps_read(uint8_t addr)
{
    csr_write(CSR_GPS_ADDR, addr);
    csr_write(CSR_GPS_WEN, 0);
    usleep(100);
    return csr_read(CSR_GPS_RDATA);
}

static void gps_write(uint8_t addr, uint32_t data)
{
    csr_write(CSR_GPS_ADDR, addr);
    csr_write(CSR_GPS_WDATA, data);
    usleep(100);
    csr_write(CSR_GPS_WEN, 1);
    usleep(100);
    csr_write(CSR_GPS_WEN, 0);
}

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

static const char *state_name(uint32_t s)
{
    static const char *names[] = {
        "IDLE", "GEN_CODE", "CLR_PWR", "CAPTURE",
        "FFT_SIG", "FFT_MUL", "FFT_IFFT", "NEXT_EPOCH",
        "FIND_BEST", "DONE"
    };
    if (s < 10) return names[s];
    return "UNKNOWN";
}

int main()
{
    auto handles = lime::DeviceRegistry::enumerate();
    if (handles.empty()) { fprintf(stderr, "No LimeSDR found\n"); return 1; }
    sdr = lime::DeviceRegistry::makeDevice(handles[0]);
    if (!sdr) { fprintf(stderr, "Failed to open device\n"); return 1; }
    sdr->Init();
    printf("Device opened.\n");

    // Configure RX at GPS L1
    lime::SDRConfig config;
    config.channel[0].rx.enabled = true;
    config.channel[0].rx.centerFrequency = 1575.42e6;
    config.channel[0].rx.sampleRate = 4096000.0;
    config.channel[0].rx.oversample = 0;
    config.channel[0].rx.lpf = 2.5e6;
    config.channel[0].rx.path = 3;
    config.channel[0].rx.gain[lime::eGainTypes::LNA] = 15;
    config.channel[0].rx.gain[lime::eGainTypes::PGA] = 10;
    sdr->Configure(config, 0);

    // Start RX stream (needed for sample_valid to fire)
    lime::StreamConfig scfg;
    scfg.channels.at(lime::TRXDir::Rx) = {0};
    scfg.format = lime::DataFormat::I16;
    scfg.linkFormat = lime::DataFormat::I12;
    rfstream = sdr->StreamCreate(scfg, 0);
    if (!rfstream) { fprintf(stderr, "StreamCreate failed\n"); return 1; }
    rfstream->Start();
    pthread_create(&drain_tid, NULL, rx_drain_thread, NULL);
    printf("RX streaming started.\n\n");

    // Let stream stabilize
    usleep(500000);

    // Stream tap debug
    printf("Stream tap debug:\n");
    printf("  lms_rx reset: %u\n", csr_read(CSR_DBG_RST));
    printf("  valid count:  %u\n", csr_read(CSR_DBG_VALID));
    printf("  ready count:  %u\n", csr_read(CSR_DBG_READY));
    printf("  both count:   %u\n\n", csr_read(CSR_DBG_BOTH));

    // VERSION
    uint32_t ver = gps_read(0x7A);
    printf("GPS VERSION: 0x%08x %s\n", ver,
           ver == 0x47505307 ? "(GPS v7 OK)" : "(UNEXPECTED)");

    // Epoch counter
    uint32_t ep1 = gps_read(0x79);
    usleep(200000);
    uint32_t ep2 = gps_read(0x79);
    printf("Epoch counter: %u -> %u (delta=%d in 200ms, expect ~200)\n\n",
           ep1, ep2, (int)(ep2 - ep1));

    // Enable global correlator
    printf("Enabling global correlator...\n");
    gps_write(0x78, 1);
    usleep(10000);

    // Read initial debug register
    uint32_t dbg0 = gps_read(0x7F);
    printf("Initial debug: 0x%08x  state=%s\n\n", dbg0,
           state_name((dbg0 >> 22) & 0xF));

    // Start acquisition: PRN=1, 20 epochs, carrier=0
    // ACQ_CFG: prn at bits [20:16], num_epochs at bits [5:0]
    printf("=== Starting acquisition: PRN=1, epochs=20 ===\n");
    gps_write(0x7C, (1u << 16) | 20u);  // ACQ_CFG: prn=1, epochs=20
    gps_write(0x7D, 0);                 // ACQ_CARRIER: 0 Hz offset
    gps_write(0x7B, 2);                 // abort any in-progress
    usleep(1000);
    gps_write(0x7B, 1);                 // start

    // Poll at 50ms intervals for up to 5 seconds
    uint32_t prev_state = 99;
    for (int i = 0; i < 100; i++) {
        usleep(50000);
        uint32_t status = gps_read(0x7B);
        uint32_t debug  = gps_read(0x7F);

        uint32_t st   = (debug >> 22) & 0xF;
        uint32_t fsyn = (debug >> 21) & 1;
        uint32_t ssn  = (debug >> 20) & 1;
        uint32_t oact = (debug >> 19) & 1;
        uint32_t ecnt = (debug >> 13) & 0x3F;
        uint32_t cnt  = debug & 0x1FFF;

        // Print on state change or every 10th iteration
        if (st != prev_state || (i % 10) == 0) {
            printf("  [%4dms] debug=0x%08x status=0x%08x  "
                   "state=%-10s fft_sync=%d sync_seen=%d out_active=%d "
                   "epoch=%d cnt=%d\n",
                   (i + 1) * 50, debug, status,
                   state_name(st), fsyn, ssn, oact, ecnt, cnt);
            prev_state = st;
        }

        if (status & 0x80000000u) {
            printf("\n*** Acquisition DONE! ***\n");
            uint32_t power = gps_read(0x7E);
            uint32_t phase = status & 0x3FF;
            printf("  best_phase=%u  best_power=%u\n", phase, power);
            goto cleanup;
        }
    }

    printf("\n*** Acquisition DID NOT COMPLETE after 5 seconds ***\n");
    printf("Final debug: 0x%08x  state=%s\n",
           gps_read(0x7F), state_name((gps_read(0x7F) >> 22) & 0xF));

cleanup:
    running = 0;
    pthread_join(drain_tid, NULL);
    rfstream->Stop();
    rfstream.reset();
    lime::DeviceRegistry::freeDevice(sdr);
    printf("\nDone.\n");
    return 0;
}
