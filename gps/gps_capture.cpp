#include "limesuiteng/limesuiteng.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <csignal>
#include <cstring>
#include <complex>

using namespace lime;

static bool stopProgram = false;
void intHandler(int) { stopProgram = true; }

static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl <= LogLevel::Warning)
        std::cerr << msg << std::endl;
}

int main(int argc, char** argv)
{
    double centerFreq = 1575.42e6;
    double sampleRate = 4.096e6;
    int durationSec = 30;
    const char* outputFile = "/tmp/gps_capture.bin";

    if (argc > 1) outputFile = argv[1];
    if (argc > 2) durationSec = std::atoi(argv[2]);

    lime::registerLogHandler(LogCallback);

    auto devices = DeviceRegistry::enumerate();
    if (devices.empty()) {
        std::cerr << "No devices found" << std::endl;
        return 1;
    }

    SDRDevice* device = DeviceRegistry::makeDevice(devices[0]);
    if (!device) {
        std::cerr << "Failed to open device" << std::endl;
        return 1;
    }

    std::cout << "Connected to: " << device->GetDescriptor().name << std::endl;
    device->SetMessageLogCallback(LogCallback);
    device->Init();

    const auto& chipDesc = device->GetDescriptor().rfSOC[0];
    int lnawIndex = -1;
    const auto& rxPaths = chipDesc.pathNames.at(TRXDir::Rx);
    for (size_t i = 0; i < rxPaths.size(); i++) {
        if (rxPaths[i] == "LNAW") { lnawIndex = i; break; }
    }
    if (lnawIndex < 0) {
        std::cerr << "LNAW antenna not found" << std::endl;
        DeviceRegistry::freeDevice(device);
        return 1;
    }
    std::cout << "Using antenna: LNAW (index " << lnawIndex << ")" << std::endl;

    SDRConfig config;
    config.channel[0].rx.enabled = true;
    config.channel[0].rx.centerFrequency = centerFreq;
    config.channel[0].rx.sampleRate = sampleRate;
    config.channel[0].rx.oversample = 2;
    config.channel[0].rx.lpf = 5e6;
    config.channel[0].rx.path = lnawIndex;
    config.channel[0].rx.calibrate = CalibrationFlag::NONE;
    config.channel[0].rx.testSignal.enabled = false;

    config.channel[0].tx.enabled = false;
    config.channel[0].tx.sampleRate = sampleRate;
    config.channel[0].tx.oversample = 2;

    std::cout << "Configuring: " << centerFreq/1e6 << " MHz, "
              << sampleRate/1e6 << " MS/s, " << durationSec << "s" << std::endl;

    device->Configure(config, 0);

    // Set gains: LNA=30dB, TIA=12dB, PGA=19dB
    device->SetGain(0, TRXDir::Rx, 0, eGainTypes::LNA, 30);
    device->SetGain(0, TRXDir::Rx, 0, eGainTypes::TIA, 12);
    device->SetGain(0, TRXDir::Rx, 0, eGainTypes::PGA, 19);
    std::cout << "Gains set: LNA=30, TIA=12, PGA=19" << std::endl;

    StreamConfig streamCfg;
    streamCfg.channels[TRXDir::Rx] = { 0 };
    streamCfg.format = DataFormat::F32;
    streamCfg.linkFormat = DataFormat::I16;

    std::unique_ptr<RFStream> stream = device->StreamCreate(streamCfg, 0);
    stream->Start();

    signal(SIGINT, intHandler);

    std::ofstream outFile(outputFile, std::ios::binary);
    if (!outFile) {
        std::cerr << "Cannot open output file: " << outputFile << std::endl;
        stream.reset();
        DeviceRegistry::freeDevice(device);
        return 1;
    }

    const int bufSize = 16384;
    complex32f_t** rxBuf = new complex32f_t*[1];
    rxBuf[0] = new complex32f_t[bufSize];

    uint64_t totalSamples = 0;
    uint64_t targetSamples = static_cast<uint64_t>(sampleRate * durationSec);

    auto startTime = std::chrono::high_resolution_clock::now();
    auto lastReport = startTime;

    StreamMeta meta{};
    while (totalSamples < targetSamples && !stopProgram) {
        uint32_t got = stream->StreamRx(rxBuf, bufSize, &meta);
        if (got == 0) continue;

        outFile.write(reinterpret_cast<const char*>(rxBuf[0]), got * sizeof(complex32f_t));
        totalSamples += got;

        auto now = std::chrono::high_resolution_clock::now();
        if (now - lastReport > std::chrono::seconds(1)) {
            lastReport = now;
            double elapsed = std::chrono::duration<double>(now - startTime).count();
            double mbWritten = totalSamples * sizeof(complex32f_t) / 1e6;
            std::cout << "  " << elapsed << "s: " << totalSamples << " samples, "
                      << mbWritten << " MB" << std::endl;
        }
    }

    outFile.close();
    stream.reset();

    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(endTime - startTime).count();
    double mbWritten = totalSamples * sizeof(complex32f_t) / 1e6;

    std::cout << "Done: " << totalSamples << " samples in " << elapsed
              << "s (" << mbWritten << " MB)" << std::endl;

    DeviceRegistry::freeDevice(device);
    delete[] rxBuf[0];
    delete[] rxBuf;
    return 0;
}
