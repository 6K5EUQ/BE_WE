#include <libbladeRF.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <unistd.h>

#define RX_GAIN       60
#define DURATION_SEC  3
#define CHANNEL       BLADERF_CHANNEL_RX(0)

/* ---------- WAV 헤더 ---------- */
struct WavHeader {
    char riff[4] = {'R','I','F','F'};
    uint32_t file_size;
    char wave[4] = {'W','A','V','E'};
    char fmt[4]  = {'f','m','t',' '};
    uint32_t fmt_size = 16;
    uint16_t audio_format = 1;
    uint16_t num_channels = 2;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample = 16;
    char data[4] = {'d','a','t','a'};
    uint32_t data_size;
};

int main() {
    double freq_mhz;
    double rate_msps;

    std::cout << "Enter center frequency (MHz): ";
    std::cin >> freq_mhz;
    std::cout << "Enter sample rate (MSPS): ";
    std::cin >> rate_msps;

    uint64_t center_freq = static_cast<uint64_t>(freq_mhz * 1e6);
    uint32_t sample_rate = static_cast<uint32_t>(rate_msps * 1e6);
    uint32_t bandwidth   = sample_rate / 2;

    /* ---------- 파일명 생성 ---------- */
    std::time_t t = std::time(nullptr);
    std::tm *tm = std::localtime(&t);

    uint64_t mhz = center_freq / 1000000;
    uint64_t khz = (center_freq / 1000) % 1000;
    uint64_t hz  = center_freq % 1000;

    std::ostringstream fname;
    fname << std::put_time(tm, "%b_%d_%Y_")
        << mhz << "-"
        << std::setw(3) << std::setfill('0') << khz << "-"
        << std::setw(3) << std::setfill('0') << hz
        << "MHz";

    std::string base = fname.str();

    /* ---------- bladeRF ---------- */
    struct bladerf *dev = nullptr;
    if (bladerf_open(&dev, nullptr) != 0) {
        fprintf(stderr, "Failed to open bladeRF\n");
        return 1;
    }

    bladerf_set_frequency(dev, CHANNEL, center_freq);
    bladerf_set_sample_rate(dev, CHANNEL, sample_rate, nullptr);
    bladerf_set_bandwidth(dev, CHANNEL, bandwidth, nullptr);
    bladerf_set_gain_mode(dev, CHANNEL, BLADERF_GAIN_MANUAL);
    bladerf_set_gain(dev, CHANNEL, RX_GAIN);

    bladerf_sync_config(
        dev,
        BLADERF_RX_X1,
        BLADERF_FORMAT_SC16_Q11,
        32,
        16384,
        16,
        5000
    );

    bladerf_enable_module(dev, CHANNEL, true);
    usleep(200000);

    size_t total_samples = sample_rate * DURATION_SEC;
    std::vector<int16_t> iq(total_samples * 2);

    size_t received = 0;
    while (received < total_samples) {
        size_t to_read = std::min<size_t>(16384, total_samples - received);
        int status = bladerf_sync_rx(
            dev,
            iq.data() + received * 2,
            to_read,
            nullptr,
            5000
        );
        if (status != 0) {
            fprintf(stderr, "RX error: %s\n", bladerf_strerror(status));
            break;
        }
        received += to_read;
    }

    bladerf_enable_module(dev, CHANNEL, false);
    bladerf_close(dev);

    /* ---------- WAV 저장 ---------- */
    WavHeader wav;
    wav.sample_rate = sample_rate;
    wav.byte_rate = sample_rate * 2 * 2;
    wav.block_align = 4;
    wav.data_size = received * 2 * sizeof(int16_t);
    wav.file_size = wav.data_size + sizeof(WavHeader) - 8;

    std::ofstream wav_file(base + ".wav", std::ios::binary);
    wav_file.write(reinterpret_cast<char*>(&wav), sizeof(wav));
    wav_file.write(reinterpret_cast<char*>(iq.data()), wav.data_size);
    wav_file.close();

    /* ---------- SigMF ---------- */
    std::ofstream data_file(base + ".sigmf-data", std::ios::binary);
    data_file.write(reinterpret_cast<char*>(iq.data()), wav.data_size);
    data_file.close();

    std::ofstream meta_file(base + ".sigmf-meta");
    meta_file <<
R"({
  "global": {
    "core:datatype": "ci16_le",
    "core:sample_rate": )" << sample_rate << R"(,
    "core:version": "1.0.0"
  },
  "captures": [
    {
      "core:frequency": )" << center_freq << R"(,
      "core:datetime": ")" << std::put_time(tm, "%FT%T") << R"("
    }
  ]
}
)";
    meta_file.close();

    printf("RX OK: %.1f MHz @ %.1f MSPS (%zu samples)\n",
           freq_mhz, rate_msps, received);

    return 0;
}
