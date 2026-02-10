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
#include <chrono>

#define RX_GAIN       60
#define CHANNEL       BLADERF_CHANNEL_RX(0)

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
    int duration_sec;

    std::cout << "Enter center frequency (MHz): ";
    std::cin >> freq_mhz;
    std::cout << "Enter sample rate (MSPS): ";
    std::cin >> rate_msps;
    std::cout << "Enter duration (seconds): ";
    std::cin >> duration_sec;

    uint64_t center_freq = static_cast<uint64_t>(freq_mhz * 1e6);
    uint32_t sample_rate = static_cast<uint32_t>(rate_msps * 1e6);

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

    struct bladerf *dev = nullptr;
    int status;
    
    status = bladerf_open(&dev, nullptr);
    if (status != 0) {
        fprintf(stderr, "Failed to open bladeRF: %s\n", bladerf_strerror(status));
        return 1;
    }

    status = bladerf_set_frequency(dev, CHANNEL, center_freq);
    if (status != 0) {
        fprintf(stderr, "Failed to set frequency: %s\n", bladerf_strerror(status));
        bladerf_close(dev);
        return 1;
    }

    uint32_t actual_rate;
    status = bladerf_set_sample_rate(dev, CHANNEL, sample_rate, &actual_rate);
    if (status != 0) {
        fprintf(stderr, "Failed to set sample rate: %s\n", bladerf_strerror(status));
        bladerf_close(dev);
        return 1;
    }

    uint32_t actual_bw;
    status = bladerf_set_bandwidth(dev, CHANNEL, sample_rate, &actual_bw);
    if (status != 0) {
        fprintf(stderr, "Failed to set bandwidth: %s\n", bladerf_strerror(status));
        bladerf_close(dev);
        return 1;
    }

    status = bladerf_set_gain_mode(dev, CHANNEL, BLADERF_GAIN_MANUAL);
    if (status != 0) {
        fprintf(stderr, "Failed to set gain mode: %s\n", bladerf_strerror(status));
        bladerf_close(dev);
        return 1;
    }

    status = bladerf_set_gain(dev, CHANNEL, RX_GAIN);
    if (status != 0) {
        fprintf(stderr, "Failed to set gain: %s\n", bladerf_strerror(status));
        bladerf_close(dev);
        return 1;
    }

    status = bladerf_sync_config(
        dev,
        BLADERF_RX_X1,
        BLADERF_FORMAT_SC16_Q11,
        512,
        16384,
        128,
        3000
    );
    if (status != 0) {
        fprintf(stderr, "Failed to configure sync: %s\n", bladerf_strerror(status));
        bladerf_close(dev);
        return 1;
    }

    status = bladerf_enable_module(dev, CHANNEL, true);
    if (status != 0) {
        fprintf(stderr, "Failed to enable RX: %s\n", bladerf_strerror(status));
        bladerf_close(dev);
        return 1;
    }
    
    usleep(200000);

    // 녹음 시작 알림
    printf("\n");
    printf("START RECORDING\n\n");

    size_t total_samples = actual_rate * duration_sec;
    std::vector<int16_t> iq(total_samples * 2);

    size_t received = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    while (received < total_samples) {
        size_t to_read = std::min<size_t>(16384, total_samples - received);
        status = bladerf_sync_rx(
            dev,
            iq.data() + received * 2,
            to_read,
            nullptr,
            3000
        );
        if (status != 0) {
            fprintf(stderr, "\nRX error: %s\n", bladerf_strerror(status));
            break;
        }
        received += to_read;
        
        // 0.1초 단위로 진행 상황 표시
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        
        // 0.1초마다 업데이트 (또는 완료 시)
        static double last_update = 0.0;
        if (elapsed - last_update >= 0.1 || received >= total_samples) {
            last_update = elapsed;
            double progress = 100.0 * received / total_samples;
            printf("\r[%5.1fs / %ds] Progress: %.1f%% ", 
                   elapsed, duration_sec, progress);
            fflush(stdout);
        }
    }
    
    printf("\n\n");

    bladerf_enable_module(dev, CHANNEL, false);
    bladerf_close(dev);

    // 파일 저장
    WavHeader wav;
    wav.sample_rate = actual_rate;
    wav.byte_rate = actual_rate * 2 * 2;
    wav.block_align = 4;
    wav.data_size = received * 2 * sizeof(int16_t);
    wav.file_size = wav.data_size + sizeof(WavHeader) - 8;

    std::ofstream wav_file(base + ".wav", std::ios::binary);
    wav_file.write(reinterpret_cast<char*>(&wav), sizeof(wav));
    wav_file.write(reinterpret_cast<char*>(iq.data()), wav.data_size);
    wav_file.close();

    std::ofstream data_file(base + ".sigmf-data", std::ios::binary);
    data_file.write(reinterpret_cast<char*>(iq.data()), wav.data_size);
    data_file.close();

    std::ofstream meta_file(base + ".sigmf-meta");
    meta_file << "{\n";
    meta_file << "  \"global\": {\n";
    meta_file << "    \"core:datatype\": \"ci16_le\",\n";
    meta_file << "    \"core:sample_rate\": " << actual_rate << ",\n";
    meta_file << "    \"core:version\": \"1.0.0\"\n";
    meta_file << "  },\n";
    meta_file << "  \"captures\": [\n";
    meta_file << "    {\n";
    meta_file << "      \"core:frequency\": " << center_freq << ",\n";
    meta_file << "      \"core:datetime\": \"" << std::put_time(tm, "%FT%T") << "\"\n";
    meta_file << "    }\n";
    meta_file << "  ]\n";
    meta_file << "}\n";
    meta_file.close();

    // 파일 크기 계산 (MB)
    double file_size_mb = wav.data_size / 1024.0 / 1024.0;

    // 성공 메시지
    printf("RECORDING SUCCESS\n");
    printf("\n");
    printf("Frequency:    %.1f MHz\n", freq_mhz);
    printf("Sample Rate:  %.2f MSPS\n", actual_rate / 1e6);
    printf("Bandwidth:    %.2f MHz\n", actual_bw / 1e6);
    printf("File Size:    %.2f MB\n", file_size_mb);
    printf("Samples:      %zu\n", received);
    printf("\n");
    printf("Files saved:\n");
    printf("  - %s.wav\n", base.c_str());
    printf("  - %s.sigmf-data\n", base.c_str());
    printf("  - %s.sigmf-meta\n", base.c_str());
    printf("\n");

    return 0;
}