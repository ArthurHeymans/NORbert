#include "Vwrapper.h"
#include "verilated.h"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Byte-level stdin protocol; stdout contains exactly one hex response per line.
// No command recognition or protection logic lives here: those belong to RTL.
class Bridge {
    Vwrapper dut;
    std::vector<uint8_t> memory = std::vector<uint8_t>(64 * 1024 * 1024, 0xff);
    std::vector<uint8_t> host_output;
    bool prev_read = false, prev_post = false, prev_pair = false, fill_b = false;
    unsigned busy_ticks = 0;

    uint64_t load(size_t addr) const {
        uint64_t value = 0;
        for (unsigned i = 0; i < 8; ++i) value |= uint64_t(memory.at(addr + i)) << (8 * i);
        return value;
    }
    void fill(uint32_t burst) {
        const uint64_t data = load(size_t(burst) * 8);
        if (fill_b) { dut.ram_b = data; dut.valid_b = 1; dut.beats_b = 0xf; }
        else { dut.ram_a = data; dut.valid_a = 1; dut.beats_a = 0xf; }
        fill_b = !fill_b;
    }
    void prefetch() {
        if (dut.cs) {
            prev_read = dut.ram_read;
            prev_post = dut.ram_post_toggle;
            prev_pair = dut.ram_pair;
            fill_b = false;
            dut.valid_a = dut.valid_b = 0;
            dut.beats_a = dut.beats_b = 0;
        } else {
            // A post-toggle edge may accompany a read rising edge: one fill only.
            if (dut.ram_read && (!prev_read || dut.ram_post_toggle != prev_post))
                fill(dut.ram_addr);
            // The controller fetches the pair burst right behind the first.
            if (dut.ram_pair && !prev_pair) fill(dut.ram_addr2);
            prev_read = dut.ram_read;
            prev_post = dut.ram_post_toggle;
            prev_pair = dut.ram_pair;
        }
        dut.eval();
    }
public:
    Bridge() {
        dut.cs = dut.wp_n = dut.hold_n = 1;
        dut.reset = 1;
        ticks(16);
        dut.reset = 0;
        ticks(512); // Page-buffer initialization must finish before START.
        host_output.clear();
    }
    void ticks(unsigned count) {
        for (unsigned i = 0; i < count; ++i) {
            dut.clk = 0;
            dut.access_accept = 0;
            dut.cmd_busy = busy_ticks != 0;
            dut.read_busy = busy_ticks != 0;
            dut.eval();
            if (!busy_ticks && dut.access_cmd) {
                const size_t addr = size_t(dut.access_addr >> 2) * 8;
                if (dut.access_cmd == 1) dut.read_data = load(addr);
                if (dut.access_cmd == 2)
                    for (unsigned b = 0; b < 8; ++b) memory.at(addr + b) = dut.write_data >> (8 * b);
                dut.access_accept = 1;
                // Deliberate read latency lets spi_program capture all 8 page bytes.
                busy_ticks = 12;
            } else if (busy_ticks) --busy_ticks;
            dut.eval();
            dut.clk = 1;
            dut.eval();
            if (dut.host_tx_strobe) host_output.push_back(dut.host_tx_data);
            prefetch();
        }
    }
    std::vector<uint8_t> host(const std::vector<uint8_t>& data) {
        host_output.clear();
        for (uint8_t b : data) {
            dut.host_data = b; dut.host_strobe = 1; ticks(1);
            dut.host_strobe = 0; ticks(15);
        }
        ticks(128);
        if (!data.empty() && (data[0] == 0x35 || data[0] == 0x34)) {
            // Control ACKs can wait for accepted array work or page scrub.
            unsigned waited = 0;
            while (host_output.empty() && waited++ < 2000000) ticks(1);
            if (host_output.empty()) throw std::runtime_error("control ACK timeout");
        }
        return host_output;
    }
    std::vector<uint8_t> spi(const std::vector<uint8_t>& data, bool finish = true) {
        if (!dut.running) throw std::runtime_error("SPI transfer while STOPPED");
        dut.sck = 0; dut.cs = 0; dut.eval(); ticks(16);
        std::vector<uint8_t> output;
        for (uint8_t b : data) {
            uint8_t r = 0;
            for (int bit = 7; bit >= 0; --bit) {
                dut.mosi = (b >> bit) & 1; ticks(12);
                dut.sck = 1; dut.eval(); prefetch(); ticks(12);
                r = uint8_t((r << 1) | (dut.miso_oe ? dut.miso : 1));
                dut.sck = 0; dut.eval(); prefetch();
            }
            output.push_back(r);
        }
        if (finish) release();
        return output;
    }
    void release() {
        dut.cs = 1; dut.eval(); ticks(32);
        if (dut.miso_oe) throw std::runtime_error("MISO still driven after CS high");
    }
    void delay(uint64_t us) {
        // Simulation time is not analog flash time. Always provide at least 512
        // controller ticks; cap huge flash delays, relying on real WIP polling.
        ticks(unsigned(std::min<uint64_t>(std::max<uint64_t>(us * 120, 512), 200000)));
    }
    void seed(size_t address, const std::vector<uint8_t>& bytes) {
        if (address > memory.size() || bytes.size() > memory.size() - address)
            throw std::runtime_error("seed outside 64MiB memory");
        std::copy(bytes.begin(), bytes.end(), memory.begin() + address);
    }
    std::vector<uint8_t> inspect(size_t address, size_t length) const {
        if (address > memory.size() || length > memory.size() - address)
            throw std::runtime_error("inspection outside 64MiB memory");
        return {memory.begin() + address, memory.begin() + address + length};
    }
};
static std::vector<uint8_t> unhex(const std::string& text) {
    if (text.size() % 2) throw std::runtime_error("odd hex length");
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < text.size(); i += 2) {
        const auto pair = text.substr(i, 2);
        if (pair.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos)
            throw std::runtime_error("invalid hex");
        bytes.push_back(std::stoul(pair, nullptr, 16));
    }
    return bytes;
}
static void printhex(const std::vector<uint8_t>& bytes) {
    static const char* digits = "0123456789abcdef";
    for (uint8_t b : bytes) std::cout << digits[b >> 4] << digits[b & 15];
    std::cout << '\n' << std::flush;
}
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    try {
        Bridge bridge;
        std::string line;
        while (std::getline(std::cin, line)) {
            std::istringstream in(line);
            char command; std::string hex; uint64_t value, length;
            in >> command;
            switch (command) {
            case 'H': in >> hex; printhex(bridge.host(unhex(hex))); break;
            case 'S': in >> hex; printhex(bridge.spi(unhex(hex))); break;
            case 'B': in >> hex; printhex(bridge.spi(unhex(hex), false)); break;
            case 'C': bridge.release(); printhex({}); break;
            case 'D': if (!(in >> value)) throw std::runtime_error("missing delay"); bridge.delay(value); printhex({}); break;
            case 'M': if (!(in >> value >> hex)) throw std::runtime_error("missing seed"); bridge.seed(value, unhex(hex)); printhex({}); break;
            case 'R': if (!(in >> value >> length)) throw std::runtime_error("missing range"); printhex(bridge.inspect(value, length)); break;
            default: throw std::runtime_error("unknown bridge command");
            }
        }
    } catch (const std::exception& e) { std::cerr << "bridge: " << e.what() << '\n'; return 1; }
}
