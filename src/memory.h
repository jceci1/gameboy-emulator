// memory.h
#ifndef MEMORY_H
#define MEMORY_H

#include <array>
#include <cstdint>
#include <vector>
#include <memory>
#include "cartridge.h"

class Memory {
public:
    Memory();
    uint8_t read(uint16_t address) const;
    void write(uint16_t address, uint8_t value);
    void loadROM(const std::vector<uint8_t>& romData);
    void loadBootROM(const std::vector<uint8_t>& bootRomData);
    void requestInterrupt(uint8_t interruptBit);

    //I/O registers
    std::array<uint8_t, 0x80> io;

    //cartridge (MBC)
    std::unique_ptr<Cartridge> cartridge;


private:
    std::array<uint8_t, 0x10000> mem;
    bool bootROMEnabled;
    std::vector<uint8_t> bootROM;

    //memory regions
    static constexpr uint16_t BOOT_ROM_END = 0x00FF;
    static constexpr uint16_t ROM_BANK_0_START = 0x0000;
    static constexpr uint16_t ROM_BANK_0_END = 0x3FFF;
    static constexpr uint16_t ROM_BANK_N_START = 0x4000;
    static constexpr uint16_t ROM_BANK_N_END = 0x7FFF;
    static constexpr uint16_t VRAM_START = 0x8000;
    static constexpr uint16_t VRAM_END = 0x9FFF;
    static constexpr uint16_t EXT_RAM_START = 0xA000;
    static constexpr uint16_t EXT_RAM_END = 0xBFFF;
    static constexpr uint16_t WRAM_START = 0xC000;
    static constexpr uint16_t WRAM_END = 0xDFFF;
    static constexpr uint16_t ECHO_RAM_START = 0xE000;
    static constexpr uint16_t ECHO_RAM_END = 0xFDFF;
    static constexpr uint16_t OAM_START = 0xFE00;
    static constexpr uint16_t OAM_END = 0xFE9F;
    static constexpr uint16_t IO_REGISTERS_START = 0xFF00;
    static constexpr uint16_t IO_REGISTERS_END = 0xFF7F;
    static constexpr uint16_t HRAM_START = 0xFF80;
    static constexpr uint16_t HRAM_END = 0xFFFE;
    static constexpr uint16_t IE_REGISTER = 0xFFFF;

    void handleMemoryBankControllers(uint16_t address, uint8_t value);
    void dmaTransfer(uint8_t value);
};

#endif // MEMORY_H