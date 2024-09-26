//memory.cpp
#include "memory.h"
#include <algorithm>
#include <stdexcept>
#include <iostream>

//constructor for memory class
Memory::Memory() : bootROMEnabled(true), cartridge(nullptr) {
    mem.fill(0);
    io.fill(0xFF); //typically, i/o registers are initialized to 0xff
}

//loads the boot rom into memory
void Memory::loadBootROM(const std::vector<uint8_t>& bootRomData) {
    if (bootRomData.size() > BOOT_ROM_END + 1) {
        throw std::runtime_error("Boot ROM size exceeds allocated memory");
    }
    bootROM = bootRomData;
}

//loads the game rom into memory
void Memory::loadROM(const std::vector<uint8_t>& romData) {
    if (romData.size() > mem.size()) {
        throw std::runtime_error("ROM size exceeds memory capacity");
    }
    std::copy(romData.begin(), romData.end(), mem.begin());
}

//reads a byte from the specified address
uint8_t Memory::read(uint16_t address) const {
    if (bootROMEnabled && address <= BOOT_ROM_END) {
        return bootROM[address];
    }

    if (address >= VRAM_START && address <= VRAM_END) {
        return mem[address];
    }

    if (address >= IO_REGISTERS_START && address <= IO_REGISTERS_END) {
        //initialize some io registers if they haven't been set yet
        if (address == 0xFF40 && io[0x40] == 0xFF) return 0x91; //lcdc default value
        if (address == 0xFF41 && io[0x41] == 0xFF) return 0x85; //stat default value
        if (address == 0xFF42 && io[0x42] == 0xFF) return 0x00; //scy default value
        if (address == 0xFF43 && io[0x43] == 0xFF) return 0x00; //scx default value
        return io[address - IO_REGISTERS_START];
    }

    if (address >= 0xA000 && address <= 0xBFFF && cartridge) { //external ram via mbc
        return cartridge->read(address);
    }

    return mem[address];
}

//writes a byte to the specified address
void Memory::write(uint16_t address, uint8_t value) {
    if (bootROMEnabled && address <= BOOT_ROM_END) {
        //typically, writes to boot rom are ignored
        return;
    }

    if (address == 0xFF40) {  //lcdc register
        std::cout << "LCDC set to: 0x" << std::hex << static_cast<int>(value) << std::dec 
                  << " (LCD Enable: " << ((value & 0x80) ? "ON" : "OFF") 
                  << ", Window Tile Map: 0x" << ((value & 0x40) ? "9C00" : "9800")
                  << ", Window Enable: " << ((value & 0x20) ? "ON" : "OFF")
                  << ", BG & Window Tile Data: 0x" << ((value & 0x10) ? "8000" : "8800")
                  << ", BG Tile Map: 0x" << ((value & 0x08) ? "9C00" : "9800")
                  << ", Sprite Size: " << ((value & 0x04) ? "8x16" : "8x8")
                  << ", Sprite Enable: " << ((value & 0x02) ? "ON" : "OFF")
                  << ", BG Enable: " << ((value & 0x01) ? "ON" : "OFF")
                  << ")" << std::endl;
    } else if (address == 0xFF42) {  //scy register
        std::cout << "SCY set to: 0x" << std::hex << static_cast<int>(value) << std::dec << std::endl;
    } else if (address == 0xFF43) {  //scx register
        std::cout << "SCX set to: 0x" << std::hex << static_cast<int>(value) << std::dec << std::endl;
    }

    if (address >= IO_REGISTERS_START && address <= IO_REGISTERS_END) {
        io[address - IO_REGISTERS_START] = value;

        //handle special i/o register writes
        if (address == 0xFF50 && value == 0x01) { //disable boot rom
            bootROMEnabled = false;
        } else if (address == 0xFF46) { //dma transfer
            dmaTransfer(value);
        }

        return;
    }

    if (address >= 0xA000 && address <= 0xBFFF && cartridge) { //external ram via mbc
        cartridge->write(address, value);
        return;
    }

    //handle mbc-controlled rom areas
    if (address >= ROM_BANK_0_START && address <= ROM_BANK_N_END) {
        handleMemoryBankControllers(address, value);
        return;
    }

    if (address >= ECHO_RAM_START && address <= ECHO_RAM_END) {
        mem[address] = value;
        mem[address - 0x2000] = value;  //mirror write to wram
        return;
    }

    mem[address] = value;
}

//handles memory bank controller operations
void Memory::handleMemoryBankControllers(uint16_t address, uint8_t value) {
    if (cartridge) {
        cartridge->write(address, value);
    }
    //if no cartridge is loaded, do nothing
}

//requests an interrupt by setting the corresponding bit in the if register
void Memory::requestInterrupt(uint8_t interruptBit) {
    if (interruptBit >= 5) return; //only 5 interrupts are defined
    io[0x0F] |= (1 << interruptBit); //if register is at 0xff0f, index 0x0f - io_registers_start = 0
}

//performs a dma transfer
void Memory::dmaTransfer(uint8_t value) {
    uint16_t sourceAddress = value << 8; //dma transfer always starts at xx00
    for (int i = 0; i < 160; ++i) {
        write(0xFE00 + i, read(sourceAddress + i));
    }
}