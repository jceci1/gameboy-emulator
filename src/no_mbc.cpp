#include "no_mbc.h"

//constructor for nombc class
NoMBC::NoMBC(std::vector<uint8_t>& rom, std::vector<uint8_t>& ram)
    : MBC(rom, ram) {}

//reads a byte from the specified address
uint8_t NoMBC::read(uint16_t address) const {
    if (address < 0x8000) { //rom area
        return rom_[address];
    }
    else if (address >= 0xA000 && address < 0xC000) { //external ram
        if (!ram_.empty()) {
            return ram_[address - 0xA000];
        }
    }
    return 0xFF; //unmapped memory
}

//writes a byte to the specified address
void NoMBC::write(uint16_t address, uint8_t value) {
    if (address >= 0xA000 && address < 0xC000 && !ram_.empty()) { //external ram
        ram_[address - 0xA000] = value;
    }
    //writes to rom are ignored
}