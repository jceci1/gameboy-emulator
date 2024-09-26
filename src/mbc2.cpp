#include "mbc2.h"
#include <stdexcept>

//constructor for mbc2 class
MBC2::MBC2(std::vector<uint8_t>& rom, std::vector<uint8_t>& ram)
    : MBC(rom, ram), romBank_(1), ramEnabled_(false) {}

//reads a byte from the specified address
uint8_t MBC2::read(uint16_t address) const {
    if (address < 0x4000) { //bank 0
        return rom_[address];
    } else if (address < 0x8000) { //switchable rom bank
        uint16_t bankOffset = getRomBankOffset();
        return rom_[bankOffset + (address - 0x4000)];
    } else if (address >= 0xA000 && address < 0xA200) { //internal ram (nibble)
        if (ramEnabled_ && !ram_.empty()) {
            //mbc2 ram is 512 nibbles, hence only lower 4 bits are used
            return ram_[address - 0xA000] & 0x0F;
        }
    }
    return 0xFF; //unmapped memory
}

//writes a byte to the specified address
void MBC2::write(uint16_t address, uint8_t value) {
    if (address < 0x4000) { //rom bank number and ram enable
        if (!(address & 0x0100)) { //only lower half (0x0000-0x0fff)
            ramEnabled_ = ((value & 0x0F) == 0x0A);
        } else { //upper half (0x1000-0x1fff)
            uint8_t bank = value & 0x0F;
            if (bank == 0) bank = 1; //bank 0 is not allowed
            romBank_ = bank;
        }
    }
    else if (address >= 0xA000 && address < 0xA200) { //internal ram (nibble)
        if (ramEnabled_ && !ram_.empty()) {
            ram_[address - 0xA000] = value & 0x0F; //only lower nibble writable
        }
    }
}

//calculates the offset for the current rom bank
uint16_t MBC2::getRomBankOffset() const {
    return romBank_ * 0x4000;
}