//mbc1.cpp
#include "mbc1.h"
#include <stdexcept>

//constructor for mbc1 class
MBC1::MBC1(std::vector<uint8_t>& rom, std::vector<uint8_t>& ram)
    : MBC(rom, ram), romBank_(1), ramBank_(0), ramEnabled_(false), mode_(false) {}

//reads a byte from the specified address
uint8_t MBC1::read(uint16_t address) const {
    if (address < 0x4000) { //bank 0
        return rom_[address];
    } else if (address < 0x8000) { //switchable rom bank
        uint16_t bankOffset = getRomBankOffset();
        return rom_[bankOffset + (address - 0x4000)];
    } else if (address >= 0xA000 && address < 0xC000) { //external ram
        if (ramEnabled_ && !ram_.empty()) {
            uint16_t bankOffset = getRamBankOffset();
            return ram_[bankOffset + (address - 0xA000)];
        }
    }
    return 0xFF; //unmapped memory
}

//writes a byte to the specified address
void MBC1::write(uint16_t address, uint8_t value) {
    if (address < 0x2000) { //ram enable
        ramEnabled_ = ((value & 0x0F) == 0x0A);
    }
    else if (address < 0x4000) { //rom bank number (lower 5 bits)
        uint8_t bank = value & 0x1F;
        if (bank == 0) bank = 1; //bank 0 is not allowed
        romBank_ = bank;
    }
    else if (address < 0x6000) { //ram bank number or upper rom bank number
        if (mode_) { //ram banking mode
            ramBank_ = value & 0x03;
        } else { //rom banking mode
            romBank_ |= (value & 0x03) << 5;
            if (romBank_ == 0) romBank_ = 1; //bank 0 is not allowed
        }
    }
    else if (address < 0x8000) { //banking mode select
        mode_ = (value & 0x01);
    }
    else if (address >= 0xA000 && address < 0xC000) { //external ram
        if (ramEnabled_ && !ram_.empty()) {
            uint16_t bankOffset = getRamBankOffset();
            ram_[bankOffset + (address - 0xA000)] = value;
        }
    }
}

//calculates the offset for the current rom bank
uint16_t MBC1::getRomBankOffset() const {
    return romBank_ * 0x4000;
}

//calculates the offset for the current ram bank
uint16_t MBC1::getRamBankOffset() const {
    return ramBank_ * 0x2000;
}