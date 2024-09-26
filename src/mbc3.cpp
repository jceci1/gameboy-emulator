#include "mbc3.h"
#include <stdexcept>

//constructor for mbc3 class
MBC3::MBC3(std::vector<uint8_t>& rom, std::vector<uint8_t>& ram)
    : MBC(rom, ram), romBank_(1), ramBank_(0), ramEnabled_(false),
      latchClockData_(false), rtcRegister_(0) {}

//reads a byte from the specified address
uint8_t MBC3::read(uint16_t address) const {
    if (address < 0x4000) { //bank 0
        return rom_[address];
    }
    else if (address < 0x8000) { //switchable rom bank
        uint16_t bankOffset = getRomBankOffset();
        return rom_[bankOffset + (address - 0x4000)];
    }
    else if (address >= 0xA000 && address < 0xC000) { //external ram or rtc
        if (ramEnabled_ && !ram_.empty()) {
            if (ramBank_ <= 3) { //ram banks 0-3
                uint16_t bankOffset = getRamBankOffset();
                return ram_[bankOffset + (address - 0xA000)];
            }
            else { //rtc registers (simplified)
                //implement rtc register reading as needed
                //for simplicity, returning rtcregister_
                return rtcRegister_;
            }
        }
    }
    return 0xFF; //unmapped memory
}

//writes a byte to the specified address
void MBC3::write(uint16_t address, uint8_t value) {
    if (address < 0x2000) { //ram enable / rtc register select
        if (address < 0x2000) {
            ramEnabled_ = ((value & 0x0F) == 0x0A);
        }
    }
    else if (address < 0x4000) { //rom bank number / rtc register select
        if (address < 0x4000) {
            romBank_ = value & 0x7F;
            if (romBank_ == 0) romBank_ = 1;
        }
    }
    else if (address < 0x6000) { //ram bank number / rtc register select
        if (address < 0x6000) {
            ramBank_ = value & 0x0F; //0-3: ram banks, 4-7: rtc registers
        }
    }
    else if (address < 0x8000) { //latch clock data
        //typically, writing 0x00 then 0x01 latches the rtc data
        if (address < 0x8000) {
            if (value == 0x00) {
                latchClockData_ = true;
            }
            else if (value == 0x01 && latchClockData_) {
                //implement rtc latching if needed
                latchClockData_ = false;
            }
        }
    }
    else if (address >= 0xA000 && address < 0xC000) { //external ram or rtc
        if (ramEnabled_ && !ram_.empty()) {
            if (ramBank_ <= 3) { //ram banks 0-3
                uint16_t bankOffset = getRamBankOffset();
                ram_[bankOffset + (address - 0xA000)] = value;
            }
            else { //rtc registers (simplified)
                //implement rtc register writing as needed
                //for simplicity, updating rtcregister_
                rtcRegister_ = value;
            }
        }
    }
}

//calculates the offset for the current rom bank
uint16_t MBC3::getRomBankOffset() const {
    return romBank_ * 0x4000;
}

//calculates the offset for the current ram bank
uint16_t MBC3::getRamBankOffset() const {
    return ramBank_ * 0x2000;
}