#ifndef MBC3_H
#define MBC3_H

#include "mbc.h"

class MBC3 : public MBC {
public:
    MBC3(std::vector<uint8_t>& rom, std::vector<uint8_t>& ram);
    uint8_t read(uint16_t address) const override;
    void write(uint16_t address, uint8_t value) override;

private:
    uint8_t romBank_;
    uint8_t ramBank_; //0-3 for RAM, 4-7 for RTC registers
    bool ramEnabled_;
    bool latchClockData_;

    //RTC Registers (simplified as a single byte for now)
    uint8_t rtcRegister_;

    uint16_t getRomBankOffset() const;
    uint16_t getRamBankOffset() const;
};

#endif // MBC3_H