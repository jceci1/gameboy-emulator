#ifndef MBC1_H
#define MBC1_H

#include "mbc.h"

class MBC1 : public MBC {
public:
    MBC1(std::vector<uint8_t>& rom, std::vector<uint8_t>& ram);
    uint8_t read(uint16_t address) const override;
    void write(uint16_t address, uint8_t value) override;

private:
    uint8_t romBank_;
    uint8_t ramBank_;
    bool ramEnabled_;
    bool mode_; //0: ROM Banking Mode, 1: RAM Banking Mode

    uint16_t getRomBankOffset() const;
    uint16_t getRamBankOffset() const;
};

#endif // MBC1_H