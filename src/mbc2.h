#ifndef MBC2_H
#define MBC2_H

#include "mbc.h"

class MBC2 : public MBC {
public:
    MBC2(std::vector<uint8_t>& rom, std::vector<uint8_t>& ram);
    uint8_t read(uint16_t address) const override;
    void write(uint16_t address, uint8_t value) override;

private:
    uint8_t romBank_;
    bool ramEnabled_;

    uint16_t getRomBankOffset() const;
};

#endif // MBC2_H