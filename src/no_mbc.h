#ifndef NO_MBC_H
#define NO_MBC_H

#include "mbc.h"

class NoMBC : public MBC {
public:
    NoMBC(std::vector<uint8_t>& rom, std::vector<uint8_t>& ram);
    uint8_t read(uint16_t address) const override;
    void write(uint16_t address, uint8_t value) override;
};

#endif // NO_MBC_H