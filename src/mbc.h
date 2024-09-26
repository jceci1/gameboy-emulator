#ifndef MBC_H
#define MBC_H

#include <cstdint>
#include <vector>

class MBC {
public:
    MBC(std::vector<uint8_t>& rom, std::vector<uint8_t>& ram) : rom_(rom), ram_(ram) {}
    virtual ~MBC() = default;
    virtual uint8_t read(uint16_t address) const = 0;
    virtual void write(uint16_t address, uint8_t value) = 0;

protected:
    std::vector<uint8_t>& rom_;
    std::vector<uint8_t>& ram_;
};

#endif // MBC_H