// cartridge.h
#ifndef CARTRIDGE_H
#define CARTRIDGE_H

#include <vector>
#include <string>
#include <memory>
#include "mbc.h"

class Cartridge {
public:
    Cartridge(const std::string& filename);
    ~Cartridge() = default;

    uint8_t read(uint16_t address) const;
    void write(uint16_t address, uint8_t value);

    bool needsBattery() const { return battery_; }
    const std::string& getTitle() const { return title_; }
    uint8_t getCartridgeType() const { return cartridgeType_; }
    const std::vector<uint8_t>& getROMData() const { return rom_; }

private:
    std::vector<uint8_t> rom_;
    std::vector<uint8_t> ram_;
    std::unique_ptr<MBC> mbc_;

    std::string title_;
    uint8_t cartridgeType_;
    bool battery_;

    void loadROM(const std::string& filename);
    void parseCartridgeHeader();
    void setupMBC();
};

#endif // CARTRIDGE_H