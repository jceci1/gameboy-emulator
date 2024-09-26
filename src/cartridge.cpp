// cartridge.cpp
#include "cartridge.h"
#include "mbc1.h"
#include "mbc2.h"
#include "mbc3.h"
#include "no_mbc.h"
#include <fstream>
#include <stdexcept>
#include <algorithm>

Cartridge::Cartridge(const std::string& filename) {
    loadROM(filename);
    parseCartridgeHeader();
    setupMBC();
}

//loads the rom data from a file
void Cartridge::loadROM(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("Unable to open ROM file: " + filename);
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    rom_.resize(size);
    if (!file.read(reinterpret_cast<char*>(rom_.data()), size)) {
        throw std::runtime_error("Unable to read ROM file: " + filename);
    }
}

//parses the acrtridge header to extract important info
void Cartridge::parseCartridgeHeader() {
    //extract titles (0x134 - 0x143)
    title_ = std::string(reinterpret_cast<char*>(&rom_[0x0134]), 16);
    title_.erase(std::find(title_.begin(), title_.end(), '\0'), title_.end());

    //cartridge type (0x147)
    cartridgeType_ = rom_[0x0147];

    //determines if battery is needed
    battery_ = (cartridgeType_ == 0x03 || cartridgeType_ == 0x06 || cartridgeType_ == 0x09 ||
                cartridgeType_ == 0x0D || cartridgeType_ == 0x0F || cartridgeType_ == 0x10 ||
                cartridgeType_ == 0x13 || cartridgeType_ == 0x1B || cartridgeType_ == 0x1E);

    //RAM size (0x149)
    uint8_t ramSize = rom_[0x0149];
    switch (ramSize) {
        case 0: ram_.resize(0); break;
        case 1: ram_.resize(2 * 1024); break;
        case 2: ram_.resize(8 * 1024); break;
        case 3: ram_.resize(32 * 1024); break;
        case 4: ram_.resize(128 * 1024); break;
        case 5: ram_.resize(64 * 1024); break;
        default: throw std::runtime_error("Invalid RAM size: " + std::to_string(ramSize));
    }
}

//sets up the correct memory bacnk controller based on cartridge type
void Cartridge::setupMBC() {
    switch (cartridgeType_) {
        case 0x00:
            mbc_ = std::make_unique<NoMBC>(rom_, ram_);
            break;
        case 0x01:
        case 0x02:
        case 0x03:
            mbc_ = std::make_unique<MBC1>(rom_, ram_);
            break;
        case 0x05:
        case 0x06:
            mbc_ = std::make_unique<MBC2>(rom_, ram_);
            break;
        case 0x0F:
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13:
            mbc_ = std::make_unique<MBC3>(rom_, ram_);
            break;
        default:
            throw std::runtime_error("Unsupported cartridge type: " + std::to_string(cartridgeType_));
    }
}

//reads byte from the cartridge at the specified address
uint8_t Cartridge::read(uint16_t address) const {
    if (mbc_) {
        return mbc_->read(address);
    }
    return 0xFF; //fallback for undefined behavior
}

//writes a byte to the acrtridge at the specified address
void Cartridge::write(uint16_t address, uint8_t value) {
    if (mbc_) {
        mbc_->write(address, value);
    }
}