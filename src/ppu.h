#ifndef PPU_H
#define PPU_H

#include <cstdint>
#include <array>
#include <vector>
#include <algorithm>
#include "memory.h"

class PPU {
public:
    PPU(Memory& memory);
    void step(int cycles);
    const std::array<std::array<uint32_t, 160>, 144>& getFrameBuffer() const;

private:
    Memory& memory;
    std::array<std::array<uint32_t, 160>, 144> frameBuffer;
    int scanline;
    int cycles;

    enum class Mode { OAMScan, Drawing, HBlank, VBlank };
    Mode mode;

    //PPU registers
    uint8_t& LCDC;
    uint8_t& STAT;
    uint8_t& SCY;
    uint8_t& SCX;
    uint8_t& LY;
    uint8_t& LYC;
    uint8_t& DMA;
    uint8_t& BGP;
    uint8_t& OBP0;
    uint8_t& OBP1;
    uint8_t& WY;
    uint8_t& WX;

    void setMode(uint8_t modeNum);
    void renderScanline();
    void renderBackground();
    void renderWindow();
    void renderSprites();
    uint32_t getColor(uint8_t colorNum, uint8_t palette);
};

#endif // PPU_H