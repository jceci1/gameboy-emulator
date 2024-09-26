#include "ppu.h"
#include <iostream>

//constructor for ppu class
PPU::PPU(Memory& mem) : 
    memory(mem),
    scanline(0),
    cycles(0),
    mode(Mode::OAMScan),
    LCDC(memory.io[0x40]),
    STAT(memory.io[0x41]),
    SCY(memory.io[0x42]),
    SCX(memory.io[0x43]),
    LY(memory.io[0x44]),
    LYC(memory.io[0x45]),
    DMA(memory.io[0x46]),
    BGP(memory.io[0x47]),
    OBP0(memory.io[0x48]),
    OBP1(memory.io[0x49]),
    WY(memory.io[0x4A]),
    WX(memory.io[0x4B])
{
    frameBuffer.fill({});
    SCX = 0;
    SCY = 0;
    LCDC = 0x91; //default value
}

//performs a step in the ppu emulation
void PPU::step(int cpuCycles) {
    cycles += cpuCycles;

    if (cycles >= 456) {
        cycles -= 456;
        scanline++;
        if (scanline > 153) {
            scanline = 0;
        }
        LY = scanline;

        if (scanline < 144) {
            renderScanline();
        } else if (scanline == 144) {
            memory.requestInterrupt(0); //v-blank interrupt
        }
    }
}

//sets the ppu mode and checks for stat interrupts
void PPU::setMode(uint8_t modeNum) {
    STAT = (STAT & 0xFC) | modeNum;
    
    //check for stat interrupts
    if ((modeNum == 0 && (STAT & 0x08)) || //h-blank
        (modeNum == 2 && (STAT & 0x20)) || //oam
        (modeNum == 1 && (STAT & 0x10)))   //v-blank
    {
        memory.requestInterrupt(1); //lcd stat interrupt
    }
}

//renders a single scanline
void PPU::renderScanline() {
    if (LCDC & 0x01) renderBackground();
    if (LCDC & 0x20) renderWindow();
    if (LCDC & 0x02) renderSprites();
}

//renders the background layer
void PPU::renderBackground() {
    if (!(LCDC & 0x01)) return; //background disabled

    uint16_t tileMap = (LCDC & 0x08) ? 0x9C00 : 0x9800;
    uint16_t tileData = (LCDC & 0x10) ? 0x8000 : 0x8800;
    bool unsignedIndex = (LCDC & 0x10) != 0;
    
    for (int x = 0; x < 160; x++) {
        int scrolledX = (x + SCX) & 255;
        int scrolledY = (scanline + SCY) & 255;
        int tileCol = scrolledX / 8;
        int tileRow = scrolledY / 8;

        uint16_t tileAddress = tileMap + tileRow * 32 + tileCol;
        uint8_t tileNum = memory.read(tileAddress);

        uint16_t tileDataAddress;
        if (unsignedIndex) {
            tileDataAddress = tileData + tileNum * 16;
        } else {
            tileDataAddress = tileData + ((int8_t)tileNum + 128) * 16;
        }

        uint8_t tileY = scrolledY % 8;
        uint16_t tileLineAddress = tileDataAddress + tileY * 2;

        uint8_t tileLowByte = memory.read(tileLineAddress);
        uint8_t tileHighByte = memory.read(tileLineAddress + 1);

        int colorBit = 7 - (scrolledX % 8);
        int colorNum = ((tileHighByte >> colorBit) & 1) << 1 | ((tileLowByte >> colorBit) & 1);
        
        frameBuffer[scanline][x] = getColor(colorNum, BGP);
    }
}

//renders the window layer
void PPU::renderWindow() {
    if (!(LCDC & 0x20) || WY > scanline) return; //window disabled or not visible on this scanline

    uint16_t tileMap = (LCDC & 0x40) ? 0x9C00 : 0x9800;
    uint16_t tileData = (LCDC & 0x10) ? 0x8000 : 0x8800;
    bool unsignedIndex = (LCDC & 0x10) != 0;

    int windowY = scanline - WY;

    for (int x = 0; x < 160; x++) {
        int windowX = x - WX + 7;
        if (windowX < 0) continue;

        int tileCol = windowX / 8;
        int tileRow = windowY / 8;

        uint16_t tileAddress = tileMap + tileRow * 32 + tileCol;
        
        int16_t tileNum;

        if (unsignedIndex) {
            tileNum = memory.read(tileAddress);
        } else {
            tileNum = static_cast<int8_t>(memory.read(tileAddress));
        }

        uint16_t tileLocation = tileData;
        if (unsignedIndex) {
            tileLocation += tileNum * 16;
        } else {
            tileLocation += (tileNum + 128) * 16;
        }

        int line = windowY % 8;
        uint8_t data1 = memory.read(tileLocation + line * 2);
        uint8_t data2 = memory.read(tileLocation + line * 2 + 1);

        int colorBit = 7 - (windowX % 8);
        int colorNum = ((data2 >> colorBit) & 1) << 1 | ((data1 >> colorBit) & 1);
        
        frameBuffer[scanline][x] = getColor(colorNum, BGP);
    }
}

//renders the sprite layer
void PPU::renderSprites() {
    if (!(LCDC & 0x02)) return; //sprites disabled

    bool use8x16 = (LCDC & 0x04) != 0;
    int spriteHeight = use8x16 ? 16 : 8;

    std::vector<int> spritesOnScanline;
    for (int sprite = 0; sprite < 40; sprite++) {
        uint8_t spriteY = memory.read(0xFE00 + sprite * 4) - 16;
        if (scanline >= spriteY && scanline < spriteY + spriteHeight) {
            spritesOnScanline.push_back(sprite);
            if (spritesOnScanline.size() == 10) break; //max 10 sprites per scanline
        }
    }

    std::sort(spritesOnScanline.begin(), spritesOnScanline.end(), [this](int a, int b) {
        return memory.read(0xFE01 + a * 4) < memory.read(0xFE01 + b * 4);
    });

    for (auto it = spritesOnScanline.rbegin(); it != spritesOnScanline.rend(); ++it) {
        int sprite = *it;
        uint8_t spriteY = memory.read(0xFE00 + sprite * 4) - 16;
        uint8_t spriteX = memory.read(0xFE01 + sprite * 4) - 8;
        uint8_t tileNumber = memory.read(0xFE02 + sprite * 4);
        uint8_t attributes = memory.read(0xFE03 + sprite * 4);

        bool yFlip = attributes & 0x40;
        bool xFlip = attributes & 0x20;
        bool belowBG = attributes & 0x80;
        uint8_t palette = attributes & 0x10 ? OBP1 : OBP0;

        int line = yFlip ? (spriteHeight - 1 - (scanline - spriteY)) : (scanline - spriteY);

        uint16_t tileAddress = 0x8000 + tileNumber * 16 + (line & 0xFE);
        uint8_t data1 = memory.read(tileAddress);
        uint8_t data2 = memory.read(tileAddress + 1);

        for (int pixel = 0; pixel < 8; pixel++) {
            int x = spriteX + (xFlip ? 7 - pixel : pixel);
            if (x < 0 || x >= 160) continue;

            int colorBit = xFlip ? pixel : 7 - pixel;
            int colorNum = ((data2 >> colorBit) & 1) << 1 | ((data1 >> colorBit) & 1);

            if (colorNum == 0) continue; //transparent pixel

            if (!belowBG || frameBuffer[scanline][x] == getColor(0, BGP)) {
                frameBuffer[scanline][x] = getColor(colorNum, palette);
            }
        }
    }
}

//converts a color number and palette to an actual color
uint32_t PPU::getColor(uint8_t colorNum, uint8_t palette) {
    uint8_t shade = (palette >> (colorNum * 2)) & 0x03;
    switch (shade) {
        case 0: return 0xFFFFFFFF; //white
        case 1: return 0xAAAAAA; //light gray
        case 2: return 0x555555; //dark gray
        case 3: return 0x000000; //black
        default: return 0x000000; //should never happen
    }
}

//returns the current frame buffer
const std::array<std::array<uint32_t, 160>, 144>& PPU::getFrameBuffer() const {
    return frameBuffer;
}