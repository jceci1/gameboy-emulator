//gameboy.h
#ifndef GAMEBOY_H
#define GAMEBOY_H

#include "memory.h"
#include "cpu.h"
#include "ppu.h"
#include "timer.h"
#include "interrupt_handler.h"
#include "joypad.h"
#include <string>
#include <vector>
#include <array>
#include <memory>

class GameBoy {
public:
    GameBoy();
    ~GameBoy();

    void loadBootROM(const std::string& filename);
    void loadGameROM(const std::string& filename);
    void run();
    void runFrame();
    void handleInput(int key, bool isPressed);
    const std::array<std::array<uint32_t, 160>, 144>& getFrameBuffer() const;

    //expose Memory for Cartridge setup
    Memory memory;

private:
    CPU cpu;
    PPU ppu;
    Timer timer;
    InterruptHandler interruptHandler;
    Joypad joypad;

    bool isRunning;
    int cyclesThisFrame;

    std::array<std::array<uint32_t, 160>, 144> frameBuffer;

    void step();
};

#endif // GAMEBOY_H