// main.cpp
#include <iostream>
#include <string>
#include "gameboy.h"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " < Boot ROM file > < Game ROM file >" << std::endl;
        return 1;
    }

    std::string bootRomPath = argv[1];
    std::string gameRomPath = argv[2];

    GameBoy gameboy;

    try {
        //load the boot ROM
        std::cout << "Loading Boot ROM" << std::endl;
        gameboy.loadBootROM(bootRomPath);
        std::cout << "Boot ROM loaded successfully." << std::endl;

        //load the game ROM
        std::cout << "Loading Game ROM" << std::endl;
        gameboy.loadGameROM(gameRomPath);
        std::cout << "Game ROM loaded successfully." << std::endl;

        //run the emulator
        std::cout << "Starting emulation" << std::endl;
        gameboy.run();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}