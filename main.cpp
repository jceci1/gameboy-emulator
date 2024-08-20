#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdint>


int main() {

    //vector to store boot ROM data
    std::vector<uint8_t> bootRomData;

    //attemps to open boot ROM
    std::ifstream bootRom("C:\\Users\\Joseph\\OneDrive\\Desktop\\gameboy emulator\\ROMS\\dmg_boot.bin", std::ios::binary);
    
    uint8_t byte;

    //ensures boot ROM is open
    if(!bootRom.is_open()) {
        std::cerr << "The boot ROM failed to open" << std::endl;
        
        return 1;
    }


    while(bootRom.read(reinterpret_cast<char*>(&byte), sizeof(byte))) {
        bootRomData.push_back(byte);
    }

    bootRom.close();

    for(std::size_t i = 0; i < bootRomData.size(); i++) {
        std::cout << bootRomData[i] << std::endl;
    }
    

    return 0;
}