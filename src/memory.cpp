#include "memory.h"

//returns contents of the memory from the desired location
uint8_t Memory::read(uint16_t loc) {
    return ram[loc];
}


//sets desired location to given value
void Memory::write(uint16_t loc, uint8_t val) {
    ram[loc] = val;
}

