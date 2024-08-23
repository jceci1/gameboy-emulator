#ifndef MEMORY_H
#define MEMORY_H

#include <cstdint>
#include <iostream>

class Memory {

    public:
        //getters and setters
        uint8_t read(uint16_t loc);
        void write(uint16_t loc, uint8_t val);

    private:
        //64kb worth of memory
        uint8_t ram[65536] = {};

};












#endif