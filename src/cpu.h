#ifndef CPU_H
#define CPU_H

#include <cstdint>
#include <iostream>

class CPU {
    private:
        //registers
        uint8_t A, B, C, D, E, F, G, H;
        uint16_t PC, SP;

    public:
        //constructor
        CPU();

};


#endif