#ifndef CPU_H
#define CPU_H

#include <cstdint>
#include <iostream>
#include "memory.cpp"

class CPU {
    private:
        //registers
        uint8_t A, B, C, D, E, H, L, F;
        uint16_t PC, SP;
        uint16_t AF, BC, DE, HL;
        
        Memory ram;

        int cycles;


    public:
        //constructor
        CPU();

        uint8_t getByte();
        void executeInstruction(uint8_t opcode);
        void nextInstruction();

};


#endif