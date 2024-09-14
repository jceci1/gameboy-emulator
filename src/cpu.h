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

        //private helper functions
        void ADD(uint8_t value);
        void ADC(uint8_t value);
        void SUB(uint8_t value);
        void SBC(uint8_t value);
        void AND(uint8_t value);
        void XOR(uint8_t value);
        void OR(uint8_t value);
        void CP(uint8_t value);  
        void RLC(uint8_t& reg);
        void RRC(uint8_t& reg);
        void RL(uint8_t& reg);
        void RR(uint8_t& reg);
        void SLA(uint8_t& reg);
        void SRA(uint8_t& reg);    
        void SWAP(uint8_t& reg);
        void SRL(uint8_t& reg);
        void BIT(uint8_t bit, uint8_t reg);



    public:
        //constructor
        CPU();

        uint8_t getByte();
        void executeInstruction(uint8_t opcode);
        void executePrefixedInstruction();
        void nextInstruction();
        

};


#endif