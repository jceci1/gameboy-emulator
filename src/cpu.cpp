#include "cpu.h"

//constructor
CPU::CPU() {
    A = 0;
    B = 0;
    C = 0;
    D = 0;
    E = 0;
    H = 0;
    L = 0;
    F = 0;
    PC = 0x0100;
    SP = 0xFFFE;
    AF = 0;
    BC = 0;
    DE = 0;
    HL = 0;
    cycles = 0;
}


uint8_t CPU::getByte() {
    //FIX: There may be a problem here where incrementing PC by doing "PC++" does not give the desired result.
    return ram.read(PC++);
}



void CPU::nextInstruction() {
    uint8_t opcode = getByte();
    executeInstruction(opcode);
}



void CPU::executeInstruction(uint8_t opcode) {
    switch(opcode) {
        //NOP
        case 0x00:
            cycles += 4;
            break;

        //LD BC, u16
        case 0x01:
            C = getByte();
            B = getByte();
            cycles += 12;
            break;

        //LD (BC), A
        case 0x02:
            ram.write((B << 8) | C, A);
            cycles += 8;
            break;

        //INC BC
        case 0x03:
            //uses bitwise operator to combine B and C
            BC = (B << 8) | C;
            BC++;
            
            //takes high byte
            B = (BC >> 8) & 0xFF;
            
            //takes low byte
            C = BC & 0xFF;
            cycles += 8;
            break;

        //INC B
        case 0x04:
            B++;
            
            //accounts for 0 flag
            if(B == 0) {
                F |= 0x80;
            } else {
                F &= ~0x80;
            }

            //clears negative flag
            F &= ~0x40;

            //accounts for half carry flag
            if((B & 0x0F) == 0x00) {
                F |= 0x20;
            } else {
                F &= ~0x20;
            }

            cycles += 4;
            break;
        
        //DEC B
        case 0x05:
            uint8_t originalB = B;

            B--;
            
            //accounts for 0 flag
            if(B == 0) {
                F |= 0x80;
            } else {
                F &= ~0x80;
            }

            //sets negative flag
            F |= 0x40;

            //accounts for half carry flag
            if((originalB & 0x0F) == 0x00) {
                F |= 0x20;
            } else {
                F &= ~0x20;
            }

            cycles += 4;
            break;

        //LD B, u8
        case 0x06:
            B = getByte();
            cycles += 8;
            break;

        
        //RLCA
        case 0x07:

            cycles += 4;
            break;

    }
}
