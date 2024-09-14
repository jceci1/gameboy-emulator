#define SET_FLAG(f) (F |= (f))
#define CLEAR_FLAG(f) (F &= ~(f))
#define IS_FLAG_SET(f) (F & (f))

#define FLAG_Z 0x80
#define FLAG_N 0x40
#define FLAG_H 0x20
#define FLAG_C 0x10

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
                
            //accounts for zero flag
            if(B == 0) {
                SET_FLAG(FLAG_Z);
            } else {
                CLEAR_FLAG(FLAG_Z);
            }

            //sets negative flag
            SET_FLAG(FLAG_N);

            //accounts for half carry flag
            if((originalB & 0x0F) == 0x00) {
                SET_FLAG(FLAG_H);
            } else {
                CLEAR_FLAG(FLAG_H);
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
            //extracts most significant bit
            uint8_t msb = (A & 0x80) >> 7;
            
            //performs rotation on A
            A = (A << 1) | msb;
            
            //handles carry flag
            if (msb) {
                F |= 0x10;
            } else {
                F &= ~0x10;
            }
            
            //unsets zero, negative, and half-carry flag
            F &= ~0x20;
            F &= ~0x40;
            F &= ~0x80;
            
            cycles += 4;
            break;

        //LD (u16), SP
        case 0x08:
            //gets the 16-bit address from the next two bytes in memory
            uint16_t loc = getByte() | (getByte() << 8);

            //Writes the value of SP to the two memory locations specified 
            ram.write(loc, SP & 0xFF);
            ram.write(loc + 1, (SP >> 8) & 0xFF);

            cycles += 20;
            break;

        //ADD HL, BC 
        case 0x09: 
            //adds together the result
            uint32_t result = HL + BC;
            
            //unsets all flags that will be modified
            F &= ~(FLAG_N | FLAG_H | FLAG_C);
            
            //sets half carry and carry flag if necessary
            if ((HL & 0xFFF) + (BC & 0xFFF) > 0xFFF) F |= FLAG_H;
            if (result > 0xFFFF) F |= FLAG_C;
            
            //sets L and H registers
            L = result & 0xFF;
            H = (result >> 8) & 0xFF;
            cycles += 8;
            break;
    
        //LD A, (BC)
        case 0x0A: 
            A = ram.read(BC);
            cycles += 8;
            break;
    
        //DEC BC
        case 0x0B:
            //decrements each affected register
            C = (BC - 1) & 0xFF;
            B = (BC - 1) >> 8;
            BC -= 1;

            cycles += 8;
            break;
    
        //INC C
        case 0x0C:
            //update registers
            C++;
            BC = (B << 8) | C;
            
            //clear all affected flags
            F &= ~((1 << FLAG_N) | (1 << FLAG_Z) | (1 << FLAG_H));
            
            //sets flags
            if (C == 0) F |= (1 << FLAG_Z);
            if ((C & 0x0F) == 0) F |= (1 << FLAG_H);


            cycles += 4;
            break;
        
        //DEC C
        case 0x0D: 
                //updates registers
                C--;
                BC = (B << 8) | C; 
                
                //sets necessary flags
                F |= (1 << FLAG_N);
                F &= ~((1 << FLAG_Z) | (1 << FLAG_H));
                if (C == 0) F |= (1 << FLAG_Z);
                if ((C & 0x0F) == 0x0F) F |= (1 << FLAG_H);
                
                cycles += 4;
                break;

        //LD C, u8
        case 0x0E: 
            //reads memory into C
            C = getByte();
            
            //updates BC
            BC = (B << 8) | C; 
            
            cycles += 8;
            break;
    
        //RRCA
        case 0x0F:
            //holds carry bit
            uint8_t carry = A & 0x01;
            
            //sets registers
            A = (A >> 1) | (carry << 7);
            
            //unsets all flags
            F = 0;
            
            //sets carry flag if necessary
            if (carry) F |= (1 << FLAG_C);
            
            cycles += 4;
            break;
    
        //STOP
        case 0x10:
            //STOP instruction implementation
            //FIX: This might require additional system-level handling
            cycles += 4;
            break;

        //LD DE, u16
        case 0x11:
            E = getByte();
            D = getByte();
            cycles += 12;
            break;

        //LD (DE), A
        case 0x12:
            ram.write((D << 8) | E, A);
            cycles += 8;
            break;

        //INC DE
        case 0x13:
            //uses bitwise operator to combine D and E
            DE = (D << 8) | E;
            DE++;
            
            //takes high byte
            D = (DE >> 8) & 0xFF;
            
            //takes low byte
            E = DE & 0xFF;
            cycles += 8;
            break;

        //INC D
        case 0x14:
            D++;
            
            //accounts for zero flag
            if(D == 0) {
                F |= 0x80;
            } else {
                F &= ~0x80;
            }

            //clears negative flag
            F &= ~0x40;

            //accounts for half carry flag
            if((D & 0x0F) == 0x00) {
                F |= 0x20;
            } else {
                F &= ~0x20;
            }

            cycles += 4;
            break;
        
        //DEC D
        case 0x15:
            uint8_t originalD = D;
            D--;
                
            //accounts for zero flag
            if(D == 0) {
                SET_FLAG(FLAG_Z);
            } else {
                CLEAR_FLAG(FLAG_Z);
            }

            //sets negative flag
            SET_FLAG(FLAG_N);

            //accounts for half carry flag
            if((originalD & 0x0F) == 0x00) {
                SET_FLAG(FLAG_H);
            } else {
                CLEAR_FLAG(FLAG_H);
            }

            cycles += 4;
            break;

        //LD D, u8
        case 0x16:
            D = getByte();
            cycles += 8;
            break;

        //RLA
        case 0x17:
            //extracts carry flag
            uint8_t carry = (F & 0x10) >> 4;
            
            //extracts most significant bit
            uint8_t msb = (A & 0x80) >> 7;
            
            //performs rotation on A
            A = (A << 1) | carry;
            
            //handles carry flag
            if (msb) {
                F |= 0x10;
            } else {
                F &= ~0x10;
            }
            
            //unsets zero, negative, and half-carry flag
            F &= ~0x20;
            F &= ~0x40;
            F &= ~0x80;
            
            cycles += 4;
            break;

        //JR i8
        case 0x18:
            //gets the signed 8-bit offset
            int8_t offset = static_cast<int8_t>(getByte());

            //adds the offset to PC
            PC += offset;

            cycles += 12;
            break;

        //ADD HL, DE 
        case 0x19: 
            //adds together the result
            uint32_t result = HL + DE;
            
            //unsets all flags that will be modified
            F &= ~(FLAG_N | FLAG_H | FLAG_C);
            
            //sets half carry and carry flag if necessary
            if ((HL & 0xFFF) + (DE & 0xFFF) > 0xFFF) F |= FLAG_H;
            if (result > 0xFFFF) F |= FLAG_C;
            
            //sets L and H registers
            L = result & 0xFF;
            H = (result >> 8) & 0xFF;
            cycles += 8;
            break;
    
        //LD A, (DE)
        case 0x1A: 
            A = ram.read(DE);
            cycles += 8;
            break;
    
        //DEC DE
        case 0x1B:
            //decrements each affected register
            E = (DE - 1) & 0xFF;
            D = (DE - 1) >> 8;
            DE -= 1;

            cycles += 8;
            break;
    
        //INC E
        case 0x1C:
            //updates registers
            E++;
            DE = (D << 8) | E;
            
            //clear all affected flags
            F &= ~((1 << FLAG_N) | (1 << FLAG_Z) | (1 << FLAG_H));
            
            //sets flags
            if (E == 0) F |= (1 << FLAG_Z);
            if ((E & 0x0F) == 0) F |= (1 << FLAG_H);

            cycles += 4;
            break;
        
        //DEC E
        case 0x1D: 
            //updates registers
            E--;
            DE = (D << 8) | E; 
            
            //sets necessary flags
            F |= (1 << FLAG_N);
            F &= ~((1 << FLAG_Z) | (1 << FLAG_H));
            if (E == 0) F |= (1 << FLAG_Z);
            if ((E & 0x0F) == 0x0F) F |= (1 << FLAG_H);
            
            cycles += 4;
            break;

        //LD E, u8
        case 0x1E: 
            //reads memory into E
            E = getByte();
            
            //updates DE
            DE = (D << 8) | E; 
            
            cycles += 8;
            break;
    
        //RRA
        case 0x1F:
            //extracts carry flag
            uint8_t carry = (F & 0x10) >> 4;
            
            //holds least significant bit
            uint8_t lsb = A & 0x01;
            
            //sets registers
            A = (A >> 1) | (carry << 7);
            
            //unsets all flags
            F = 0;
            
            //sets carry flag if necessary
            if (lsb) F |= (1 << FLAG_C);
            
            cycles += 4;
            break;
    
        
        //JR NZ, i8
        case 0x20:
            
            //holds the offset to jump by
            int8_t offset = static_cast<int8_t>(getByte());
            
            //checks jump condition
            if (!IS_FLAG_SET(FLAG_Z)) {
                PC += offset;
                cycles += 12;
            } else {
                cycles += 8;
            }
            
            break;

        //LD HL, u16
        case 0x21:
            //loads proper data into registers
            L = getByte();
            H = getByte();
            
            //updates HL register
            HL = (H << 8) | L;
            cycles += 12;
            
            break;

        //LD (HL+), A
        case 0x22:
            ram.write(HL++, A);
            cycles += 8;
            break;

        //INC HL
        case 0x23:
            //updates registers as necessary
            HL++;
            H = (HL >> 8) & 0xFF;
            L = HL & 0xFF;
            
            cycles += 8;
            break;

        //INC H
        case 0x24:
            H++;
            
            //accounts for zero flag
            if(H == 0) {
                SET_FLAG(FLAG_Z);
            } else {
                CLEAR_FLAG(FLAG_Z);
            }

            //clears negative flag
            CLEAR_FLAG(FLAG_N);

            //accounts for half carry flag
            if((H & 0x0F) == 0x00) {
                SET_FLAG(FLAG_H);
            } else {
                CLEAR_FLAG(FLAG_H);
            }

            cycles += 4;
            break;
        
        //DEC H
        case 0x25:
            {
                uint8_t originalH = H;
                H--;
                
                //accounts for zero flag
                if(H == 0) {
                    SET_FLAG(FLAG_Z);
                } else {
                    CLEAR_FLAG(FLAG_Z);
                }

                //sets negative flag
                SET_FLAG(FLAG_N);

                //accounts for half carry flag
                if((originalH & 0x0F) == 0x00) {
                    SET_FLAG(FLAG_H);
                } else {
                    CLEAR_FLAG(FLAG_H);
                }

                cycles += 4;
            }
            break;

        //LD H, u8
        case 0x26:
            H = getByte();
            HL = (H << 8) | L;
            cycles += 8;
            break;

        //DAA
        case 0x27:
            
            uint16_t correction = 0;

            //checks half carry flag
            if (IS_FLAG_SET(FLAG_H) || (A & 0x0F) > 9) {
                correction |= 0x06;
            }

            //checks carry flag
            if (IS_FLAG_SET(FLAG_C) || A > 0x99) {
                correction |= 0x60;
                SET_FLAG(FLAG_C);
            }

            //updates A register
            A += (IS_FLAG_SET(FLAG_N)) ? -correction : correction;

            //sets flags
            CLEAR_FLAG(FLAG_H);
            if (A == 0) SET_FLAG(FLAG_Z);
            else CLEAR_FLAG(FLAG_Z);

            cycles += 4;
            
            break;

        //JR Z, i8
        case 0x28:
        
            //holds offset to be jumped by
            int8_t offset = static_cast<int8_t>(getByte());
            
            //checks zero flag
            if (IS_FLAG_SET(FLAG_Z)) {
                PC += offset;
                cycles += 12;
            } else {
                cycles += 8;
            }
            
            break;

        //ADD HL, HL
        case 0x29:
            
            uint32_t result = HL + HL;
 
            CLEAR_FLAG(FLAG_N);
                
            //sets relevant flags    
            if ((HL & 0xFFF) + (HL & 0xFFF) > 0xFFF) SET_FLAG(FLAG_H);
            else CLEAR_FLAG(FLAG_H);
                
            if (result > 0xFFFF) SET_FLAG(FLAG_C);
            else CLEAR_FLAG(FLAG_C);
                
            //sets registers    
            HL = result & 0xFFFF;
            H = (HL >> 8) & 0xFF;
            L = HL & 0xFF;
                
            cycles += 8;
            
            break;

        //LD A, (HL+)
        case 0x2A:
            
            //loads memory into A register
            A = ram.read(HL++);
            cycles += 8;
            break;

        //DEC HL
        case 0x2B:
            //updates necessary registers
            HL--;
            H = (HL >> 8) & 0xFF;
            L = HL & 0xFF;
            cycles += 8;
            break;

        //INC L
        case 0x2C:
            L++;
            
            //accounts for zero flag
            if(L == 0) {
                SET_FLAG(FLAG_Z);
            } else {
                CLEAR_FLAG(FLAG_Z);
            }

            //clears negative flag
            CLEAR_FLAG(FLAG_N);

            //accounts for half carry flag
            if((L & 0x0F) == 0x00) {
                SET_FLAG(FLAG_H);
            } else {
                CLEAR_FLAG(FLAG_H);
            }

            HL = (H << 8) | L;
            cycles += 4;
            break;
        
        //DEC L
        case 0x2D:
            {
                uint8_t originalL = L;
                L--;
                
                //accounts for zero flag
                if(L == 0) {
                    SET_FLAG(FLAG_Z);
                } else {
                    CLEAR_FLAG(FLAG_Z);
                }

                //sets negative flag
                SET_FLAG(FLAG_N);

                //accounts for half carry flag
                if((originalL & 0x0F) == 0x00) {
                    SET_FLAG(FLAG_H);
                } else {
                    CLEAR_FLAG(FLAG_H);
                }

                HL = (H << 8) | L;
                cycles += 4;
            }
            break;

        //LD L, u8
        case 0x2E:
            //updates registers
            L = getByte();
            HL = (H << 8) | L;
            cycles += 8;
            break;

        //CPL
        case 0x2F:
            //updates registers and necessary flags
            A = ~A;
            SET_FLAG(FLAG_N);
            SET_FLAG(FLAG_H);
            cycles += 4;
            break;
        
        //JR NC, i8
        case 0x30:
            
            //read the signed 8-bit offset
            int8_t offset = static_cast<int8_t>(getByte());
                
            //check if carry flag is not set
            if (!IS_FLAG_SET(FLAG_C)) {
                //if carry is not set, add offset to PC
                PC += offset;
                cycles += 12;
            } else {
                //if carry is set, don't jump
                cycles += 8;
            }
            
            break;

        //LD SP, u16
        case 0x31:
            //load 16-bit immediate value into SP
            SP = getByte() | (getByte() << 8);
            cycles += 12;
            break;

        //LD (HL-), A
        case 0x32:
            //write A to memory address pointed by HL, then decrement HL
            ram.write(HL--, A);
            cycles += 8;
            break;

        //INC SP
        case 0x33:
            //increment stack pointer
            SP++;
            cycles += 8;
            break;

        //INC (HL)
        case 0x34:
            
            //read value from memory address pointed by HL
            uint8_t value = ram.read(HL);
            value++;

            //write incremented value back to memory
            ram.write(HL, value);

            //update flags
            if(value == 0) {
                SET_FLAG(FLAG_Z);
            } else {
                CLEAR_FLAG(FLAG_Z);
            }
            CLEAR_FLAG(FLAG_N);
            if((value & 0x0F) == 0x00) {
                SET_FLAG(FLAG_H);
            } else {
                CLEAR_FLAG(FLAG_H);
            }

            cycles += 12;
            
            break;

        //DEC (HL)
        case 0x35:
            
            //read value from memory address pointed by HL
            uint8_t value = ram.read(HL);
            uint8_t original = value;
            value--;

            //write decremented value back to memory
            ram.write(HL, value);

            //update flags
            if(value == 0) {
                SET_FLAG(FLAG_Z);
            } else {
                CLEAR_FLAG(FLAG_Z);
            }
            SET_FLAG(FLAG_N);
            if((original & 0x0F) == 0x00) {
                SET_FLAG(FLAG_H);
            } else {
                CLEAR_FLAG(FLAG_H);
            }

            cycles += 12;
            
            break;

        //LD (HL), u8
        case 0x36:
            //load immediate 8-bit value into memory address pointed by HL
            ram.write(HL, getByte());
            cycles += 12;
            break;

        //SCF
        case 0x37:
            //set carry flag
            SET_FLAG(FLAG_C);
            CLEAR_FLAG(FLAG_N);
            CLEAR_FLAG(FLAG_H);
            cycles += 4;
            break;

        //JR C, i8
        case 0x38:
            
            //read the signed 8-bit offset
            int8_t offset = static_cast<int8_t>(getByte());
                
            //check if carry flag is set
            if (IS_FLAG_SET(FLAG_C)) {
                //if carry is set, add offset to PC
                PC += offset;
                cycles += 12;
            } else {
                //if carry is not set, don't jump
                cycles += 8;
            }
            
            break;

        //ADD HL, SP
        case 0x39:
            //perform 16-bit addition
            uint32_t result = HL + SP;
                
            CLEAR_FLAG(FLAG_N);
                
            //check for half carry
            if ((HL & 0xFFF) + (SP & 0xFFF) > 0xFFF) {
                SET_FLAG(FLAG_H);
            } else {
                CLEAR_FLAG(FLAG_H);
            }
                
            //check for carry
            if (result > 0xFFFF) {
                SET_FLAG(FLAG_C);
            } else {
                CLEAR_FLAG(FLAG_C);
            }
                
            //store result in HL
            HL = result & 0xFFFF;
                
            cycles += 8;
            
            break;

        //LD A, (HL-)
        case 0x3A:
            //load value from memory address pointed by HL into A, then decrement HL
            A = ram.read(HL--);
            cycles += 8;
            break;

        //DEC SP
        case 0x3B:
            //decrement stack pointer
            SP--;
            cycles += 8;
            break;

        //INC A
        case 0x3C:
            A++;
            
            //update flags
            if(A == 0) {
                SET_FLAG(FLAG_Z);
            } else {
                CLEAR_FLAG(FLAG_Z);
            }
            CLEAR_FLAG(FLAG_N);
            if((A & 0x0F) == 0x00) {
                SET_FLAG(FLAG_H);
            } else {
                CLEAR_FLAG(FLAG_H);
            }

            cycles += 4;
            break;

        //DEC A
        case 0x3D:
            //holds A and increments
            uint8_t originalA = A;
            A--;
                
            //update flags
            if(A == 0) {
                SET_FLAG(FLAG_Z);
            } else {
                CLEAR_FLAG(FLAG_Z);
            }
            SET_FLAG(FLAG_N);
            if((originalA & 0x0F) == 0x00) {
                SET_FLAG(FLAG_H);
            } else {
                CLEAR_FLAG(FLAG_H);
            }

            cycles += 4;
            
            break;

        //LD A, u8
        case 0x3E:
            //load immediate 8-bit value into A
            A = getByte();
            cycles += 8;
            break;

        //CCF
        case 0x3F:
            //complement carry flag
            if (IS_FLAG_SET(FLAG_C)) {
                CLEAR_FLAG(FLAG_C);
            } else {
                SET_FLAG(FLAG_C);
            }
            CLEAR_FLAG(FLAG_N);
            CLEAR_FLAG(FLAG_H);
            cycles += 4;
            break;

        //LD B, B
        case 0x40:
            //no operation needed, B = B
            B = B;
            cycles += 4;
            break;

        //LD B, C
        case 0x41:
            B = C;
            cycles += 4;
            break;

        //LD B, D
        case 0x42:
            B = D;
            cycles += 4;
            break;

        //LD B, E
        case 0x43:
            B = E;
            cycles += 4;
            break;

        //LD B, H
        case 0x44:
            B = H;
            cycles += 4;
            break;

        //LD B, L
        case 0x45:
            B = L;
            cycles += 4;
            break;

        //LD B, (HL)
        case 0x46:
            //load value from memory address pointed by HL into B
            B = ram.read(HL);
            cycles += 8;
            break;

        //LD B, A
        case 0x47:
            B = A;
            cycles += 4;
            break;

        //LD C, B
        case 0x48:
            C = B;
            cycles += 4;
            break;

        //LD C, C
        case 0x49:
            //no operation needed, C = C
            C = C;
            cycles += 4;
            break;

        //LD C, D
        case 0x4A:
            C = D;
            cycles += 4;
            break;

        //LD C, E
        case 0x4B:
            C = E;
            cycles += 4;
            break;

        //LD C, H
        case 0x4C:
            C = H;
            cycles += 4;
            break;

        //LD C, L
        case 0x4D:
            C = L;
            cycles += 4;
            break;

        //LD C, (HL)
        case 0x4E:
            //load value from memory address pointed by HL into C
            C = ram.read(HL);
            cycles += 8;
            break;

        //LD C, A
        case 0x4F:
            C = A;
            cycles += 4;
            break;

        //LD D, B
        case 0x50:
            D = B;
            cycles += 4;
            break;

        //LD D, C
        case 0x51:
            D = C;
            cycles += 4;
            break;

        //LD D, D
        case 0x52:
            //no operation needed, D = D
            cycles += 4;
            break;

        //LD D, E
        case 0x53:
            D = E;
            cycles += 4;
            break;

        //LD D, H
        case 0x54:
            D = H;
            cycles += 4;
            break;

        //LD D, L
        case 0x55:
            D = L;
            cycles += 4;
            break;

        //LD D, (HL)
        case 0x56:
            //load value from memory address pointed by HL into D
            D = ram.read(HL);
            cycles += 8;
            break;

        //LD D, A
        case 0x57:
            D = A;
            cycles += 4;
            break;

        //LD E, B
        case 0x58:
            E = B;
            cycles += 4;
            break;

        //LD E, C
        case 0x59:
            E = C;
            cycles += 4;
            break;

        //LD E, D
        case 0x5A:
            E = D;
            cycles += 4;
            break;

        //LD E, E
        case 0x5B:
            //no operation needed, E = E
            cycles += 4;
            break;

        //LD E, H
        case 0x5C:
            E = H;
            cycles += 4;
            break;

        //LD E, L
        case 0x5D:
            E = L;
            cycles += 4;
            break;

        //LD E, (HL)
        case 0x5E:
            //load value from memory address pointed by HL into E
            E = ram.read(HL);
            cycles += 8;
            break;

        //LD E, A
        case 0x5F:
            E = A;
            cycles += 4;
            break;

        //LD H, B
        case 0x60:
            H = B;
            cycles += 4;
            break;

        //LD H, C
        case 0x61:
            H = C;
            cycles += 4;
            break;

        //LD H, D
        case 0x62:
            H = D;
            cycles += 4;
            break;

        //LD H, E
        case 0x63:
            H = E;
            cycles += 4;
            break;

        //LD H, H
        case 0x64:
            //no operation needed, H = H
            cycles += 4;
            break;

        //LD H, L
        case 0x65:
            H = L;
            cycles += 4;
            break;

        //LD H, (HL)
        case 0x66:
            //load value from memory address pointed by HL into H
            H = ram.read(HL);
            cycles += 8;
            break;

        //LD H, A
        case 0x67:
            H = A;
            cycles += 4;
            break;

        //LD L, B
        case 0x68:
            L = B;
            cycles += 4;
            break;

        //LD L, C
        case 0x69:
            L = C;
            cycles += 4;
            break;

        //LD L, D
        case 0x6A:
            L = D;
            cycles += 4;
            break;

        //LD L, E
        case 0x6B:
            L = E;
            cycles += 4;
            break;

        //LD L, H
        case 0x6C:
            L = H;
            cycles += 4;
            break;

        //LD L, L
        case 0x6D:
            //no operation needed, L = L
            cycles += 4;
            break;

        //LD L, (HL)
        case 0x6E:
            //load value from memory address pointed by HL into L
            L = ram.read(HL);
            cycles += 8;
            break;

        //LD L, A
        case 0x6F:
            L = A;
            cycles += 4;
            break;

        //LD (HL), B
        case 0x70:
            //store value of B into memory address pointed by HL
            ram.write(HL, B);
            cycles += 8;
            break;

        //LD (HL), C
        case 0x71:
            //store value of C into memory address pointed by HL
            ram.write(HL, C);
            cycles += 8;
            break;

        //LD (HL), D
        case 0x72:
            //store value of D into memory address pointed by HL
            ram.write(HL, D);
            cycles += 8;
            break;

        //LD (HL), E
        case 0x73:
            //store value of E into memory address pointed by HL
            ram.write(HL, E);
            cycles += 8;
            break;

        //LD (HL), H
        case 0x74:
            //store value of H into memory address pointed by HL
            ram.write(HL, H);
            cycles += 8;
            break;

        //LD (HL), L
        case 0x75:
            //store value of L into memory address pointed by HL
            ram.write(HL, L);
            cycles += 8;
            break;

        //HALT
        case 0x76:
            //FIX: Implement HALT functionality
            //this can actually last forever
            cycles += 4;
            break;

        //LD (HL), A
        case 0x77:
            //store value of A into memory address pointed by HL
            ram.write(HL, A);
            cycles += 8;
            break;

        //LD A, B
        case 0x78:
            A = B;
            cycles += 4;
            break;

        //LD A, C
        case 0x79:
            A = C;
            cycles += 4;
            break;

        //LD A, D
        case 0x7A:
            A = D;
            cycles += 4;
            break;

        //LD A, E
        case 0x7B:
            A = E;
            cycles += 4;
            break;

        //LD A, H
        case 0x7C:
            A = H;
            cycles += 4;
            break;

        //LD A, L
        case 0x7D:
            A = L;
            cycles += 4;
            break;

        //LD A, (HL)
        case 0x7E:
            //load value from memory address pointed by HL into A
            A = ram.read(HL);
            cycles += 8;
            break;

        //LD A, A
        case 0x7F:
            //no operation needed, A = A
            cycles += 4;
            break;
    
        //ADD A, B
        case 0x80:
            ADD(B);
            cycles += 4;
            break;

        //ADD A, C
        case 0x81:
            ADD(C);
            cycles += 4;
            break;

        //ADD A, D
        case 0x82:
            ADD(D);
            cycles += 4;
            break;

        //ADD A, E
        case 0x83:
            ADD(E);
            cycles += 4;
            break;

        //ADD A, H
        case 0x84:
            ADD(H);
            cycles += 4;
            break;

        //ADD A, L
        case 0x85:
            ADD(L);
            cycles += 4;
            break;

        //ADD A, (HL)
        case 0x86:
            ADD(ram.read(HL));
            cycles += 8;
            break;

        //ADD A, A
        case 0x87:
            ADD(A);
            cycles += 4;
            break;

        //ADC A, B
        case 0x88:
            ADC(B);
            cycles += 4;
            break;

        //ADC A, C
        case 0x89:
            ADC(C);
            cycles += 4;
            break;

        //ADC A, D
        case 0x8A:
            ADC(D);
            cycles += 4;
            break;

        //ADC A, E
        case 0x8B:
            ADC(E);
            cycles += 4;
            break;

        //ADC A, H
        case 0x8C:
            ADC(H);
            cycles += 4;
            break;

        //ADC A, L
        case 0x8D:
            ADC(L);
            cycles += 4;
            break;

        //ADC A, (HL)
        case 0x8E:
            ADC(ram.read(HL));
            cycles += 8;
            break;

        //ADC A, A
        case 0x8F:
            ADC(A);
            cycles += 4;
            break;

        //SUB A, B
        case 0x90:
            SUB(B);
            cycles += 4;
            break;

        //SUB A, C
        case 0x91:
            SUB(C);
            cycles += 4;
            break;

        //SUB A, D
        case 0x92:
            SUB(D);
            cycles += 4;
            break;

        //SUB A, E
        case 0x93:
            SUB(E);
            cycles += 4;
            break;

        //SUB A, H
        case 0x94:
            SUB(H);
            cycles += 4;
            break;

        //SUB A, L
        case 0x95:
            SUB(L);
            cycles += 4;
            break;

        //SUB A, (HL)
        case 0x96:
            SUB(ram.read(HL));
            cycles += 8;
            break;

        //SUB A, A
        case 0x97:
            SUB(A);
            cycles += 4;
            break;

        //SBC A, B
        case 0x98:
            SBC(B);
            cycles += 4;
            break;

        //SBC A, C
        case 0x99:
            SBC(C);
            cycles += 4;
            break;

        //SBC A, D
        case 0x9A:
            SBC(D);
            cycles += 4;
            break;

        //SBC A, E
        case 0x9B:
            SBC(E);
            cycles += 4;
            break;

        //SBC A, H
        case 0x9C:
            SBC(H);
            cycles += 4;
            break;

        //SBC A, L
        case 0x9D:
            SBC(L);
            cycles += 4;
            break;

        //SBC A, (HL)
        case 0x9E:
            SBC(ram.read(HL));
            cycles += 8;
            break;

        //SBC A, A
        case 0x9F:
            SBC(A);
            cycles += 4;
            break;

        //AND A, B
        case 0xA0:
            AND(B);
            cycles += 4;
            break;

        //AND A, C
        case 0xA1:
            AND(C);
            cycles += 4;
            break;

        //AND A, D
        case 0xA2:
            AND(D);
            cycles += 4;
            break;

        //AND A, E
        case 0xA3:
            AND(E);
            cycles += 4;
            break;

        //AND A, H
        case 0xA4:
            AND(H);
            cycles += 4;
            break;

        //AND A, L
        case 0xA5:
            AND(L);
            cycles += 4;
            break;

        //AND A, (HL)
        case 0xA6:
            AND(ram.read(HL));
            cycles += 8;
            break;

        //AND A, A
        case 0xA7:
            AND(A);
            cycles += 4;
            break;

        //XOR A, B
        case 0xA8:
            XOR(B);
            cycles += 4;
            break;

        //XOR A, C
        case 0xA9:
            XOR(C);
            cycles += 4;
            break;

        //XOR A, D
        case 0xAA:
            XOR(D);
            cycles += 4;
            break;

        //XOR A, E
        case 0xAB:
            XOR(E);
            cycles += 4;
            break;

        //XOR A, H
        case 0xAC:
            XOR(H);
            cycles += 4;
            break;

        //XOR A, L
        case 0xAD:
            XOR(L);
            cycles += 4;
            break;

        //XOR A, (HL)
        case 0xAE:
            XOR(ram.read(HL));
            cycles += 8;
            break;

        //XOR A, A
        case 0xAF:
            XOR(A);
            cycles += 4;
            break;

        //OR A, B
        case 0xB0:
            OR(B);
            cycles += 4;
            break;

        //OR A, C
        case 0xB1:
            OR(C);
            cycles += 4;
            break;

        //OR A, D
        case 0xB2:
            OR(D);
            cycles += 4;
            break;

        //OR A, E
        case 0xB3:
            OR(E);
            cycles += 4;
            break;

        //OR A, H
        case 0xB4:
            OR(H);
            cycles += 4;
            break;

        //OR A, L
        case 0xB5:
            OR(L);
            cycles += 4;
            break;

        //OR A, (HL)
        case 0xB6:
            OR(ram.read(HL));
            cycles += 8;
            break;

        //OR A, A
        case 0xB7:
            OR(A);
            cycles += 4;
            break;

        //CP A, B
        case 0xB8:
            CP(B);
            cycles += 4;
            break;

        //CP A, C
        case 0xB9:
            CP(C);
            cycles += 4;
            break;

        //CP A, D
        case 0xBA:
            CP(D);
            cycles += 4;
            break;

        //CP A, E
        case 0xBB:
            CP(E);
            cycles += 4;
            break;

        //CP A, H
        case 0xBC:
            CP(H);
            cycles += 4;
            break;

        //CP A, L
        case 0xBD:
            CP(L);
            cycles += 4;
            break;

        //CP A, (HL)
        case 0xBE:
            CP(ram.read(HL));
            cycles += 8;
            break;

        //CP A, A
        case 0xBF:
            CP(A);
            cycles += 4;
            break;

    
        //RET NZ
        case 0xC0:
            if (!IS_FLAG_SET(FLAG_Z)) {
                PC = (ram.read(SP + 1) << 8) | ram.read(SP);
                SP += 2;
                cycles += 20;
            } else {
                cycles += 8;
            }
            break;

        //POP BC
        case 0xC1:
            C = ram.read(SP++);
            B = ram.read(SP++);
            cycles += 12;
            break;

        //JP NZ, u16
        case 0xC2:
            
            uint16_t address = getByte() | (getByte() << 8);
            if (!IS_FLAG_SET(FLAG_Z)) {
                PC = address;
                cycles += 16;
            } else {
                cycles += 12;
            }
            
            break;

        //JP u16
        case 0xC3:
            PC = getByte() | (getByte() << 8);
            cycles += 16;
            break;

        //CALL NZ, u16
        case 0xC4:
            
            uint16_t address = getByte() | (getByte() << 8);
            if (!IS_FLAG_SET(FLAG_Z)) {
                SP -= 2;
                ram.write(SP, PC & 0xFF);
                ram.write(SP + 1, PC >> 8);
                PC = address;
                cycles += 24;
            } else {
                cycles += 12;
            }
            
            break;

        //PUSH BC
        case 0xC5:
            SP -= 2;
            ram.write(SP, C);
            ram.write(SP + 1, B);
            cycles += 16;
            break;

        //ADD A, u8
        case 0xC6:
            ADD(getByte());
            cycles += 8;
            break;

        //RST 00h
        case 0xC7:
            SP -= 2;
            ram.write(SP, PC & 0xFF);
            ram.write(SP + 1, PC >> 8);
            PC = 0x0000;
            cycles += 16;
            break;

        //RET Z
        case 0xC8:
            if (IS_FLAG_SET(FLAG_Z)) {
                PC = (ram.read(SP + 1) << 8) | ram.read(SP);
                SP += 2;
                cycles += 20;
            } else {
                cycles += 8;
            }
            break;

        //RET
        case 0xC9:
            PC = (ram.read(SP + 1) << 8) | ram.read(SP);
            SP += 2;
            cycles += 16;
            break;

        //JP Z, u16
        case 0xCA:
            
            uint16_t address = getByte() | (getByte() << 8);
            if (IS_FLAG_SET(FLAG_Z)) {
                PC = address;
                cycles += 16;
            } else {
                cycles += 12;
            }
            
            break;

        //PREFIX CB
        case 0xCB:
            executePrefixedInstruction();
            break;

        //CALL Z, u16
        case 0xCC:
            
            uint16_t address = getByte() | (getByte() << 8);
            if (IS_FLAG_SET(FLAG_Z)) {
                SP -= 2;
                ram.write(SP, PC & 0xFF);
                ram.write(SP + 1, PC >> 8);
                PC = address;
                cycles += 24;
            } else {
                cycles += 12;
            }
            
            break;

        //CALL u16
        case 0xCD:
            
            uint16_t address = getByte() | (getByte() << 8);
            SP -= 2;
            ram.write(SP, PC & 0xFF);
            ram.write(SP + 1, PC >> 8);
            PC = address;
            cycles += 24;
            
            break;

        //ADC A, u8
        case 0xCE:
            ADC(getByte());
            cycles += 8;
            break;

        //RST 08h
        case 0xCF:
            SP -= 2;
            ram.write(SP, PC & 0xFF);
            ram.write(SP + 1, PC >> 8);
            PC = 0x0008;
            cycles += 16;
            break;

        //RET NC
        case 0xD0:
            if (!IS_FLAG_SET(FLAG_C)) {
                PC = (ram.read(SP + 1) << 8) | ram.read(SP);
                SP += 2;
                cycles += 20;
            } else {
                cycles += 8;
            }
            break;

        //POP DE
        case 0xD1:
            E = ram.read(SP++);
            D = ram.read(SP++);
            cycles += 12;
            break;

        //JP NC, u16
        case 0xD2:
            
            uint16_t address = getByte() | (getByte() << 8);
            if (!IS_FLAG_SET(FLAG_C)) {
                PC = address;
                cycles += 16;
            } else {
                cycles += 12;
            }
            
            break;

        //CALL NC, u16
        case 0xD4:
            
            uint16_t address = getByte() | (getByte() << 8);
            if (!IS_FLAG_SET(FLAG_C)) {
                SP -= 2;
                ram.write(SP, PC & 0xFF);
                ram.write(SP + 1, PC >> 8);
                PC = address;
                cycles += 24;
            } else {
                cycles += 12;
            }
            
            break;

        //PUSH DE
        case 0xD5:
            SP -= 2;
            ram.write(SP, E);
            ram.write(SP + 1, D);
            cycles += 16;
            break;

        //SUB A, u8
        case 0xD6:
            SUB(getByte());
            cycles += 8;
            break;

        //RST 10h
        case 0xD7:
            SP -= 2;
            ram.write(SP, PC & 0xFF);
            ram.write(SP + 1, PC >> 8);
            PC = 0x0010;
            cycles += 16;
            break;

        //RET C
        case 0xD8:
            if (IS_FLAG_SET(FLAG_C)) {
                PC = (ram.read(SP + 1) << 8) | ram.read(SP);
                SP += 2;
                cycles += 20;
            } else {
                cycles += 8;
            }
            break;

        //RETI
        case 0xD9:
            PC = (ram.read(SP + 1) << 8) | ram.read(SP);
            SP += 2;
            // Enable interrupts
            // FIX: Implement interrupt handling
            cycles += 16;
            break;

        //JP C, u16
        case 0xDA:
            
            uint16_t address = getByte() | (getByte() << 8);
            if (IS_FLAG_SET(FLAG_C)) {
                PC = address;
                cycles += 16;
            } else {
                cycles += 12;
            }
            
            break;

        //CALL C, u16
        case 0xDC:
            
            uint16_t address = getByte() | (getByte() << 8);
            if (IS_FLAG_SET(FLAG_C)) {
                SP -= 2;
                ram.write(SP, PC & 0xFF);
                ram.write(SP + 1, PC >> 8);
                PC = address;
                cycles += 24;
            } else {
                cycles += 12;
            }
            
            break;

        //SBC A, u8
        case 0xDE:
            SBC(getByte());
            cycles += 8;
            break;

        //RST 18h
        case 0xDF:
            SP -= 2;
            ram.write(SP, PC & 0xFF);
            ram.write(SP + 1, PC >> 8);
            PC = 0x0018;
            cycles += 16;
            break;

        //LD (FF00+u8), A
        case 0xE0:
            
            uint8_t offset = getByte();
            ram.write(0xFF00 + offset, A);
            cycles += 12;
            
            break;

        //POP HL
        case 0xE1:
            L = ram.read(SP++);
            H = ram.read(SP++);
            cycles += 12;
            break;

        //LD (FF00+C), A
        case 0xE2:
            ram.write(0xFF00 + C, A);
            cycles += 8;
            break;

        //PUSH HL
        case 0xE5:
            SP -= 2;
            ram.write(SP, L);
            ram.write(SP + 1, H);
            cycles += 16;
            break;

        //AND A, u8
        case 0xE6:
            AND(getByte());
            cycles += 8;
            break;

        //RST 20h
        case 0xE7:
            SP -= 2;
            ram.write(SP, PC & 0xFF);
            ram.write(SP + 1, PC >> 8);
            PC = 0x0020;
            cycles += 16;
            break;

        //ADD SP, i8
        case 0xE8:
            
            int8_t offset = static_cast<int8_t>(getByte());
            uint32_t result = SP + offset;
            CLEAR_FLAG(FLAG_Z);
            CLEAR_FLAG(FLAG_N);
            ((SP & 0xFF) + (offset & 0xFF) > 0xFF) ? SET_FLAG(FLAG_C) : CLEAR_FLAG(FLAG_C);
            ((SP & 0xF) + (offset & 0xF) > 0xF) ? SET_FLAG(FLAG_H) : CLEAR_FLAG(FLAG_H);
            SP = result & 0xFFFF;
            cycles += 16;
            
            break;

        //JP HL
        case 0xE9:
            PC = HL;
            cycles += 4;
            break;

        //LD (u16), A
        case 0xEA:
            
            uint16_t address = getByte() | (getByte() << 8);
            ram.write(address, A);
            cycles += 16;
            
            break;

        //XOR A, u8
        case 0xEE:
            XOR(getByte());
            cycles += 8;
            break;

        //RST 28h
        case 0xEF:
            SP -= 2;
            ram.write(SP, PC & 0xFF);
            ram.write(SP + 1, PC >> 8);
            PC = 0x0028;
            cycles += 16;
            break;

        //LD A, (FF00+u8)
        case 0xF0:
            
            uint8_t offset = getByte();
            A = ram.read(0xFF00 + offset);
            cycles += 12;
            
            break;

        //POP AF
        case 0xF1:
            F = ram.read(SP++) & 0xF0;  // Lower 4 bits of F are always 0
            A = ram.read(SP++);
            cycles += 12;
            break;

        //LD A, (FF00+C)
        case 0xF2:
            A = ram.read(0xFF00 + C);
            cycles += 8;
            break;

        //DI
        case 0xF3:
            // Disable interrupts
            // FIX: Implement interrupt handling
            cycles += 4;
            break;

        //PUSH AF
        case 0xF5:
            SP -= 2;
            ram.write(SP, F);
            ram.write(SP + 1, A);
            cycles += 16;
            break;

        //OR A, u8
        case 0xF6:
            OR(getByte());
            cycles += 8;
            break;

        //RST 30h
        case 0xF7:
            SP -= 2;
            ram.write(SP, PC & 0xFF);
            ram.write(SP + 1, PC >> 8);
            PC = 0x0030;
            cycles += 16;
            break;

        //LD HL, SP+i8
        case 0xF8:
            
            int8_t offset = static_cast<int8_t>(getByte());
            uint32_t result = SP + offset;
            CLEAR_FLAG(FLAG_Z);
            CLEAR_FLAG(FLAG_N);
            ((SP & 0xFF) + (offset & 0xFF) > 0xFF) ? SET_FLAG(FLAG_C) : CLEAR_FLAG(FLAG_C);
            ((SP & 0xF) + (offset & 0xF) > 0xF) ? SET_FLAG(FLAG_H) : CLEAR_FLAG(FLAG_H);
            HL = result & 0xFFFF;
            cycles += 12;
            
            break;

        //LD SP, HL
        case 0xF9:
            SP = HL;
            cycles += 8;
            break;

        //LD A, (u16)
        case 0xFA:
            
            uint16_t address = getByte() | (getByte() << 8);
            A = ram.read(address);
            cycles += 16;
            
            break;

        //EI
        case 0xFB:
            // Enable interrupts
            // FIX: Implement interrupt handling
            cycles += 4;
            break;

        //CP A, u8
        case 0xFE:
            CP(getByte());
            cycles += 8;
            break;

        //RST 38h
        case 0xFF:
            SP -= 2;
            ram.write(SP, PC & 0xFF);
            ram.write(SP + 1, PC >> 8);
            PC = 0x0038;
            cycles += 16;
            break;
    }
}



void CPU::executePrefixedInstruction() {
    uint8_t opcode = getByte();
    switch(opcode) {
        //RLC B
        case 0x00:
            RLC(B);
            cycles += 8;
            break;

        //RLC C
        case 0x01:
            RLC(C);
            cycles += 8;
            break;

        //RLC D
        case 0x02:
            RLC(D);
            cycles += 8;
            break;

        //RLC E
        case 0x03:
            RLC(E);
            cycles += 8;
            break;

        //RLC H
        case 0x04:
            RLC(H);
            cycles += 8;
            break;

        //RLC L
        case 0x05:
            RLC(L);
            cycles += 8;
            break;

        //RLC (HL)
        case 0x06:
            
            uint8_t value = ram.read(HL);
            RLC(value);
            ram.write(HL, value);
            cycles += 16;
            
            break;

        //RLC A
        case 0x07:
            RLC(A);
            cycles += 8;
            break;

        //RRC B
        case 0x08:
            RRC(B);
            cycles += 8;
            break;

        //RRC C
        case 0x09:
            RRC(C);
            cycles += 8;
            break;

        //RRC D
        case 0x0A:
            RRC(D);
            cycles += 8;
            break;

        //RRC E
        case 0x0B:
            RRC(E);
            cycles += 8;
            break;

        //RRC H
        case 0x0C:
            RRC(H);
            cycles += 8;
            break;

        //RRC L
        case 0x0D:
            RRC(L);
            cycles += 8;
            break;

        //RRC (HL)
        case 0x0E:
            
            uint8_t value = ram.read(HL);
            RRC(value);
            ram.write(HL, value);
            cycles += 16;
            
            break;

        //RRC A
        case 0x0F:
            RRC(A);
            cycles += 8;
            break;

        //RL B
        case 0x10:
            RL(B);
            cycles += 8;
            break;

        //RL C
        case 0x11:
            RL(C);
            cycles += 8;
            break;

        //RL D
        case 0x12:
            RL(D);
            cycles += 8;
            break;

        //RL E
        case 0x13:
            RL(E);
            cycles += 8;
            break;

        //RL H
        case 0x14:
            RL(H);
            cycles += 8;
            break;

        //RL L
        case 0x15:
            RL(L);
            cycles += 8;
            break;

        //RL (HL)
        case 0x16:
            
            uint8_t value = ram.read(HL);
            RL(value);
            ram.write(HL, value);
            cycles += 16;
            
            break;

        //RL A
        case 0x17:
            RL(A);
            cycles += 8;
            break;

        //RR B
        case 0x18:
            RR(B);
            cycles += 8;
            break;

        //RR C
        case 0x19:
            RR(C);
            cycles += 8;
            break;

        //RR D
        case 0x1A:
            RR(D);
            cycles += 8;
            break;

        //RR E
        case 0x1B:
            RR(E);
            cycles += 8;
            break;

        //RR H
        case 0x1C:
            RR(H);
            cycles += 8;
            break;

        //RR L
        case 0x1D:
            RR(L);
            cycles += 8;
            break;

        //RR (HL)
        case 0x1E:
            
            uint8_t value = ram.read(HL);
            RR(value);
            ram.write(HL, value);
            cycles += 16;
            
            break;

        //RR A
        case 0x1F:
            RR(A);
            cycles += 8;
            break;

        //SLA B
        case 0x20:
            SLA(B);
            cycles += 8;
            break;

        //SLA C
        case 0x21:
            SLA(C);
            cycles += 8;
            break;

        //SLA D
        case 0x22:
            SLA(D);
            cycles += 8;
            break;

        //SLA E
        case 0x23:
            SLA(E);
            cycles += 8;
            break;

        //SLA H
        case 0x24:
            SLA(H);
            cycles += 8;
            break;

        //SLA L
        case 0x25:
            SLA(L);
            cycles += 8;
            break;

        //SLA (HL)
        case 0x26:
            
            uint8_t value = ram.read(HL);
            SLA(value);
            ram.write(HL, value);
            cycles += 16;
            
            break;

        //SLA A
        case 0x27:
            SLA(A);
            cycles += 8;
            break;

        //SRA B
        case 0x28:
            SRA(B);
            cycles += 8;
            break;

        //SRA C
        case 0x29:
            SRA(C);
            cycles += 8;
            break;

        //SRA D
        case 0x2A:
            SRA(D);
            cycles += 8;
            break;

        //SRA E
        case 0x2B:
            SRA(E);
            cycles += 8;
            break;

        //SRA H
        case 0x2C:
            SRA(H);
            cycles += 8;
            break;

        //SRA L
        case 0x2D:
            SRA(L);
            cycles += 8;
            break;

        //SRA (HL)
        case 0x2E:
            
            uint8_t value = ram.read(HL);
            SRA(value);
            ram.write(HL, value);
            cycles += 16;
            
            break;

        //SRA A
        case 0x2F:
            SRA(A);
            cycles += 8;
            break;
    
        //swap b
        case 0x30:
            SWAP(B);
            cycles += 8;
            break;

        //swap c
        case 0x31:
            SWAP(C);
            cycles += 8;
            break;

        //swap d
        case 0x32:
            SWAP(D);
            cycles += 8;
            break;

        //swap e
        case 0x33:
            SWAP(E);
            cycles += 8;
            break;

        //swap h
        case 0x34:
            SWAP(H);
            cycles += 8;
            break;

        //swap l
        case 0x35:
            SWAP(L);
            cycles += 8;
            break;

        //swap (hl)
        case 0x36:
            
            uint8_t value = ram.read(HL);
            SWAP(value);
            ram.write(HL, value);
            cycles += 16;
            
            break;

        //swap a
        case 0x37:
            SWAP(A);
            cycles += 8;
            break;

        //srl b
        case 0x38:
            SRL(B);
            cycles += 8;
            break;

        //srl c
        case 0x39:
            SRL(C);
            cycles += 8;
            break;

        //srl d
        case 0x3A:
            SRL(D);
            cycles += 8;
            break;

        //srl e
        case 0x3B:
            SRL(E);
            cycles += 8;
            break;

        //srl h
        case 0x3C:
            SRL(H);
            cycles += 8;
            break;

        //srl l
        case 0x3D:
            SRL(L);
            cycles += 8;
            break;

        //srl (hl)
        case 0x3E:
            
            uint8_t value = ram.read(HL);
            SRL(value);
            ram.write(HL, value);
            cycles += 16;
            
            break;

        //srl a
        case 0x3F:
            SRL(A);
            cycles += 8;
            break;

        //bit 0,b
        case 0x40:
            BIT(0, B);
            cycles += 8;
            break;

        //bit 0,c
        case 0x41:
            BIT(0, C);
            cycles += 8;
            break;

        //bit 0,d
        case 0x42:
            BIT(0, D);
            cycles += 8;
            break;

        //bit 0,e
        case 0x43:
            BIT(0, E);
            cycles += 8;
            break;

        //bit 0,h
        case 0x44:
            BIT(0, H);
            cycles += 8;
            break;

        //bit 0,l
        case 0x45:
            BIT(0, L);
            cycles += 8;
            break;

        //bit 0,(hl)
        case 0x46:
            BIT(0, ram.read(HL));
            cycles += 12;
            break;

        //bit 0,a
        case 0x47:
            BIT(0, A);
            cycles += 8;
            break;

        //bit 1,b
        case 0x48:
            BIT(1, B);
            cycles += 8;
            break;

        //bit 1,c
        case 0x49:
            BIT(1, C);
            cycles += 8;
            break;

        //bit 1,d
        case 0x4A:
            BIT(1, D);
            cycles += 8;
            break;

        //bit 1,e
        case 0x4B:
            BIT(1, E);
            cycles += 8;
            break;

        //bit 1,h
        case 0x4C:
            BIT(1, H);
            cycles += 8;
            break;

        //bit 1,l
        case 0x4D:
            BIT(1, L);
            cycles += 8;
            break;

        //bit 1,(hl)
        case 0x4E:
            BIT(1, ram.read(HL));
            cycles += 12;
            break;

        //bit 1,a
        case 0x4F:
            BIT(1, A);
            cycles += 8;
            break;

        //bit 2,b
        case 0x50:
            BIT(2, B);
            cycles += 8;
            break;

        //bit 2,c
        case 0x51:
            BIT(2, C);
            cycles += 8;
            break;

        //bit 2,d
        case 0x52:
            BIT(2, D);
            cycles += 8;
            break;

        //bit 2,e
        case 0x53:
            BIT(2, E);
            cycles += 8;
            break;

        //bit 2,h
        case 0x54:
            BIT(2, H);
            cycles += 8;
            break;

        //bit 2,l
        case 0x55:
            BIT(2, L);
            cycles += 8;
            break;

        //bit 2,(hl)
        case 0x56:
            BIT(2, ram.read(HL));
            cycles += 12;
            break;

        //bit 2,a
        case 0x57:
            BIT(2, A);
            cycles += 8;
            break;

        //bit 3,b
        case 0x58:
            BIT(3, B);
            cycles += 8;
            break;

        //bit 3,c
        case 0x59:
            BIT(3, C);
            cycles += 8;
            break;

        //bit 3,d
        case 0x5A:
            BIT(3, D);
            cycles += 8;
            break;

        //bit 3,e
        case 0x5B:
            BIT(3, E);
            cycles += 8;
            break;

        //bit 3,h
        case 0x5C:
            BIT(3, H);
            cycles += 8;
            break;

        //bit 3,l
        case 0x5D:
            BIT(3, L);
            cycles += 8;
            break;

        //bit 3,(hl)
        case 0x5E:
            BIT(3, ram.read(HL));
            cycles += 12;
            break;

        //bit 3,a
        case 0x5F:
            BIT(3, A);
            cycles += 8;
            break;

        //bit 4,b
        case 0x60:
            BIT(4, B);
            cycles += 8;
            break;

        //bit 4,c
        case 0x61:
            BIT(4, C);
            cycles += 8;
            break;

        //bit 4,d
        case 0x62:
            BIT(4, D);
            cycles += 8;
            break;

        //bit 4,e
        case 0x63:
            BIT(4, E);
            cycles += 8;
            break;

        //bit 4,h
        case 0x64:
            BIT(4, H);
            cycles += 8;
            break;

        //bit 4,l
        case 0x65:
            BIT(4, L);
            cycles += 8;
            break;

        //bit 4,(hl)
        case 0x66:
            BIT(4, ram.read(HL));
            cycles += 12;
            break;

        //bit 4,a
        case 0x67:
            BIT(4, A);
            cycles += 8;
            break;

        //bit 5,b
        case 0x68:
            BIT(5, B);
            cycles += 8;
            break;

        //bit 5,c
        case 0x69:
            BIT(5, C);
            cycles += 8;
            break;

        //bit 5,d
        case 0x6A:
            BIT(5, D);
            cycles += 8;
            break;

        //bit 5,e
        case 0x6B:
            BIT(5, E);
            cycles += 8;
            break;

        //bit 5,h
        case 0x6C:
            BIT(5, H);
            cycles += 8;
            break;

        //bit 5,l
        case 0x6D:
            BIT(5, L);
            cycles += 8;
            break;

        //bit 5,(hl)
        case 0x6E:
            BIT(5, ram.read(HL));
            cycles += 12;
            break;

        //bit 5,a
        case 0x6F:
            BIT(5, A);
            cycles += 8;
            break;

        switch(opcode) {
        // ... (previous cases)

        //bit 6,b
        case 0x70:
            BIT(6, B);
            cycles += 8;
            break;

        //bit 6,c
        case 0x71:
            BIT(6, C);
            cycles += 8;
            break;

        //bit 6,d
        case 0x72:
            BIT(6, D);
            cycles += 8;
            break;

        //bit 6,e
        case 0x73:
            BIT(6, E);
            cycles += 8;
            break;

        //bit 6,h
        case 0x74:
            BIT(6, H);
            cycles += 8;
            break;

        //bit 6,l
        case 0x75:
            BIT(6, L);
            cycles += 8;
            break;

        //bit 6,(hl)
        case 0x76:
            BIT(6, ram.read(HL));
            cycles += 12;
            break;

        //bit 6,a
        case 0x77:
            BIT(6, A);
            cycles += 8;
            break;

        //bit 7,b
        case 0x78:
            BIT(7, B);
            cycles += 8;
            break;

        //bit 7,c
        case 0x79:
            BIT(7, C);
            cycles += 8;
            break;

        //bit 7,d
        case 0x7A:
            BIT(7, D);
            cycles += 8;
            break;

        //bit 7,e
        case 0x7B:
            BIT(7, E);
            cycles += 8;
            break;

        //bit 7,h
        case 0x7C:
            BIT(7, H);
            cycles += 8;
            break;

        //bit 7,l
        case 0x7D:
            BIT(7, L);
            cycles += 8;
            break;

        //bit 7,(hl)
        case 0x7E:
            BIT(7, ram.read(HL));
            cycles += 12;
            break;

        //bit 7,a
        case 0x7F:
            BIT(7, A);
            cycles += 8;
            break;

        //res 0,b
        case 0x80:
            B &= ~(1 << 0);
            cycles += 8;
            break;

        //res 0,c
        case 0x81:
            C &= ~(1 << 0);
            cycles += 8;
            break;

        //res 0,d
        case 0x82:
            D &= ~(1 << 0);
            cycles += 8;
            break;

        //res 0,e
        case 0x83:
            E &= ~(1 << 0);
            cycles += 8;
            break;

        //res 0,h
        case 0x84:
            H &= ~(1 << 0);
            cycles += 8;
            break;

        //res 0,l
        case 0x85:
            L &= ~(1 << 0);
            cycles += 8;
            break;

        //res 0,(hl)
        case 0x86:
            ram.write(HL, ram.read(HL) & ~(1 << 0));
            cycles += 16;
            break;

        //res 0,a
        case 0x87:
            A &= ~(1 << 0);
            cycles += 8;
            break;

        //res 1,b
        case 0x88:
            B &= ~(1 << 1);
            cycles += 8;
            break;

        //res 1,c
        case 0x89:
            C &= ~(1 << 1);
            cycles += 8;
            break;

        //res 1,d
        case 0x8A:
            D &= ~(1 << 1);
            cycles += 8;
            break;

        //res 1,e
        case 0x8B:
            E &= ~(1 << 1);
            cycles += 8;
            break;

        //res 1,h
        case 0x8C:
            H &= ~(1 << 1);
            cycles += 8;
            break;

        //res 1,l
        case 0x8D:
            L &= ~(1 << 1);
            cycles += 8;
            break;

        //res 1,(hl)
        case 0x8E:
            ram.write(HL, ram.read(HL) & ~(1 << 1));
            cycles += 16;
            break;

        //res 1,a
        case 0x8F:
            A &= ~(1 << 1);
            cycles += 8;
            break;

        //res 2,b
        case 0x90:
            B &= ~(1 << 2);
            cycles += 8;
            break;

        //res 2,c
        case 0x91:
            C &= ~(1 << 2);
            cycles += 8;
            break;

        //res 2,d
        case 0x92:
            D &= ~(1 << 2);
            cycles += 8;
            break;

        //res 2,e
        case 0x93:
            E &= ~(1 << 2);
            cycles += 8;
            break;

        //res 2,h
        case 0x94:
            H &= ~(1 << 2);
            cycles += 8;
            break;

        //res 2,l
        case 0x95:
            L &= ~(1 << 2);
            cycles += 8;
            break;

        //res 2,(hl)
        case 0x96:
            ram.write(HL, ram.read(HL) & ~(1 << 2));
            cycles += 16;
            break;

        //res 2,a
        case 0x97:
            A &= ~(1 << 2);
            cycles += 8;
            break;

        //res 3,b
        case 0x98:
            B &= ~(1 << 3);
            cycles += 8;
            break;

        //res 3,c
        case 0x99:
            C &= ~(1 << 3);
            cycles += 8;
            break;

        //res 3,d
        case 0x9A:
            D &= ~(1 << 3);
            cycles += 8;
            break;

        //res 3,e
        case 0x9B:
            E &= ~(1 << 3);
            cycles += 8;
            break;

        //res 3,h
        case 0x9C:
            H &= ~(1 << 3);
            cycles += 8;
            break;

        //res 3,l
        case 0x9D:
            L &= ~(1 << 3);
            cycles += 8;
            break;

        //res 3,(hl)
        case 0x9E:
            ram.write(HL, ram.read(HL) & ~(1 << 3));
            cycles += 16;
            break;

        //res 3,a
        case 0x9F:
            A &= ~(1 << 3);
            cycles += 8;
            break;

        //res 4,b
        case 0xA0:
            B &= ~(1 << 4);
            cycles += 8;
            break;

        //res 4,c
        case 0xA1:
            C &= ~(1 << 4);
            cycles += 8;
            break;

        //res 4,d
        case 0xA2:
            D &= ~(1 << 4);
            cycles += 8;
            break;

        //res 4,e
        case 0xA3:
            E &= ~(1 << 4);
            cycles += 8;
            break;

        //res 4,h
        case 0xA4:
            H &= ~(1 << 4);
            cycles += 8;
            break;

        //res 4,l
        case 0xA5:
            L &= ~(1 << 4);
            cycles += 8;
            break;

        //res 4,(hl)
        case 0xA6:
            ram.write(HL, ram.read(HL) & ~(1 << 4));
            cycles += 16;
            break;

        //res 4,a
        case 0xA7:
            A &= ~(1 << 4);
            cycles += 8;
            break;

        //res 5,b
        case 0xA8:
            B &= ~(1 << 5);
            cycles += 8;
            break;

        //res 5,c
        case 0xA9:
            C &= ~(1 << 5);
            cycles += 8;
            break;

        //res 5,d
        case 0xAA:
            D &= ~(1 << 5);
            cycles += 8;
            break;

        //res 5,e
        case 0xAB:
            E &= ~(1 << 5);
            cycles += 8;
            break;

        //res 5,h
        case 0xAC:
            H &= ~(1 << 5);
            cycles += 8;
            break;

        //res 5,l
        case 0xAD:
            L &= ~(1 << 5);
            cycles += 8;
            break;

        //res 5,(hl)
        case 0xAE:
            ram.write(HL, ram.read(HL) & ~(1 << 5));
            cycles += 16;
            break;

        //res 5,a
        case 0xAF:
            A &= ~(1 << 5);
            cycles += 8;
            break;

        //res 6,b
        case 0xB0:
            B &= ~(1 << 6);
            cycles += 8;
            break;

        //res 6,c
        case 0xB1:
            C &= ~(1 << 6);
            cycles += 8;
            break;

        //res 6,d
        case 0xB2:
            D &= ~(1 << 6);
            cycles += 8;
            break;

        //res 6,e
        case 0xB3:
            E &= ~(1 << 6);
            cycles += 8;
            break;

        //res 6,h
        case 0xB4:
            H &= ~(1 << 6);
            cycles += 8;
            break;

        //res 6,l
        case 0xB5:
            L &= ~(1 << 6);
            cycles += 8;
            break;

        //res 6,(hl)
        case 0xB6:
            ram.write(HL, ram.read(HL) & ~(1 << 6));
            cycles += 16;
            break;

        //res 6,a
        case 0xB7:
            A &= ~(1 << 6);
            cycles += 8;
            break;

        //res 7,b
        case 0xB8:
            B &= ~(1 << 7);
            cycles += 8;
            break;

        //res 7,c
        case 0xB9:
            C &= ~(1 << 7);
            cycles += 8;
            break;

        //res 7,d
        case 0xBA:
            D &= ~(1 << 7);
            cycles += 8;
            break;

        //res 7,e
        case 0xBB:
            E &= ~(1 << 7);
            cycles += 8;
            break;

        //res 7,h
        case 0xBC:
            H &= ~(1 << 7);
            cycles += 8;
            break;

        //res 7,l
        case 0xBD:
            L &= ~(1 << 7);
            cycles += 8;
            break;

        //res 7,(hl)
        case 0xBE:
            ram.write(HL, ram.read(HL) & ~(1 << 7));
            cycles += 16;
            break;

        //res 7,a
        case 0xBF:
            A &= ~(1 << 7);
            cycles += 8;
            break;

        //set 0,b
        case 0xC0:
            B |= (1 << 0);
            cycles += 8;
            break;

        //set 0,c
        case 0xC1:
            C |= (1 << 0);
            cycles += 8;
            break;

        //set 0,d
        case 0xC2:
            D |= (1 << 0);
            cycles += 8;
            break;

        //set 0,e
        case 0xC3:
            E |= (1 << 0);
            cycles += 8;
            break;

        //set 0,h
        case 0xC4:
            H |= (1 << 0);
            cycles += 8;
            break;

        //set 0,l
        case 0xC5:
            L |= (1 << 0);
            cycles += 8;
            break;

        //set 0,(hl)
        case 0xC6:
            ram.write(HL, ram.read(HL) | (1 << 0));
            cycles += 16;
            break;

        //set 0,a
        case 0xC7:
            A |= (1 << 0);
            cycles += 8;
            break;

        //set 1,b
        case 0xC8:
            B |= (1 << 1);
            cycles += 8;
            break;

        //set 1,c
        case 0xC9:
            C |= (1 << 1);
            cycles += 8;
            break;

        //set 1,d
        case 0xCA:
            D |= (1 << 1);
            cycles += 8;
            break;

        //set 1,e
        case 0xCB:
            E |= (1 << 1);
            cycles += 8;
            break;

        //set 1,h
        case 0xCC:
            H |= (1 << 1);
            cycles += 8;
            break;

        //set 1,l
        case 0xCD:
            L |= (1 << 1);
            cycles += 8;
            break;

        //set 1,(hl)
        case 0xCE:
            ram.write(HL, ram.read(HL) | (1 << 1));
            cycles += 16;
            break;

        //set 1,a
        case 0xCF:
            A |= (1 << 1);
            cycles += 8;
            break;

        //set 2,b
        case 0xD0:
            B |= (1 << 2);
            cycles += 8;
            break;

        //set 2,c
        case 0xD1:
            C |= (1 << 2);
            cycles += 8;
            break;

        //set 2,d
        case 0xD2:
            D |= (1 << 2);
            cycles += 8;
            break;

        //set 2,e
        case 0xD3:
            E |= (1 << 2);
            cycles += 8;
            break;

        //set 2,h
        case 0xD4:
            H |= (1 << 2);
            cycles += 8;
            break;

        //set 2,l
        case 0xD5:
            L |= (1 << 2);
            cycles += 8;
            break;

        //set 2,(hl)
        case 0xD6:
            ram.write(HL, ram.read(HL) | (1 << 2));
            cycles += 16;
            break;

        //set 2,a
        case 0xD7:
            A |= (1 << 2);
            cycles += 8;
            break;

        //set 3,b
        case 0xD8:
            B |= (1 << 3);
            cycles += 8;
            break;

        //set 3,c
        case 0xD9:
            C |= (1 << 3);
            cycles += 8;
            break;

        //set 3,d
        case 0xDA:
            D |= (1 << 3);
            cycles += 8;
            break;

        //set 3,e
        case 0xDB:
            E |= (1 << 3);
            cycles += 8;
            break;

        //set 3,h
        case 0xDC:
            H |= (1 << 3);
            cycles += 8;
            break;

        //set 3,l
        case 0xDD:
            L |= (1 << 3);
            cycles += 8;
            break;

        //set 3,(hl)
        case 0xDE:
            ram.write(HL, ram.read(HL) | (1 << 3));
            cycles += 16;
            break;

        //set 3,a
        case 0xDF:
            A |= (1 << 3);
            cycles += 8;
            break;
    
        //set 4,b
        case 0xE0:
            B |= (1 << 4);
            cycles += 8;
            break;

        //set 4,c
        case 0xE1:
            C |= (1 << 4);
            cycles += 8;
            break;

        //set 4,d
        case 0xE2:
            D |= (1 << 4);
            cycles += 8;
            break;

        //set 4,e
        case 0xE3:
            E |= (1 << 4);
            cycles += 8;
            break;

        //set 4,h
        case 0xE4:
            H |= (1 << 4);
            cycles += 8;
            break;

        //set 4,l
        case 0xE5:
            L |= (1 << 4);
            cycles += 8;
            break;

        //set 4,(hl)
        case 0xE6:
            ram.write(HL, ram.read(HL) | (1 << 4));
            cycles += 16;
            break;

        //set 4,a
        case 0xE7:
            A |= (1 << 4);
            cycles += 8;
            break;

        //set 5,b
        case 0xE8:
            B |= (1 << 5);
            cycles += 8;
            break;

        //set 5,c
        case 0xE9:
            C |= (1 << 5);
            cycles += 8;
            break;

        //set 5,d
        case 0xEA:
            D |= (1 << 5);
            cycles += 8;
            break;

        //set 5,e
        case 0xEB:
            E |= (1 << 5);
            cycles += 8;
            break;

        //set 5,h
        case 0xEC:
            H |= (1 << 5);
            cycles += 8;
            break;

        //set 5,l
        case 0xED:
            L |= (1 << 5);
            cycles += 8;
            break;

        //set 5,(hl)
        case 0xEE:
            ram.write(HL, ram.read(HL) | (1 << 5));
            cycles += 16;
            break;

        //set 5,a
        case 0xEF:
            A |= (1 << 5);
            cycles += 8;
            break;

        //set 6,b
        case 0xF0:
            B |= (1 << 6);
            cycles += 8;
            break;

        //set 6,c
        case 0xF1:
            C |= (1 << 6);
            cycles += 8;
            break;

        //set 6,d
        case 0xF2:
            D |= (1 << 6);
            cycles += 8;
            break;

        //set 6,e
        case 0xF3:
            E |= (1 << 6);
            cycles += 8;
            break;

        //set 6,h
        case 0xF4:
            H |= (1 << 6);
            cycles += 8;
            break;

        //set 6,l
        case 0xF5:
            L |= (1 << 6);
            cycles += 8;
            break;

        //set 6,(hl)
        case 0xF6:
            ram.write(HL, ram.read(HL) | (1 << 6));
            cycles += 16;
            break;

        //set 6,a
        case 0xF7:
            A |= (1 << 6);
            cycles += 8;
            break;

        //set 7,b
        case 0xF8:
            B |= (1 << 7);
            cycles += 8;
            break;

        //set 7,c
        case 0xF9:
            C |= (1 << 7);
            cycles += 8;
            break;

        //set 7,d
        case 0xFA:
            D |= (1 << 7);
            cycles += 8;
            break;

        //set 7,e
        case 0xFB:
            E |= (1 << 7);
            cycles += 8;
            break;

        //set 7,h
        case 0xFC:
            H |= (1 << 7);
            cycles += 8;
            break;

        //set 7,l
        case 0xFD:
            L |= (1 << 7);
            cycles += 8;
            break;

        //set 7,(hl)
        case 0xFE:
            ram.write(HL, ram.read(HL) | (1 << 7));
            cycles += 16;
            break;

        //set 7,a
        case 0xFF:
            A |= (1 << 7);
            cycles += 8;
            break;
    
    }
}




//helper functions
void CPU::ADD(uint8_t value) {
    uint16_t result = A + value;
    
    //clear necessary flags
    CLEAR_FLAG(FLAG_N);
    
    //set affected flags
    ((A & 0xF) + (value & 0xF) > 0xF) ? SET_FLAG(FLAG_H) : CLEAR_FLAG(FLAG_H);
    (result > 0xFF) ? SET_FLAG(FLAG_C) : CLEAR_FLAG(FLAG_C);
    A = result & 0xFF;
    (A == 0) ? SET_FLAG(FLAG_Z) : CLEAR_FLAG(FLAG_Z);
}

void CPU::ADC(uint8_t value) {
    uint16_t result = A + value + (F & FLAG_C ? 1 : 0);
    
    //clears negative flag
    CLEAR_FLAG(FLAG_N);
    
    //sets flags
    ((A & 0xF) + (value & 0xF) + (F & FLAG_C ? 1 : 0) > 0xF) ? SET_FLAG(FLAG_H) : CLEAR_FLAG(FLAG_H);
    (result > 0xFF) ? SET_FLAG(FLAG_C) : CLEAR_FLAG(FLAG_C);
    A = result & 0xFF;
    (A == 0) ? SET_FLAG(FLAG_Z) : CLEAR_FLAG(FLAG_Z);
}

void CPU::SUB(uint8_t value) {
    uint8_t result = A - value;
    
    //sets necessary flags
    SET_FLAG(FLAG_N);
    (A & 0xF) < (value & 0xF) ? SET_FLAG(FLAG_H) : CLEAR_FLAG(FLAG_H);
    A < value ? SET_FLAG(FLAG_C) : CLEAR_FLAG(FLAG_C);
    A = result;
    (A == 0) ? SET_FLAG(FLAG_Z) : CLEAR_FLAG(FLAG_Z);
}

void CPU::SBC(uint8_t value) {
    uint8_t carry = (F & FLAG_C) ? 1 : 0;
    uint8_t result = A - value - carry;
    
    //checks and sets flags
    SET_FLAG(FLAG_N);
    ((A & 0xF) < ((value & 0xF) + carry)) ? SET_FLAG(FLAG_H) : CLEAR_FLAG(FLAG_H);
    (A < (value + carry)) ? SET_FLAG(FLAG_C) : CLEAR_FLAG(FLAG_C);
    A = result;
    (A == 0) ? SET_FLAG(FLAG_Z) : CLEAR_FLAG(FLAG_Z);
}

void CPU::AND(uint8_t value) {
    A &= value;
    
    //checks and sets flags
    (A == 0) ? SET_FLAG(FLAG_Z) : CLEAR_FLAG(FLAG_Z);
    CLEAR_FLAG(FLAG_N);
    SET_FLAG(FLAG_H);
    CLEAR_FLAG(FLAG_C);
}

void CPU::XOR(uint8_t value) {
    A ^= value;
    
    //checks and sets flags
    (A == 0) ? SET_FLAG(FLAG_Z) : CLEAR_FLAG(FLAG_Z);
    CLEAR_FLAG(FLAG_N);
    CLEAR_FLAG(FLAG_H);
    CLEAR_FLAG(FLAG_C);
}

void CPU::OR(uint8_t value) {
    A |= value;
    
    //checks and sets flags
    (A == 0) ? SET_FLAG(FLAG_Z) : CLEAR_FLAG(FLAG_Z);
    CLEAR_FLAG(FLAG_N);
    CLEAR_FLAG(FLAG_H);
    CLEAR_FLAG(FLAG_C);
}

void CPU::CP(uint8_t value) {
    uint8_t result = A - value;
    
    //checks and sets flags
    (result == 0) ? SET_FLAG(FLAG_Z) : CLEAR_FLAG(FLAG_Z);
    SET_FLAG(FLAG_N);
    ((A & 0xF) < (value & 0xF)) ? SET_FLAG(FLAG_H) : CLEAR_FLAG(FLAG_H);
    (A < value) ? SET_FLAG(FLAG_C) : CLEAR_FLAG(FLAG_C);
}


void CPU::RLC(uint8_t& reg) {
    uint8_t carry = (reg & 0x80) >> 7;
    reg = (reg << 1) | carry;
    F = (reg == 0 ? FLAG_Z : 0) | (carry ? FLAG_C : 0);
}

void CPU::RRC(uint8_t& reg) {
    uint8_t carry = reg & 0x01;
    reg = (reg >> 1) | (carry << 7);
    F = (reg == 0 ? FLAG_Z : 0) | (carry ? FLAG_C : 0);
}

void CPU::RL(uint8_t& reg) {
    uint8_t oldCarry = F & FLAG_C ? 1 : 0;
    uint8_t newCarry = (reg & 0x80) >> 7;
    reg = (reg << 1) | oldCarry;
    F = (reg == 0 ? FLAG_Z : 0) | (newCarry ? FLAG_C : 0);
}

void CPU::RR(uint8_t& reg) {
    uint8_t oldCarry = F & FLAG_C ? 1 : 0;
    uint8_t newCarry = reg & 0x01;
    reg = (reg >> 1) | (oldCarry << 7);
    F = (reg == 0 ? FLAG_Z : 0) | (newCarry ? FLAG_C : 0);
}

void CPU::SLA(uint8_t& reg) {
    uint8_t carry = (reg & 0x80) >> 7;
    reg <<= 1;
    F = (reg == 0 ? FLAG_Z : 0) | (carry ? FLAG_C : 0);
}

void CPU::SRA(uint8_t& reg) {
    uint8_t carry = reg & 0x01;
    reg = (reg & 0x80) | (reg >> 1);
    F = (reg == 0 ? FLAG_Z : 0) | (carry ? FLAG_C : 0);
}

void CPU::SWAP(uint8_t& reg) {
    reg = ((reg & 0xF0) >> 4) | ((reg & 0x0F) << 4);
    F = reg == 0 ? FLAG_Z : 0;
}

void CPU::SRL(uint8_t& reg) {
    uint8_t carry = reg & 0x01;
    reg >>= 1;
    F = (reg == 0 ? FLAG_Z : 0) | (carry ? FLAG_C : 0);
}

void CPU::BIT(uint8_t bit, uint8_t reg) {
    F = (F & FLAG_C) | FLAG_H | ((reg & (1 << bit)) ? 0 : FLAG_Z);
}