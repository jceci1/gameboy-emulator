#include "joypad.h"

Joypad::Joypad(Memory& mem) : 
    memory(mem), 
    buttonStates(0xFF),
    P1(memory.io[0x00])  //P1 is at 0xFF00
{}

void Joypad::pressButton(int button) {
    buttonStates &= ~(1 << button);
    update();
}

void Joypad::releaseButton(int button) {
    buttonStates |= (1 << button);
    update();
}

void Joypad::update() {
    uint8_t selection = P1 & 0x30;
    uint8_t newP1 = selection | 0xCF;  //set bits 4-5 from P1, bits 6-7 to 1, and bits 0-3 to 1 (will be cleared below if button pressed)

    if (!(selection & 0x10)) {  //button keys selected
        if (!(buttonStates & (1 << A)))     newP1 &= ~(1 << 0);
        if (!(buttonStates & (1 << B)))     newP1 &= ~(1 << 1);
        if (!(buttonStates & (1 << SELECT))) newP1 &= ~(1 << 2);
        if (!(buttonStates & (1 << START)))  newP1 &= ~(1 << 3);
    }

    if (!(selection & 0x20)) {  //direction keys selected
        if (!(buttonStates & (1 << RIGHT))) newP1 &= ~(1 << 0);
        if (!(buttonStates & (1 << LEFT)))  newP1 &= ~(1 << 1);
        if (!(buttonStates & (1 << UP)))    newP1 &= ~(1 << 2);
        if (!(buttonStates & (1 << DOWN)))  newP1 &= ~(1 << 3);
    }

    if (newP1 != P1) {
        P1 = newP1;
        requestInterrupt();
    }
}

void Joypad::requestInterrupt() {
    memory.requestInterrupt(4);  //joypad interrupt
}