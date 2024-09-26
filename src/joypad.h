#ifndef JOYPAD_H
#define JOYPAD_H

#include "memory.h"

class Joypad {
public:
    Joypad(Memory& memory);

    void pressButton(int button);
    void releaseButton(int button);
    void update();

    //button constants
    static const int A = 0;
    static const int B = 1;
    static const int SELECT = 2;
    static const int START = 3;
    static const int RIGHT = 4;
    static const int LEFT = 5;
    static const int UP = 6;
    static const int DOWN = 7;

private:
    Memory& memory;
    uint8_t buttonStates;

    uint8_t& P1;  //joypad register (0xFF00)

    void requestInterrupt();
};

#endif // JOYPAD_H