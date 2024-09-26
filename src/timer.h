#ifndef TIMER_H
#define TIMER_H

#include "memory.h"

class Timer {
public:
    Timer(Memory& memory);
    void step(int cycles);

private:
    Memory& memory;
    int dividerCounter;
    int timerCounter;

    uint8_t& DIV;  //divider Register
    uint8_t& TIMA; //timer Counter
    uint8_t& TMA;  //timer Modulo
    uint8_t& TAC;  //timer Control

    void updateDivider(int cycles);
    void updateTimer(int cycles);
    int getTimerThreshold();
};

#endif // TIMER_H