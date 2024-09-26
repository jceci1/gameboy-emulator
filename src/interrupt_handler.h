#ifndef INTERRUPT_HANDLER_H
#define INTERRUPT_HANDLER_H

#include "memory.h"

class InterruptHandler {
public:
    InterruptHandler(Memory& memory);

    void checkInterrupts();
    bool isInterruptPending() const;
    uint16_t getInterruptVector();
    void disableInterrupts();
    void enableInterrupts();

private:
    Memory& memory;
    bool ime; //interrupt Master Enable flag

    //interrupt bit positions
    static constexpr uint8_t VBLANK_INTERRUPT = 0;
    static constexpr uint8_t LCD_STAT_INTERRUPT = 1;
    static constexpr uint8_t TIMER_INTERRUPT = 2;
    static constexpr uint8_t SERIAL_INTERRUPT = 3;
    static constexpr uint8_t JOYPAD_INTERRUPT = 4;

    //interrupt vectors
    static constexpr uint16_t VBLANK_VECTOR = 0x0040;
    static constexpr uint16_t LCD_STAT_VECTOR = 0x0048;
    static constexpr uint16_t TIMER_VECTOR = 0x0050;
    static constexpr uint16_t SERIAL_VECTOR = 0x0058;
    static constexpr uint16_t JOYPAD_VECTOR = 0x0060;

    uint8_t getIE() const;
    uint8_t getIF() const;
    void setIF(uint8_t value);
};

#endif // INTERRUPT_HANDLER_H