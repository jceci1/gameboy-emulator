#include "interrupt_handler.h"

//constructor for interrupthandler class
InterruptHandler::InterruptHandler(Memory& mem) : memory(mem), ime(false) {}

//checks for and handles pending interrupts
void InterruptHandler::checkInterrupts() {
    if (!ime) return;

    uint8_t ie = getIE();
    uint8_t if_ = getIF();
    uint8_t interrupts = ie & if_;

    if (interrupts == 0) return;

    //handle interrupts in priority order
    if (interrupts & (1 << VBLANK_INTERRUPT)) {
        setIF(if_ & ~(1 << VBLANK_INTERRUPT));
        ime = false;
        //cpu should jump to vblank_vector
    } else if (interrupts & (1 << LCD_STAT_INTERRUPT)) {
        setIF(if_ & ~(1 << LCD_STAT_INTERRUPT));
        ime = false;
        //cpu should jump to lcd_stat_vector
    } else if (interrupts & (1 << TIMER_INTERRUPT)) {
        setIF(if_ & ~(1 << TIMER_INTERRUPT));
        ime = false;
        //cpu should jump to timer_vector
    } else if (interrupts & (1 << SERIAL_INTERRUPT)) {
        setIF(if_ & ~(1 << SERIAL_INTERRUPT));
        ime = false;
        //cpu should jump to serial_vector
    } else if (interrupts & (1 << JOYPAD_INTERRUPT)) {
        setIF(if_ & ~(1 << JOYPAD_INTERRUPT));
        ime = false;
        //cpu should jump to joypad_vector
    }
}

//checks if any interrupt is pending
bool InterruptHandler::isInterruptPending() const {
    return (getIE() & getIF()) != 0;
}

//gets the vector address of the highest priority pending interrupt
uint16_t InterruptHandler::getInterruptVector() {
    uint8_t ie = getIE();
    uint8_t if_ = getIF();
    uint8_t interrupts = ie & if_;

    if (interrupts & (1 << VBLANK_INTERRUPT)) {
        setIF(if_ & ~(1 << VBLANK_INTERRUPT));
        return VBLANK_VECTOR;
    } else if (interrupts & (1 << LCD_STAT_INTERRUPT)) {
        setIF(if_ & ~(1 << LCD_STAT_INTERRUPT));
        return LCD_STAT_VECTOR;
    } else if (interrupts & (1 << TIMER_INTERRUPT)) {
        setIF(if_ & ~(1 << TIMER_INTERRUPT));
        return TIMER_VECTOR;
    } else if (interrupts & (1 << SERIAL_INTERRUPT)) {
        setIF(if_ & ~(1 << SERIAL_INTERRUPT));
        return SERIAL_VECTOR;
    } else if (interrupts & (1 << JOYPAD_INTERRUPT)) {
        setIF(if_ & ~(1 << JOYPAD_INTERRUPT));
        return JOYPAD_VECTOR;
    }

    return 0; //no interrupt
}

//disables all interrupts
void InterruptHandler::disableInterrupts() {
    ime = false;
}

//enables all interrupts
void InterruptHandler::enableInterrupts() {
    ime = true;
}

//gets the interrupt enable register
uint8_t InterruptHandler::getIE() const {
    return memory.read(0xFFFF);
}

//gets the interrupt flag register
uint8_t InterruptHandler::getIF() const {
    return memory.read(0xFF0F);
}

//sets the interrupt flag register
void InterruptHandler::setIF(uint8_t value) {
    memory.write(0xFF0F, value);
}