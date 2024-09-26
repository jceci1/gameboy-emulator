﻿# gameboy-emulator
This is a basic gameboy emulator (still in progress) written in C++. Does not include audio support.

# Building
To build the project, cmake, and SDL2 are required. The Cmake file is included in the repository.

# Running
Running the program is done in the following format:

`./gameboy_emulator path/to/boot.rom path/to/game.rom`


## Game Controls

| Gameboy Button | Keyboard Key |
|:--------------:|:------------:|
|       A        |      Z       |
|       B        |      X       |
|     Start      |      S       |
|     Select     |      A       |

## D-Pad

|   Direction   | Keyboard Key |
|:-------------:|:------------:|
|      Up       |   ↑ (Up)     |
|     Down      |   ↓ (Down)   |
|     Left      |   ← (Left)   |
|     Right     |   → (Right)  |


