//gameboy.cpp
#include "gameboy.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cstring> //for memcpy
#include "SDL.h"

//constructor for gameboy class. initializes components and sets default values
GameBoy::GameBoy() :
    cpu(memory),
    ppu(memory),
    timer(memory),
    interruptHandler(memory),
    joypad(memory),
    isRunning(false),
    cyclesThisFrame(0),
    frameBuffer{}
{}

//destructor for gameboy class. currently empty
GameBoy::~GameBoy() {
}

//loads the boot rom from a file
void GameBoy::loadBootROM(const std::string& filename) {
    std::ifstream bootRomFile(filename, std::ios::binary);
    if (!bootRomFile.is_open()) {
        throw std::runtime_error("Failed to open Boot ROM file: " + filename);
    }

    std::vector<uint8_t> bootRomData((std::istreambuf_iterator<char>(bootRomFile)),
                                     std::istreambuf_iterator<char>());
    bootRomFile.close();

    memory.loadBootROM(bootRomData);
}

//loads the game rom from a file
void GameBoy::loadGameROM(const std::string& filename) {
    memory.cartridge = std::make_unique<Cartridge>(filename);
    memory.loadROM(memory.cartridge->getROMData());
}

//main run loop for the gameboy emulator
void GameBoy::run() {
    isRunning = true;

    //initialize sdl2 for rendering
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "Failed to initialize SDL2: " << SDL_GetError() << std::endl;
        return;
    }

    //create sdl window
    SDL_Window* window = SDL_CreateWindow("GameBoy Emulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        160 * 4, 144 * 4, 0); //scale x4 for visibility

    if (!window) {
        std::cerr << "Failed to create SDL2 window: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return;
    }

    //create sdl renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Failed to create SDL2 renderer: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return;
    }

    //create sdl texture
    SDL_Texture* texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        160, 144);

    if (!texture) {
        std::cerr << "Failed to create SDL2 texture: " << SDL_GetError() << std::endl;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return;
    }

    //main game loop
    while (isRunning) {
        runFrame();

        //update the sdl texture with frame buffer data
        void* pixels;
        int pitch;
        if (SDL_LockTexture(texture, NULL, &pixels, &pitch) != 0) {
            std::cerr << "Failed to lock SDL2 texture: " << SDL_GetError() << std::endl;
            break;
        }

        uint32_t* dst = static_cast<uint32_t*>(pixels);
        for (int y = 0; y < 144; y++) {
            for (int x = 0; x < 160; x++) {
                dst[y * 160 + x] = frameBuffer[y][x];
            }
        }

        SDL_UnlockTexture(texture);

        //render the texture to the window
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        //handle sdl events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                isRunning = false;
            }
            //handle keyboard input events
            if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) {
                bool isPressed = (event.type == SDL_KEYDOWN);
                int button = -1;
                switch (event.key.keysym.sym) {
                    case SDLK_z: button = Joypad::A; break;
                    case SDLK_x: button = Joypad::B; break;
                    case SDLK_a: button = Joypad::SELECT; break;
                    case SDLK_s: button = Joypad::START; break;
                    case SDLK_UP: button = Joypad::UP; break;
                    case SDLK_DOWN: button = Joypad::DOWN; break;
                    case SDLK_LEFT: button = Joypad::LEFT; break;
                    case SDLK_RIGHT: button = Joypad::RIGHT; break;
                    default: break;
                }
                if (button != -1) {
                    handleInput(button, isPressed);
                }
            }
        }

        //limit frame rate to approximately 60 fps
        SDL_Delay(16);
    }

    //clean up sdl resources
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

//runs a single frame of the gameboy emulation
void GameBoy::runFrame() {
    cyclesThisFrame = 0;
    //set cycles per frame (70224 is approximate for 60 fps)
    const int cyclesPerFrame = 70224;

    while (cyclesThisFrame < cyclesPerFrame) {
        step();
    }

    //after frame is complete, copy the ppu's frame buffer
    frameBuffer = ppu.getFrameBuffer();
}

//performs a single step of the gameboy emulation
void GameBoy::step() {
    int cycles = cpu.step();

    ppu.step(cycles);
    timer.step(cycles);
    interruptHandler.checkInterrupts();
    joypad.update();

    cyclesThisFrame += cycles;
}

//handles input events for the gameboy joypad
void GameBoy::handleInput(int key, bool isPressed) {
    if (isPressed) {
        joypad.pressButton(key);
    }
    else {
        joypad.releaseButton(key);
    }
}

//returns the current frame buffer
const std::array<std::array<uint32_t, 160>, 144>& GameBoy::getFrameBuffer() const {
    return frameBuffer;
}