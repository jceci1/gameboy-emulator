cmake_minimum_required(VERSION 3.10)
project(GameboyEmulator)

set(CMAKE_CXX_STANDARD 17)

# Set SDL2 paths
set(SDL2_PATH "C:/SDL2/SDL2-2.30.7")
set(SDL2_INCLUDE_DIR "${SDL2_PATH}/include")
set(SDL2_LIBRARY "${SDL2_PATH}/lib/x64/SDL2.lib")
set(SDL2_MAIN_LIBRARY "${SDL2_PATH}/lib/x64/SDL2main.lib")

# Include SDL2 headers
include_directories(${SDL2_INCLUDE_DIR})

# Add source files
file(GLOB SOURCES "src/*.cpp")

add_executable(gameboy_emulator ${SOURCES})

# Link SDL2 libraries
target_link_libraries(gameboy_emulator ${SDL2_LIBRARY} ${SDL2_MAIN_LIBRARY})

# Additional libraries needed
target_link_libraries(gameboy_emulator winmm version imm32 setupapi)

# Copy SDL2.dll to the output directory
add_custom_command(TARGET gameboy_emulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${SDL2_PATH}/lib/x64/SDL2.dll"
    $<TARGET_FILE_DIR:gameboy_emulator>
)