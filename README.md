# RstRyzsr
A simple rasterizer in ~400 lines of code

## Required libraries
* LibC++17
* SDL2
* GLM

## Compiling
<code>
clang++ -std=c++17 -I/usr/include/SDL2 -D_REENTRANT -pthread -lSDL2 -O2 -Wall -Wextra rasterizer.cpp
</code>

## Usage
Esc = Toggle pause

WASD = Forward, Backward, Left, and Right

SPACE LCTRL = Up and down

J K = Increase / decrease framerate (default: 60)

G H = Increase / decrease the rotation of the object

F = Toggle rotation