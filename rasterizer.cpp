#include <SDL2/SDL.h>
#include <glm/glm.hpp>
using namespace glm;

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <optional>
#include <vector>

#include "model.hpp"

// Color of the wireframe edges
// Mask: 0xAABBGGRR
#define WIREFRAME_COLOR 0xFFFFFFFF
struct Object
{
    vec3                             pos;
    vec3                             rot;
    std::vector<std::array<vec3, 3>> faces;
};

SDL_Window*   window;
SDL_Renderer* renderer;
SDL_Texture*  texture;
SDL_Surface*  surface;

// Some of the viewport settings
const int WIDTH  = 1280;
const int HEIGHT = 720;
float     FOV    = 70;
float     FPS    = 60;
Uint32              framebuffer[HEIGHT][WIDTH];
std::vector<Object> objects;
struct
{
    vec3  pos;
    vec3  rot;
    float clip;
    float width;
    float height;
    struct
    {
        float x;
        float y;
    } cfov;
} camera;

/*
    Rotate vertex around a defined origin (a camera for example)
    The function returns the new position of the vertex
*/
vec3 rotateAroundOrigin(vec3 pos, vec3 rot, vec3 origin)
{
    vec3 newpos;

    newpos.x = pos.x;
    newpos.y = sin(rot.x) * (pos.z - origin.z) +
               cos(rot.x) * (pos.y - origin.y) + origin.y;
    newpos.z = cos(rot.x) * (pos.z - origin.z) -
               sin(rot.x) * (pos.y - origin.y) + origin.z;
    pos = newpos;

    newpos.x = cos(rot.y) * (pos.x - origin.x) -
               sin(rot.y) * (pos.z - origin.z) + origin.x;
    newpos.y = pos.y;
    newpos.z = sin(rot.y) * (pos.x - origin.x) +
               cos(rot.y) * (pos.z - origin.z) + origin.z;
    pos = newpos;

    newpos.x = cos(rot.z) * (pos.x - origin.x) -
               sin(rot.z) * (pos.y - origin.y) + origin.x;
    newpos.y = sin(rot.z) * (pos.x - origin.x) +
               cos(rot.z) * (pos.y - origin.y) + origin.y;
    newpos.z = pos.z;

    return newpos;
}

/*
    Calculate the vec2 position of a vertex on the raster image.
    This function assumes that the camera is currently facing +X according to
    the right-hand rule. Vertices will have to be rotated beforehand.

    If the vertex is outside the image boundaries the function will not return
    anything.
*/
std::optional<vec2> projectVertex(vec3 vertex)
{
    std::optional<vec2> p;

    // The vertex will definitely not be visible if it is behind the camera
    if (vertex.x - camera.pos.x < camera.clip)
        return p;

    // Calculate the angle between the camera and the vertex in both dimentions
    // On the X axis for example 0 = left, π = right, and 1/2π = forward
    double rotx =
      vertex.y != camera.pos.y
        ? atan((vertex.x - camera.pos.x) / (vertex.y - camera.pos.y))
        : M_PI_2;
    double roty =
      vertex.z != camera.pos.z
        ? atan((vertex.x - camera.pos.x) / (vertex.z - camera.pos.z))
        : M_PI_2;

    // An easy way to check if the vertex is visible - check if the right
    // triangle lies outside the image
    if (M_PI - abs(rotx) * 2 > camera.cfov.x ||
        M_PI - abs(roty) * 2 > camera.cfov.y)
        return p;

    // Use the previously calculated angle to get the vec2 position on the image
    // a / tan(θ) = b
    float x = rotx != M_PI_2 ? camera.clip / tan(rotx) : 0;
    float y = roty != M_PI_2 ? camera.clip / tan(roty) : 0;

    p = vec2(x, y);

    return p;
}
/*
    Draw a line between two points on the raster image
*/
void projectEdge(vec2 point0, vec2 point1)
{
    // First, get the actual pixel values of the points
    uint32_t point0PixelX = static_cast<uint32_t>(
      round((abs((point0.x - (camera.width / 2))) / camera.width) * WIDTH));
    uint32_t point0PixelY = static_cast<uint32_t>(
      round((abs((point0.y - (camera.height / 2))) / camera.height) * HEIGHT));
    uint32_t point1PixelX = static_cast<uint32_t>(
      round((abs((point1.x - (camera.width / 2))) / camera.width) * WIDTH));
    uint32_t point1PixelY = static_cast<uint32_t>(
      round((abs((point1.y - (camera.height / 2))) / camera.height) * HEIGHT));

    // Get the angle between the points
    double angle = atan((point1.y - point0.y) / (point1.x - point0.x));

// Linear function defining the position of a point on the line between point0
// and point1
#define F(x, c) (change * (x) + c)

    // Draw on the X axis if the line is not steeper than 45 degrees (either
    // way)
    if (abs(angle) <= M_PI_4) {
        // Calculate the derivative (used in F(x, c))
        double change =
          (double)((point1.y - point0.y) / (double)(point0.x - point1.x));
        // Iterate for every pixel between the points...
        for (uint32_t p = point0PixelX;;) {
            // Calculate the relative position from point0
            double rp =
              ((double)p - (double)point0PixelX) * (camera.width / WIDTH);
            // Calculate the y coordinate based on this
            double y = F(rp, point0.y);

            // Get the actual pixel index for the specific position
            uint32_t intersectionY = static_cast<uint32_t>(
              round(abs((y - (camera.height / 2)) / camera.height) * HEIGHT));

            // Set the pixel at that position to a color of choice, the if
            // statement is there as a safeguard if invalid argument values were
            // to be passed and thus avoiding a segmentation fault
            if (intersectionY < HEIGHT && p < WIDTH)
                framebuffer[intersectionY][p] = WIREFRAME_COLOR;

            // All pixels have been evaluated
            if (p == point1PixelX)
                break;

            // Change the current pixel based on the position of the points
            if (point0PixelX > point1PixelX)
                --p;
            else
                ++p;
        }
    }
    // Else draw on the Y axis
    else {
        double change =
          (double)((point0.x - point1.x) / (double)(point1.y - point0.y));
        for (uint32_t p = point0PixelY;;) {
            double rp =
              ((double)p - (double)point0PixelY) * (camera.height / HEIGHT);
            double x = F(rp, point0.x);

            uint32_t intersectionX = static_cast<uint32_t>(
              round(abs((x - (camera.width / 2)) / camera.width) * WIDTH));

            if (intersectionX < WIDTH && p < HEIGHT)
                framebuffer[p][intersectionX] = WIREFRAME_COLOR;

            if (p == point1PixelY)
                break;
            if (point0PixelY > point1PixelY)
                --p;
            else
                ++p;
        }
    }
}
/*
    Renders the edges of a face
*/
void renderFace(std::array<vec3, 3> vertices)
{
    /*
        In order to simplify the maths for the actual rasterization process, the
        vertices are rotated around the camera instead of rotating the camera
    */
    vertices[0] = rotateAroundOrigin(vertices[0], camera.rot * vec3(-1, -1, -1),
                                     camera.pos);
    vertices[1] = rotateAroundOrigin(vertices[1], camera.rot * vec3(-1, -1, -1),
                                     camera.pos);
    vertices[2] = rotateAroundOrigin(vertices[2], camera.rot * vec3(-1, -1, -1),
                                     camera.pos);

    // Get the position of the vertices on the raster image
    std::array<std::optional<vec2>, 3> projection;
    projection[0] = projectVertex(vertices[0]);
    projection[1] = projectVertex(vertices[1]);
    projection[2] = projectVertex(vertices[2]);

    // Draw the edges. Both vertices needs to be in the image for it to be drawn
    if (projection[0].has_value()) {
        if (projection[1].has_value())
            projectEdge(projection[0].value(), projection[1].value());
        if (projection[2].has_value())
            projectEdge(projection[0].value(), projection[2].value());
    }
    if (projection[1].has_value() && projection[2].has_value())
        projectEdge(projection[1].value(), projection[2].value());
}
/*
    Renders an object
*/
void renderObject(const Object& obj)
{
    for (auto& face : obj.faces) {
        // The values of the vertices are relative to the object's position.
        // Here we apply the object's global postion the vertices to the global
        // position of them
        std::array<vec3, 3> globalVertices = face;
        globalVertices[0] += obj.pos;
        globalVertices[1] += obj.pos;
        globalVertices[2] += obj.pos;

        // Apply the rotation of the object to the vertices
        globalVertices[0] =
          rotateAroundOrigin(globalVertices[0], obj.rot, obj.pos);
        globalVertices[1] =
          rotateAroundOrigin(globalVertices[1], obj.rot, obj.pos);
        globalVertices[2] =
          rotateAroundOrigin(globalVertices[2], obj.rot, obj.pos);

        renderFace(globalVertices);
    }
}
/*
    Draws a single frame

    Result is in framebuffer, surface, and texture
*/
void getFrame()
{
    memset(framebuffer, 0, sizeof(framebuffer));

    camera.cfov.x = FOV * (M_PI / 180);
    camera.cfov.y = camera.cfov.x * ((float)HEIGHT / (float)WIDTH);

    camera.width  = tan(camera.cfov.x / 2) * camera.clip * 2;
    camera.height = tan(camera.cfov.y / 2) * camera.clip * 2;

    for (auto& object : objects)
        renderObject(object);

    SDL_memcpy(surface->pixels, framebuffer, sizeof(framebuffer));
    texture = SDL_CreateTextureFromSurface(renderer, surface);
}

Uint32 getTimeLeft(Uint32 next)
{
    Uint32 now = SDL_GetTicks();

    return next > now ? next - now : 0;
}
int main()
{
    // Boilerplate to initalize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
        return EXIT_FAILURE;

    SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0");

    window = SDL_CreateWindow("Rasterizer", SDL_WINDOWPOS_CENTERED,
                              SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, 0);
    if (!window) {
        SDL_Quit();
        return EXIT_FAILURE;
    }
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        SDL_DestroyWindow(window);
        SDL_Quit();
        return EXIT_FAILURE;
    }
    surface = SDL_CreateRGBSurface(0, WIDTH, HEIGHT, 32, 0x000000FF, 0x0000FF00,
                                   0x00FF0000, 0xFF000000);
    if (!surface) {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return EXIT_FAILURE;
    }

    // Configure the scene
    // Unit for rotations = radians
    camera.pos         = vec3(-5, 0, 2);
    camera.rot         = vec3(0, -0.4, 0);
    camera.clip = 0.01;

    // Configure render objects
    objects.push_back(Object());
    objects[0].pos = vec3(0, 0, -0.5);
    objects[0].rot = vec3(M_PI_2, 0, M_PI_2);
    objects[0].faces.resize(MODEL_FACES);

    for (int i = 0; i < MODEL_FACES; ++i) {
        objects[0].faces[i][0].x = model[9 * i + 0 * 3 + 0];
        objects[0].faces[i][0].y = model[9 * i + 0 * 3 + 1];
        objects[0].faces[i][0].z = model[9 * i + 0 * 3 + 2];

        objects[0].faces[i][1].x = model[9 * i + 1 * 3 + 0];
        objects[0].faces[i][1].y = model[9 * i + 1 * 3 + 1];
        objects[0].faces[i][1].z = model[9 * i + 1 * 3 + 2];

        objects[0].faces[i][2].x = model[9 * i + 2 * 3 + 0];
        objects[0].faces[i][2].y = model[9 * i + 2 * 3 + 1];
        objects[0].faces[i][2].z = model[9 * i + 2 * 3 + 2];
    }

    SDL_Event e;
    bool      active   = true;
    bool      render   = true;
    bool      rotation = true;
    double    spr      = 8;
    Uint32    timeNext = 0;
    while (active) {
        double timeStep = 1000 / FPS;

        if (render) {
            const Uint8* keystate = SDL_GetKeyboardState(NULL);

            if (keystate[SDL_SCANCODE_W])
                camera.pos.x += (1 / FPS);
            if (keystate[SDL_SCANCODE_S])
                camera.pos.x -= (1 / FPS);
            if (keystate[SDL_SCANCODE_A])
                camera.pos.y += (1 / FPS);
            if (keystate[SDL_SCANCODE_D])
                camera.pos.y -= (1 / FPS);
            if (keystate[SDL_SCANCODE_SPACE])
                camera.pos.z += (1 / FPS);
            if (keystate[SDL_SCANCODE_LCTRL])
                camera.pos.z -= (1 / FPS);

            if (keystate[SDL_SCANCODE_UP])
                camera.rot.y = fmod(camera.rot.y + (M_PI_4 / FPS), 2 * M_PI);
            if (keystate[SDL_SCANCODE_DOWN])
                camera.rot.y = fmod(camera.rot.y - (M_PI_4 / FPS), 2 * M_PI);
        }
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
                case SDL_QUIT: active = false; break;
                case SDL_KEYDOWN:
                    if (e.key.keysym.sym == SDLK_ESCAPE)
                        render = !render;
                    else if (e.key.keysym.sym == SDLK_j) {
                        FPS += 5;
                        printf("FPS: %f\n", FPS);
                    } else if (e.key.keysym.sym == SDLK_k && FPS > 5) {
                        FPS -= 5;
                        printf("FPS: %f\n", FPS);
                    } else if (e.key.keysym.sym == SDLK_g) {
                        spr += 0.1;
                        printf("Seconds per rotation %f\n", spr);
                    } else if (e.key.keysym.sym == SDLK_h) {
                        spr -= 0.1;
                        printf("Seconds per rotation %f\n", spr);
                    } else if (e.key.keysym.sym == SDLK_f) {
                        rotation         = !rotation;
                    }
                    break;
            }
        }
        if (!render) {
            SDL_Delay(getTimeLeft(timeNext));
            timeNext += timeStep;
            continue;
        }

        // Rotate the object so that it does one revolution every 8 seconds
        if (rotation)
            objects[0].rot.z =
              fmod(objects[0].rot.z + ((2 * M_PI / spr) / FPS), 2 * M_PI);

        getFrame();
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
        SDL_DestroyTexture(texture);
        SDL_Delay(getTimeLeft(timeNext));
        timeNext += timeStep;
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return EXIT_SUCCESS;
}