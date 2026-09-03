#include <vector>
#include "raylib.h"
#include <cmath>
#include <random>
#include <algorithm>

#if defined(PLATFORM_WEB)
    #include <emscripten/emscripten.h>
#endif

#define SCREEN_WIDTH 1000
#define SCREEN_HEIGHT 800


struct Boid {
    float x;
    float y;
    float vx;
    float vy;
    Boid(float x, float y, float vx, float vy) : x(x), y(y), vx(vx), vy(vy) {}
};

struct Flock {
    std::vector<Boid> boids;
};

// Everything the frame callback needs, since it can't rely on main()'s stack.
struct AppState {
    Flock flock;
    std::vector<Vector2> newVelocities;
    float interactionRadius;
    float attractionStrength;
    float repulsionStrength;
    float velocityAlignmentStrength;
    float maxSpeed;
    float minSpeed;
};

void drawBoid(const Boid &boid) {
    float angle = std::atan2(boid.vy, boid.vx);
    DrawTriangle(
        {boid.x + 10 * std::cos(angle), boid.y + 10 * std::sin(angle)},
        {boid.x + 6 * std::cos(angle - 2.5f), boid.y + 6 * std::sin(angle - 2.5f)},
        {boid.x + 6 * std::cos(angle + 2.5f), boid.y + 6 * std::sin(angle + 2.5f)},
        PINK
    );
}

float Slider(Rectangle bounds, const char *label, float value, float minVal, float maxVal, bool *anySliderActive) {
    Vector2 mouse = GetMousePosition();
    bool hovered = CheckCollisionPointRec(mouse, bounds);

    if (hovered && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        float t = (mouse.x - bounds.x) / bounds.width;
        value = minVal + std::clamp(t, 0.0f, 1.0f) * (maxVal - minVal);
        *anySliderActive = true;
    }

    DrawRectangleRec(bounds, Fade(WHITE, 0.15f));
    float fillW = bounds.width * (value - minVal) / (maxVal - minVal);
    DrawRectangle((int)bounds.x, (int)bounds.y, (int)fillW, (int)bounds.height, Fade(SKYBLUE, 0.6f));
    DrawRectangle((int)(bounds.x + fillW - 3), (int)bounds.y - 2, 6, (int)bounds.height + 4, WHITE);
    DrawText(TextFormat("%s: %.1f", label, value), (int)bounds.x, (int)bounds.y - 18, 16, WHITE);

    return value;
}

Vector2 updateBoidVelocity(const Boid &boid, const Flock &flock, float interactionRadius, float attractionStrength, float repulsionStrength, float velocityAlignmentStrength, float maxSpeed, float minSpeed) {
    float cx = 0.0f;
    float cy = 0.0f;
    float count = 0.0f;
    float sumVx = 0.0f;
    float sumVy = 0.0f;
    float avgVx = 0.0f;
    float avgVy = 0.0f;
    float vx = boid.vx;
    float vy = boid.vy;
    for (const auto &other : flock.boids) {
        if (&other != &boid) {
            float dx = other.x - boid.x;
            float dy = other.y - boid.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < interactionRadius) {
                // Center Attraction
                cx += other.x;
                cy += other.y;
                // Repulsion
                vx -= repulsionStrength * dx / (dist * dist);
                vy -= repulsionStrength * dy / (dist * dist);
                // Alignment
                sumVx += other.vx;
                sumVy += other.vy;
                count += 1.0f;
            }
        }
    }
    if (count > 0) {
        cx /= count;
        cy /= count;
        vx += attractionStrength * (cx - boid.x);
        vy += attractionStrength * (cy - boid.y);

        avgVx = sumVx / count;
        avgVy = sumVy / count;
        vx += velocityAlignmentStrength * (avgVx - vx);
        vy += velocityAlignmentStrength * (avgVy - vy);
        if (vx * vx + vy * vy > maxSpeed * maxSpeed) {
            float speed = std::sqrt(vx * vx + vy * vy);
            vx = (vx / speed) * maxSpeed;
            vy = (vy / speed) * maxSpeed;
        } else if (vx * vx + vy * vy < minSpeed * minSpeed) {
            float speed = std::sqrt(vx * vx + vy * vy);
            vx = (vx / speed) * minSpeed;
            vy = (vy / speed) * minSpeed;
        }
    }
    // Wall avoidance ramps up smoothly and remains bounded at the screen edge.
    constexpr float wallMargin = 20.0f;
    constexpr float wallTurnStrength = 0.5f;
    if (boid.x > SCREEN_WIDTH - wallMargin) {
        vx -= wallTurnStrength * (boid.x - (SCREEN_WIDTH - wallMargin)) / wallMargin;
    } else if (boid.x < wallMargin) {
        vx += wallTurnStrength * (wallMargin - boid.x) / wallMargin;
    }
    if (boid.y > SCREEN_HEIGHT - wallMargin) {
        vy -= wallTurnStrength * (boid.y - (SCREEN_HEIGHT - wallMargin)) / wallMargin;
    } else if (boid.y < wallMargin) {
        vy += wallTurnStrength * (wallMargin - boid.y) / wallMargin;
    }
    return {vx, vy};
}

// One frame: draw, then integrate. Called by the browser via requestAnimationFrame
// on web, and by a plain while loop on desktop.
void UpdateDrawFrame(void *arg) {
    AppState *state = (AppState *)arg;
    Flock &flock = state->flock;

    float sw = (float)GetScreenWidth();
    float sh = (float)GetScreenHeight();
    bool sliderActive = false;

    BeginDrawing();
    ClearBackground(DARKBLUE);
    for (auto &boid : flock.boids) {
        drawBoid(boid);
    }
    state->repulsionStrength = Slider({ sw - 220, sh - 110, 200, 12 }, "Repulsion Range", state->repulsionStrength, 0.0f, 2.0f, &sliderActive);
    state->attractionStrength = Slider({ sw - 220, sh - 40, 200, 12 }, "Attraction Range", state->attractionStrength, 0.0f, 0.01f, &sliderActive);
    state->velocityAlignmentStrength = Slider({ sw - 220, sh - 75, 200, 12 }, "Velocity Alignment", state->velocityAlignmentStrength, 0.0f, 0.1f, &sliderActive);
    EndDrawing();

    state->newVelocities.clear();
    for (auto &boid : flock.boids) {
        state->newVelocities.push_back(updateBoidVelocity(boid, flock, state->interactionRadius, state->attractionStrength, state->repulsionStrength, state->velocityAlignmentStrength, state->maxSpeed, state->minSpeed));
    }
    for (size_t i = 0; i < flock.boids.size(); ++i) {
        flock.boids[i].vx = state->newVelocities[i].x;
        flock.boids[i].vy = state->newVelocities[i].y;
    }
    for (auto &boid : flock.boids) {
        boid.x += boid.vx;
        boid.y += boid.vy;
        if (boid.x > SCREEN_WIDTH) boid.x = 0;
        if (boid.y > SCREEN_HEIGHT) boid.y = 0;
        if (boid.x < 0) boid.x = SCREEN_WIDTH;
        if (boid.y < 0) boid.y = SCREEN_HEIGHT;
    }
}

// Static so its lifetime outlives main() when the browser owns the loop.
static AppState state;

#if defined(PLATFORM_WEB)
// Called from JS when the React component unmounts.
extern "C" EMSCRIPTEN_KEEPALIVE void StopSim(void) {
    emscripten_cancel_main_loop();
    CloseWindow();
}
#endif

int main() {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "boids");

    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dis_angle(0.0f, 2.0f * PI);
    std::uniform_int_distribution<int> dis_width(0, SCREEN_WIDTH);
    std::uniform_int_distribution<int> dis_height(0, SCREEN_HEIGHT);

    float velocity = 3.0f;
    state.interactionRadius = 30.0f;
    state.attractionStrength = 0.005f;
    state.repulsionStrength = 0.9f;
    state.velocityAlignmentStrength = 0.05f;
    state.maxSpeed = 6.0f;
    state.minSpeed = 1.0f;

    for (int i = 0; i < 500; ++i) {
        float a = dis_angle(gen);
        state.flock.boids.push_back(Boid(dis_width(gen), dis_height(gen), velocity * std::cos(a), velocity * std::sin(a)));
    }
    state.newVelocities.reserve(state.flock.boids.size());

#if defined(PLATFORM_WEB)
    emscripten_set_main_loop_arg(UpdateDrawFrame, &state, 0, 1);
#else
    SetTargetFPS(60);
    while (!WindowShouldClose()) {
        UpdateDrawFrame(&state);
    }
#endif

    CloseWindow();
    return 0;
}