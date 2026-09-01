#include <iostream>
#include <vector>
#include "raylib.h"
#include <cmath>
#include <random>

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600


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

void drawBoid(const Boid &boid) {
    float angle = std::atan2(boid.vy, boid.vx);
    DrawTriangle(
        {boid.x + 8 * std::cos(angle), boid.y + 8 * std::sin(angle)},
        {boid.x + 4 * std::cos(angle - 2.5f), boid.y + 4 * std::sin(angle - 2.5f)},
        {boid.x + 4 * std::cos(angle + 2.5f), boid.y + 4 * std::sin(angle + 2.5f)},
        PINK
    );
}

Vector2 updateBoidVelocity(const Boid &boid, const Flock &flock, float interactionRadius, float attractionStrength, float repulsionStrength, float velocityAlignmentStrength, float maxSpeed) {
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
    }
    return {vx, vy};
}


int main() {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "boids");
    SetTargetFPS(60);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis_angle(0.0f, 2.0f * PI);
    std::uniform_int_distribution<int> dis_width(0, SCREEN_WIDTH);
    std::uniform_int_distribution<int> dis_height(0, SCREEN_HEIGHT);

    float velocity = 3.0f;
    float interactionRadius = 50;
    float attractionStrength = 0.01f;
    float repulsionStrength = 0.6f;
    float velocityAlignmentStrength = 0.05f;
    float maxSpeed = 5.0f;


    Flock flock;
    for (int i = 0; i < 50; ++i) {
        float a = dis_angle(gen);
        flock.boids.push_back(Boid(dis_width(gen), dis_height(gen), velocity * std::cos(a), velocity * std::sin(a)));
    }

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        for (auto &boid : flock.boids) {
            drawBoid(boid);
        }
        EndDrawing();
        std::vector<Vector2> newVelocities;
        for (auto &boid : flock.boids) {
            newVelocities.push_back(updateBoidVelocity(boid, flock, interactionRadius, attractionStrength, repulsionStrength, velocityAlignmentStrength, maxSpeed));
        }
        for (size_t i = 0; i < flock.boids.size(); ++i) {
            flock.boids[i].vx = newVelocities[i].x;
            flock.boids[i].vy = newVelocities[i].y;
        }
        for (auto &boid : flock.boids) {
            boid.x += boid.vx; // Move the circle to the right each frame
            boid.y += boid.vy; // Move the circle down each frame
            if (boid.x > SCREEN_WIDTH) boid.x = 0;
            if (boid.y > SCREEN_HEIGHT) boid.y = 0;
            if (boid.x < 0) boid.x = SCREEN_WIDTH;
            if (boid.y < 0) boid.y = SCREEN_HEIGHT;
        }
    }

    CloseWindow();
    return 0;
}