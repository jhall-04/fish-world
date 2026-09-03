#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <random>
#include <iostream>
#include <algorithm>

constexpr int num_boids = 500;


constexpr int zone_of_repulsion = 1;
constexpr int zone_of_orientation = zone_of_repulsion + 4;
constexpr int zone_of_attraction = zone_of_orientation + 5;
constexpr float wall_avoidance_distance = 50.0f;

constexpr float BOID_LENGTH = zone_of_repulsion; // Length of the boid for drawing
constexpr float BOID_RADIUS = BOID_LENGTH * 0.2f; // Radius of the boid for drawing

constexpr float speed = 3.0f; // Speed of the boid
constexpr float blind_angle = 0.79f; // Blind angle in radians (e.g., 0.5 rad ~ 28.6 degrees)
constexpr float dt = 0.1f;
constexpr float max_turn_angle = 0.7f * dt; // Maximum turn angle in radians per update

static thread_local std::random_device rd;
constexpr float noise_sigma = 0.05f; // radians; runtime float later if you want a slider
static thread_local std::normal_distribution<float> noise_dist{ 0.0f, 1.0f };
static thread_local std::mt19937 gen(rd());
static thread_local std::uniform_real_distribution<float> dis_pos(-15.0f, 15.0f);
static thread_local std::uniform_real_distribution<float> dis_vel(-1.0f, 1.0f);


struct Boid {
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    Boid(float x, float y, float z, float vx, float vy, float vz) : x(x), y(y), z(z), vx(vx), vy(vy), vz(vz) {}
};

struct Flock {
    std::vector<Boid> boids;
    Flock() {
        boids.reserve(num_boids);
    }
};

void drawBoid(const Boid &boid) {
    Vector3 pos = { boid.x, boid.y, boid.z };
    Vector3 vel = { boid.vx, boid.vy, boid.vz };

    float speed = Vector3Length(vel);
    Vector3 dir = (speed > 1e-5f) ? Vector3Scale(vel, 1.0f / speed)
                                  : Vector3{ 0.0f, 0.0f, 1.0f };

    Vector3 tail = Vector3Subtract(pos, Vector3Scale(dir, BOID_LENGTH * 0.5f));
    Vector3 nose = Vector3Add(pos, Vector3Scale(dir, BOID_LENGTH * 0.5f));
    
    Color color = RED;

    DrawCylinderEx(tail, nose, BOID_RADIUS, 0.0f, 8, color);
}

float angleBetweenVectors(Vector3 a, Vector3 b) {
    float dot = Vector3DotProduct(a, b);
    float lengths = Vector3Length(a) * Vector3Length(b);
    if (lengths > 1e-5f) {
        return acosf(std::clamp(dot / lengths, -1.0f, 1.0f));
    }
    return 0.0f;
}

Vector3 v1Tov2Rotation(Vector3 v1, Vector3 v2) {
    Vector3 current = Vector3Normalize(v1);
    Vector3 desired = Vector3Normalize(v2);
    float dot = std::clamp(Vector3DotProduct(current, desired), -1.0f, 1.0f);
    float angle = acosf(dot);
    if (angle <= max_turn_angle) {
        return Vector3Scale(desired, speed);
    }

    Vector3 perpendicular = Vector3Subtract(desired, Vector3Scale(current, dot));
    if (Vector3Length(perpendicular) <= 1e-5f) {
        Vector3 reference = (fabsf(current.x) < 0.9f)
            ? Vector3{ 1.0f, 0.0f, 0.0f }
            : Vector3{ 0.0f, 1.0f, 0.0f };
        perpendicular = Vector3CrossProduct(current, reference);
    }
    perpendicular = Vector3Normalize(perpendicular);

    Vector3 turned = Vector3Add(
        Vector3Scale(current, cosf(max_turn_angle)),
        Vector3Scale(perpendicular, sinf(max_turn_angle)));
    return Vector3Scale(turned, speed);
}

Vector3 perturbDirection(Vector3 dir, float sigma) {
    if (sigma <= 0.0f) return dir;
    Vector3 offset = {
        noise_dist(gen) * sigma,
        noise_dist(gen) * sigma,
        noise_dist(gen) * sigma
    };
    return Vector3Normalize(Vector3Add(dir, offset));
}

Vector3 updateBoidVelocity(const Boid &boid, const Flock &flock) {
    int repulsionCount = 0;
    int orientationCount = 0;
    int attractionCount = 0;
    Vector3 repulsion = { 0.0f, 0.0f, 0.0f };
    Vector3 orientation = { 0.0f, 0.0f, 0.0f };
    Vector3 attraction = { 0.0f, 0.0f, 0.0f };

    for (const auto &other : flock.boids) {
        if (&other == &boid) {
            continue;
        }

        Vector3 toOther = { other.x - boid.x, other.y - boid.y, other.z - boid.z };
        float angle = angleBetweenVectors({ boid.vx, boid.vy, boid.vz }, toOther);
        if (angle > 3.14159265f - blind_angle) {
            continue;
        }

        float dist = Vector3Length(toOther);
        if (dist < zone_of_repulsion) {
            Vector3 normRep = Vector3Normalize(Vector3Negate(toOther));
            repulsion.x += normRep.x;
            repulsion.y += normRep.y;
            repulsion.z += normRep.z;
            repulsionCount++;
        } else if (dist < zone_of_orientation) {
            Vector3 normVel = Vector3Normalize({ other.vx, other.vy, other.vz });
            orientation.x += normVel.x;
            orientation.y += normVel.y;
            orientation.z += normVel.z;
            orientationCount++;
        } else if (dist < zone_of_attraction) {
            Vector3 normAttr = Vector3Normalize(toOther);
            attraction.x += normAttr.x;
            attraction.y += normAttr.y;
            attraction.z += normAttr.z;
            attractionCount++;
        }
    }
    if (repulsionCount > 0) {
        return v1Tov2Rotation({ boid.vx, boid.vy, boid.vz }, repulsion);
    }
    if (orientationCount > 0) {
        Vector3 current = Vector3Normalize({ boid.vx, boid.vy, boid.vz });
        orientation.x += current.x;
        orientation.y += current.y;
        orientation.z += current.z;
        orientationCount++;
    }

    if (orientationCount > 0 and attractionCount == 0) {
        return v1Tov2Rotation({ boid.vx, boid.vy, boid.vz }, orientation);
    } else if (attractionCount > 0 and orientationCount == 0) {
        return v1Tov2Rotation({ boid.vx, boid.vy, boid.vz }, attraction);
    } else if (orientationCount > 0 and attractionCount > 0) {
        Vector3 combined = { (orientation.x + attraction.x) / 2, (orientation.y + attraction.y) / 2, (orientation.z + attraction.z) / 2 };
        return v1Tov2Rotation({ boid.vx, boid.vy, boid.vz }, combined);
    }
    return { boid.vx, boid.vy, boid.vz };
}

float Slider(Rectangle bounds, const char *label, float value, float minVal, float maxVal, bool *anySliderActive) {
    Vector2 mouse = GetMousePosition();
    bool hovered = CheckCollisionPointRec(mouse, bounds);

    if (hovered && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        float t = (mouse.x - bounds.x) / bounds.width;
        value = minVal + std::clamp(t, 0.0f, 1.0f) * (maxVal - minVal);
        *anySliderActive = true;
    }

    // Track
    DrawRectangleRec(bounds, Fade(WHITE, 0.15f));
    // Fill up to current value
    float fillW = bounds.width * (value - minVal) / (maxVal - minVal);
    DrawRectangle((int)bounds.x, (int)bounds.y, (int)fillW, (int)bounds.height, Fade(SKYBLUE, 0.6f));
    // Handle
    DrawRectangle((int)(bounds.x + fillW - 3), (int)bounds.y - 2, 6, (int)bounds.height + 4, WHITE);
    // Label
    DrawText(TextFormat("%s: %.1f", label, value), (int)bounds.x, (int)bounds.y - 18, 16, WHITE);

    return value;
}




int main() {
    InitWindow(800, 600, "3d");

    Flock flock;
    

    Vector3 polarization = { 0.0f, 0.0f, 0.0f };
    Vector3 momentum = { 0.0f, 0.0f, 0.0f };
    Vector3 flockCenter = { 0.0f, 0.0f, 0.0f };

    // Orbit variables
    float radius = 100 * 1.5f; // Distance from the center of the flock
    float alpha = 0.0f; // Horizontal angle
    float beta = 1.0f;  // Vertical angle
    
    // ADJUST THIS TO CHANGE SPEED (Lower = Slower)
    float turnSpeed = 0.002f; 

    int zone_of_repulsion = 1;
    int zone_of_orientation = zone_of_repulsion + 15;
    int zone_of_attraction = zone_of_orientation + 15;
    float wall_avoidance_distance = 50.0f;

    float BOID_LENGTH = zone_of_repulsion; // Length of the boid for drawing
    float BOID_RADIUS = BOID_LENGTH * 0.2f; // Radius of the boid for drawing
    
    for (int i = 0; i < num_boids; ++i) {
        Vector3 velocity = { dis_vel(gen), dis_vel(gen), dis_vel(gen) };
        velocity = Vector3Scale(Vector3Normalize(velocity), speed);
        flock.boids.push_back(Boid(dis_pos(gen), dis_pos(gen), dis_pos(gen), velocity.x, velocity.y, velocity.z));
    }

    Camera3D camera = {};
    camera.target   = { 0.0f, 0.0f, 0.0f };
    camera.up       = { 0.0f, 1.0f, 0.0f };
    camera.fovy     = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 mouseDelta = GetMouseDelta();
            alpha -= mouseDelta.x * turnSpeed;
            beta  += mouseDelta.y * turnSpeed;
            
            // Clamp vertical rotation to avoid flipping upside down
            if (beta > 1.5f) beta = 1.5f;
            if (beta < 0.1f) beta = 0.1f;
        }
        // Zoom with scroll wheel
        radius -= GetMouseWheelMove() * 10.0f;
        radius = Clamp(radius, 20.0f, 600.0f);
        camera.target = flockCenter;
        camera.position.x = camera.target.x + radius * cosf(beta) * sinf(alpha);
        camera.position.y = camera.target.y + radius * sinf(beta);
        camera.position.z = camera.target.z + radius * cosf(beta) * cosf(alpha);
        std::vector<Vector3> newVelocities;
        BeginDrawing();
            ClearBackground(DARKBLUE);
            
            BeginMode3D(camera);
                for (const auto &boid : flock.boids) {

                    drawBoid(boid);
                    
                }
            EndMode3D();
            DrawFPS(10, 10);
            DrawText(TextFormat("Polarization: (%.2f)", Vector3Length(polarization)), 10, 30, 20, WHITE);
            DrawText(TextFormat("Momentum: (%.2f)", Vector3Length(momentum)), 10, 60, 20, WHITE);    
            float sw = (float)GetScreenWidth();
            float sh = (float)GetScreenHeight();
            bool sliderActive = false;

            float d_ro = zone_of_orientation - zone_of_repulsion;
            float d_ra = zone_of_attraction - zone_of_orientation;

            d_ro = Slider({ sw - 220, sh - 90, 200, 12 }, "d_ro", d_ro, 0.0f, 15.0f, &sliderActive);
            d_ra = Slider({ sw - 220, sh - 40, 200, 12 }, "d_ra", d_ra, 0.0f, 20.0f, &sliderActive);

            zone_of_orientation = zone_of_repulsion + d_ro;
            zone_of_attraction  = zone_of_orientation + d_ra;
            EndDrawing();
        for (auto &boid : flock.boids) {
            newVelocities.push_back(perturbDirection(updateBoidVelocity(boid, flock), noise_sigma));
        }
        for (size_t i = 0; i < flock.boids.size(); ++i) {
            flock.boids[i].vx = newVelocities[i].x;
            flock.boids[i].vy = newVelocities[i].y;
            flock.boids[i].vz = newVelocities[i].z;
        }
        flockCenter = { 0.0f, 0.0f, 0.0f };
        for (auto &boid : flock.boids) {
            boid.x += boid.vx * dt;
            boid.y += boid.vy * dt;
            boid.z += boid.vz * dt;
            flockCenter.x += boid.x;
            flockCenter.y += boid.y;
            flockCenter.z += boid.z;
        }
        flockCenter.x /= flock.boids.size();
        flockCenter.y /= flock.boids.size();
        flockCenter.z /= flock.boids.size();
        polarization = { 0.0f, 0.0f, 0.0f };
        momentum = { 0.0f, 0.0f, 0.0f };
        for (auto &boid : flock.boids) {
            Vector3 ric = Vector3Subtract({ boid.x, boid.y, boid.z }, flockCenter);
            ric = Vector3Normalize(ric);
            Vector3 velocity = { boid.vx, boid.vy, boid.vz };
            velocity = Vector3Normalize(velocity);
            Vector3 cross = Vector3CrossProduct(ric, velocity);
            momentum = Vector3Add(momentum, cross);
            polarization = Vector3Add(polarization, Vector3Normalize({ boid.vx, boid.vy, boid.vz }));
        }
        momentum.x /= flock.boids.size();
        momentum.y /= flock.boids.size();
        momentum.z /= flock.boids.size();
        polarization.x /= flock.boids.size();
        polarization.y /= flock.boids.size();
        polarization.z /= flock.boids.size();
    }

    CloseWindow();
    return 0;
}