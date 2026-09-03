#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

constexpr int num_boids = 500;

// Zones. rr is fixed (paper convention); ro and ra are runtime so the
// sliders (and later the sweep driver) can change them. Everything reads
// these globals -- there must be NO local copies in main.
constexpr float zone_of_repulsion = 1.0f;
float zone_of_orientation = zone_of_repulsion + 2.25f;
float zone_of_attraction  = zone_of_orientation + 10.0f;   // d_ra = 10: ra must exceed group radius

constexpr float BOID_LENGTH = zone_of_repulsion;      // 1 body length
constexpr float BOID_RADIUS = BOID_LENGTH * 0.2f;

constexpr float speed = 3.0f;          // body lengths per time unit
constexpr float blind_angle = 0.79f;   // half-angle of blind cone (rad) -> 270 deg FOV
constexpr float dt = 0.1f;
constexpr float max_turn_angle = 0.7f * dt;   // theta * dt
float noise_sigma = 0.05f;             // radians per update; runtime for later tuning

static thread_local std::mt19937 gen{ std::random_device{}() };
static thread_local std::normal_distribution<float> noise_dist{ 0.0f, 1.0f };

struct Boid {
    float x, y, z;
    float vx, vy, vz;
    Boid(float x, float y, float z, float vx, float vy, float vz)
        : x(x), y(y), z(z), vx(vx), vy(vy), vz(vz) {}
};

struct Flock {
    std::vector<Boid> boids;
    Flock() { boids.reserve(num_boids); }
};

void drawBoid(const Boid &boid) {
    Vector3 pos = { boid.x, boid.y, boid.z };
    Vector3 vel = { boid.vx, boid.vy, boid.vz };

    float len = Vector3Length(vel);
    Vector3 dir = (len > 1e-5f) ? Vector3Scale(vel, 1.0f / len)
                                : Vector3{ 0.0f, 0.0f, 1.0f };

    Vector3 tail = Vector3Subtract(pos, Vector3Scale(dir, BOID_LENGTH * 0.5f));
    Vector3 nose = Vector3Add(pos, Vector3Scale(dir, BOID_LENGTH * 0.5f));

    DrawCylinderEx(tail, nose, BOID_RADIUS, 0.0f, 8, RED);
}

float angleBetweenVectors(Vector3 a, Vector3 b) {
    float dot = Vector3DotProduct(a, b);
    float lengths = Vector3Length(a) * Vector3Length(b);
    if (lengths > 1e-5f) {
        return acosf(std::clamp(dot / lengths, -1.0f, 1.0f));
    }
    return 0.0f;
}

// Perturb a unit direction by ~Gaussian angular noise (std dev sigma rad).
Vector3 perturbDirection(Vector3 dir, float sigma) {
    if (sigma <= 0.0f) return dir;
    Vector3 offset = {
        noise_dist(gen) * sigma,
        noise_dist(gen) * sigma,
        noise_dist(gen) * sigma
    };
    return Vector3Normalize(Vector3Add(dir, offset));
}

// Turn from v1 toward v2, limited to max_turn_angle, with noise applied to
// the desired direction BEFORE the turn limit (so noise cannot exceed the
// physical turning rate). Always returns a vector of magnitude `speed`.
Vector3 v1Tov2Rotation(Vector3 v1, Vector3 v2) {
    Vector3 current = Vector3Normalize(v1);
    Vector3 desired = Vector3Normalize(v2);
    desired = perturbDirection(desired, noise_sigma);

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
            repulsion = Vector3Add(repulsion, normRep);
            repulsionCount++;
        } else if (dist < zone_of_orientation) {
            Vector3 normVel = Vector3Normalize({ other.vx, other.vy, other.vz });
            orientation = Vector3Add(orientation, normVel);
            orientationCount++;
        } else if (dist < zone_of_attraction) {
            Vector3 normAttr = Vector3Normalize(toOther);
            attraction = Vector3Add(attraction, normAttr);
            attractionCount++;
        }
    }

    Vector3 self = { boid.vx, boid.vy, boid.vz };

    if (repulsionCount > 0) {
        return v1Tov2Rotation(self, repulsion);
    }
    if (orientationCount > 0) {
        // Paper: orientation response includes own direction
        orientation = Vector3Add(orientation, Vector3Normalize(self));
        orientationCount++;
    }

    if (orientationCount > 0 && attractionCount == 0) {
        return v1Tov2Rotation(self, orientation);
    } else if (attractionCount > 0 && orientationCount == 0) {
        return v1Tov2Rotation(self, attraction);
    } else if (orientationCount > 0 && attractionCount > 0) {
        Vector3 combined = Vector3Scale(Vector3Add(orientation, attraction), 0.5f);
        return v1Tov2Rotation(self, combined);
    }

    // No neighbours: desired = current. Still goes through the rotation so
    // the boid receives noise and its speed stays pinned to `speed`.
    return v1Tov2Rotation(self, self);
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

int main() {
    InitWindow(800, 600, "3d");

    Flock flock;
    std::uniform_real_distribution<float> dis_pos(-15.0f, 15.0f);
    std::uniform_real_distribution<float> dis_vel(-1.0f, 1.0f);

    Vector3 polarization = { 0.0f, 0.0f, 0.0f };
    Vector3 momentum = { 0.0f, 0.0f, 0.0f };
    Vector3 flockCenter = { 0.0f, 0.0f, 0.0f };

    // Orbit camera
    float radius = 150.0f;
    float alpha = 0.0f;
    float beta = 1.0f;
    float turnSpeed = 0.002f;
    bool uiCapturedMouse = false;

    // Random-swarm init (for upward sweeps). Swap with the seeded-torus
    // init below by commenting/uncommenting.
    // for (int i = 0; i < num_boids; ++i) {
    //     Vector3 velocity = { dis_vel(gen), dis_vel(gen), dis_vel(gen) };
    //     velocity = Vector3Scale(Vector3Normalize(velocity), speed);
    //     flock.boids.push_back(Boid(dis_pos(gen), dis_pos(gen), dis_pos(gen),
    //                                velocity.x, velocity.y, velocity.z));
    // }

    // Seeded-torus init: a FAT ring (tube), not a thin circle.
    // Mean radius 9 (comfortably above the ~4.3 minimum turning radius),
    // +/-2.5 radial and vertical thickness so the torus starts with the
    // cross-section it needs. Velocities tangent to the ring.
    std::uniform_real_distribution<float> jitter(-1.0f, 1.0f);
    for (int i = 0; i < num_boids; ++i) {
        float theta = 2.0f * PI * i / num_boids;
        float ringR = 9.0f + jitter(gen) * 2.5f;   // radial thickness
        float x = ringR * cosf(theta);
        float y = jitter(gen) * 2.5f;              // vertical thickness
        float z = ringR * sinf(theta);
        // velocity tangent to the ring:
        Vector3 v = Vector3Scale(Vector3Normalize({ -sinf(theta), 0.0f, cosf(theta) }), speed);
        flock.boids.push_back(Boid(x, y, z, v.x, v.y, v.z));
    }

    Camera3D camera = {};
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);

    std::vector<Vector3> newVelocities;
    newVelocities.reserve(num_boids);

    while (!WindowShouldClose()) {
        // Camera drag -- disabled while a slider drag is in progress
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !uiCapturedMouse) {
            Vector2 mouseDelta = GetMouseDelta();
            alpha -= mouseDelta.x * turnSpeed;
            beta  += mouseDelta.y * turnSpeed;
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

        BeginDrawing();
            ClearBackground(DARKBLUE);

            BeginMode3D(camera);
                for (const auto &boid : flock.boids) {
                    drawBoid(boid);
                }
            EndMode3D();

            DrawFPS(10, 10);
            DrawText(TextFormat("Polarization: %.2f", Vector3Length(polarization)), 10, 30, 20, WHITE);
            DrawText(TextFormat("Momentum: %.2f", Vector3Length(momentum)), 10, 60, 20, WHITE);

            // Sliders (bottom right), parameterized as zone WIDTHS like the paper
            float sw = (float)GetScreenWidth();
            float sh = (float)GetScreenHeight();
            bool sliderActive = false;

            float d_ro = zone_of_orientation - zone_of_repulsion;
            float d_ra = zone_of_attraction - zone_of_orientation;

            d_ro = Slider({ sw - 220, sh - 90, 200, 12 }, "d_ro", d_ro, 0.0f, 15.0f, &sliderActive);
            d_ra = Slider({ sw - 220, sh - 40, 200, 12 }, "d_ra", d_ra, 0.0f, 20.0f, &sliderActive);

            zone_of_orientation = zone_of_repulsion + d_ro;
            zone_of_attraction  = zone_of_orientation + d_ra;

            // Sticky capture: once a slider drag starts, lock the camera out
            // until the mouse button is released.
            uiCapturedMouse = sliderActive || (uiCapturedMouse && IsMouseButtonDown(MOUSE_BUTTON_LEFT));
        EndDrawing();

        // --- Simulation step (synchronous update) ---
        newVelocities.clear();
        for (auto &boid : flock.boids) {
            newVelocities.push_back(updateBoidVelocity(boid, flock));
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
        flockCenter = Vector3Scale(flockCenter, 1.0f / flock.boids.size());

        // --- Order parameters (paper definitions, both in [0, 1]) ---
        polarization = { 0.0f, 0.0f, 0.0f };
        momentum = { 0.0f, 0.0f, 0.0f };
        for (auto &boid : flock.boids) {
            Vector3 ric = Vector3Normalize(Vector3Subtract({ boid.x, boid.y, boid.z }, flockCenter));
            Vector3 vhat = Vector3Normalize({ boid.vx, boid.vy, boid.vz });
            momentum = Vector3Add(momentum, Vector3CrossProduct(ric, vhat));
            polarization = Vector3Add(polarization, vhat);
        }
        momentum = Vector3Scale(momentum, 1.0f / flock.boids.size());
        polarization = Vector3Scale(polarization, 1.0f / flock.boids.size());
    }

    CloseWindow();
    return 0;
}