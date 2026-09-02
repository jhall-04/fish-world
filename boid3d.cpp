#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <random>

constexpr int flock_area = 500;

constexpr float BOID_LENGTH = flock_area * 0.05f; // Length of the boid for drawing
constexpr float BOID_RADIUS = flock_area * 0.005f; // Radius of the boid for drawing

constexpr int zone_of_repulsion = 10;
constexpr int zone_of_orientation = 20;
constexpr int zone_of_attraction = 30;

constexpr float max_turn_angle = 0.1f; // Maximum turn angle in radians per update

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
};

void drawBoid(const Boid &boid) {
    Vector3 pos = { boid.x, boid.y, boid.z };
    Vector3 vel = { boid.vx, boid.vy, boid.vz };

    float speed = Vector3Length(vel);
    Vector3 dir = (speed > 1e-5f) ? Vector3Scale(vel, 1.0f / speed)
                                  : Vector3{ 0.0f, 0.0f, 1.0f };

    Vector3 tail = Vector3Subtract(pos, Vector3Scale(dir, BOID_LENGTH * 0.5f));
    Vector3 nose = Vector3Add(pos, Vector3Scale(dir, BOID_LENGTH * 0.5f));
    
    int colorx = static_cast<int>((boid.x + flock_area / 2.0f) * 255.0f / flock_area);
    int colory = static_cast<int>((boid.y + flock_area / 2.0f) * 255.0f / flock_area);
    int colorz = static_cast<int>((boid.z + flock_area / 2.0f) * 255.0f / flock_area);

    DrawCylinderEx(tail, nose, BOID_RADIUS, 0.0f, 8, Color{ (unsigned char)colorx, (unsigned char)colory, (unsigned char)colorz, 255 });
}

Vector3 vector3Normalize(Vector3 v) {
    float length = Vector3Length(v);
    if (length > 1e-5f) {
        return Vector3Scale(v, 1.0f / length);
    }
    return Vector3{ 0.0f, 0.0f, 0.0f };
}

float angleBetweenVectors(Vector3 a, Vector3 b) {
    float dot = Vector3Dot(a, b);
    float lengths = Vector3Length(a) * Vector3Length(b);
    if (lengths > 1e-5f) {
        return acosf(dot / lengths);
    }
    return 0.0f;
}

Vector3 updateBoidVelocity(const Boid &boid, const Flock &flock) {
    int repulsionCount = 0;
    int orientationCount = 0;
    int attractionCount = 0;
    Vector3 repulsion = { 0.0f, 0.0f, 0.0f };
    Vector3 orientation = { 0.0f, 0.0f, 0.0f };
    Vector3 attraction = { 0.0f, 0.0f, 0.0f };
    for (const auto &other : flock.boids) {
        if (&other != &boid) {
            float dist = Vector3Distance({ boid.x, boid.y, boid.z }, { other.x, other.y, other.z });
            if (dist < zone_of_repulsion) {
                Vector3 normRep = vector3Normalize({ boid.x - other.x, boid.y - other.y, boid.z - other.z });
                repulsion.x += normRep.x;
                repulsion.y += normRep.y;
                repulsion.z += normRep.z;
                repulsionCount++;
            } else if (dist < zone_of_orientation) {
                Vector3 normVel = vector3Normalize({ other.vx, other.vy, other.vz });
                orientation.x += normVel.x;
                orientation.y += normVel.y;
                orientation.z += normVel.z;
                orientationCount++;
            } else if (dist < zone_of_attraction) {
                Vector3 normAttr = vector3Normalize({ other.x - boid.x, other.y - boid.y, other.z - boid.z });
                attraction.x += normAttr.x;
                attraction.y += normAttr.y;
                attraction.z += normAttr.z;
                attractionCount++;
            }
        }
    }
    if (repulsionCount > 0) {
        repulsion = vector3Normalize(repulsion);
        repulsion.x /= repulsionCount;
        repulsion.y /= repulsionCount;
        repulsion.z /= repulsionCount;
        
    }
    if (orientationCount > 0) {
        orientation = vector3Normalize(orientation);
        orientation.x /= orientationCount;
        orientation.y /= orientationCount;
        orientation.z /= orientationCount;
    }
    if (attractionCount > 0) {
        attraction = vector3Normalize(attraction);
        attraction.x /= attractionCount;
        attraction.y /= attractionCount;
        attraction.z /= attractionCount;
    }

    return { boid.vx, boid.vy, boid.vz };
}




int main() {
    InitWindow(800, 600, "3d");

    Flock flock;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis_pos(-flock_area / 2.0f, flock_area / 2.0f);
    std::uniform_real_distribution<float> dis_vel(-1.0f, 1.0f);

    // Orbit variables
    float radius = flock_area * 1.5f; // Distance from the center of the flock
    float alpha = 0.0f; // Horizontal angle
    float beta = 1.0f;  // Vertical angle
    
    // ADJUST THIS TO CHANGE SPEED (Lower = Slower)
    float turnSpeed = 0.002f; 

    for (int i = 0; i < 300; ++i) {
        flock.boids.push_back(Boid(dis_pos(gen), dis_pos(gen), dis_pos(gen), dis_vel(gen), dis_vel(gen), dis_vel(gen)));
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
                DrawCubeWires({0.0f, 0.0f, 0.0f}, (float)flock_area, (float)flock_area, (float)flock_area, WHITE);
            EndMode3D();
            DrawFPS(10, 10);
        EndDrawing();
        for (auto &boid : flock.boids) {
            newVelocities.push_back(updateBoidVelocity(boid, flock));
        }
        for (size_t i = 0; i < flock.boids.size(); ++i) {
            flock.boids[i].vx = newVelocities[i].x;
            flock.boids[i].vy = newVelocities[i].y;
            flock.boids[i].vz = newVelocities[i].z;
        }
        for (auto &boid : flock.boids) {
            boid.x += boid.vx;
            boid.y += boid.vy;
            boid.z += boid.vz;

            // Wrap around the cube boundaries
            if (boid.x > flock_area / 2.0f) boid.x = -flock_area / 2.0f;
            if (boid.x < -flock_area / 2.0f) boid.x = flock_area / 2.0f;
            if (boid.y > flock_area / 2.0f) boid.y = -flock_area / 2.0f;
            if (boid.y < -flock_area / 2.0f) boid.y = flock_area / 2.0f;
            if (boid.z > flock_area / 2.0f) boid.z = -flock_area / 2.0f;
            if (boid.z < -flock_area / 2.0f) boid.z = flock_area / 2.0f;
        }
    }

    CloseWindow();
    return 0;
}