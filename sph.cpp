#include <algorithm>
#include <fmt/base.h>
#include <fmt/core.h>
#include <random>
#include <cmath> // for std::isnan
#include <stdexcept> // for std::runtime_error
#include <list>

#include "glm/geometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "rlgl.h"

#include "imgui.h"
#include "imgui_impl_raylib.h"

#include "quadtree.h"

/* Reference:
Eurographics/ACM SIGGRAPH Symposium on Computer Animation (2005)
K. Anjyo, P. Faloutsos (Editors)
Particle-based Viscoelastic Fluid Simulation
Simon Clavet, Philippe Beaudoin, and Pierre Poulin †
LIGUM, Dept. IRO, Université de Montréal
*/

const float h = 0.5f; // Particle interaction radius
const float hinv = 1.0f / h;
float k = 140.0f; // Stiffness constant (Pressure Multiplier)
float rho0 = 2.0f; // Target density
const float sigma = 0.05f; // Some viscosity factor
const float beta = 0.05f; // Some viscosity factor

// Physics
const int NUM_PARTICLES = 3000;
const float DELTA_TIME = 0.026f;
const glm::vec2 BOUNDS = glm::vec2(50, 20);
float ACCELERATION_DAMPING = 0.003f;
bool gravity = true;
bool drawQuadtree = false;

// Boundary vars
const float BOUNDARY_EPSILON = 0.001f;

// Create a random engine
std::random_device rd; // Seed with a random value from hardware (if available)
std::mt19937 gen; // Mersenne Twister engine for randomness

// Define a uniform distribution in the desired range (e.g., 0.0 to 1.0)
std::uniform_real_distribution<>
uniform(0.0, 1.0); // Generates random floats in range [0.0, 1.0]

struct Particle {
    glm::vec2 position;
    glm::vec2 velocity;
    float density;
    float pressure;
};

void ApplyInput(
    Particle *particles, glm::vec2 mousePosition,
    glm::vec2 mouseDelta
) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        float distance = glm::length(mousePosition - particles[i].position);
        if (distance < 2 * h) {
            particles[i].velocity =
                10.0f * mouseDelta / (glm::length(mouseDelta) + 0.0001f);
        }
    }
}

void ApplyViscosity(Particle *particles, Quadtree *tree) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        std::list<Element> elementsInRadius;
        GetElementsInRadius(particles[i].position, h, elementsInRadius, tree);

        // Loop over neighours
        for (auto &element: elementsInRadius) {
            Particle &otherParticle = particles[element.id];
            glm::vec2 rij = otherParticle.position - particles[i].position;
            float distance = glm::length(rij);

            // Check for NaN in distance
            if (std::isnan(distance)) {
                throw std::runtime_error("Detected NaN in 'distance' computation.");
            }

            glm::vec2 direction = glm::normalize(rij);
            if (distance == 0 || distance > h)
                continue;

            float q = distance * hinv;

            // Check for NaN in q
            if (std::isnan(q)) {
                throw std::runtime_error("Detected NaN in 'q' computation.");
            }

            float u = glm::dot(otherParticle.velocity - particles[i].velocity, direction);

            // Check for NaN in u
            if (std::isnan(u)) {
                throw std::runtime_error("Detected NaN in 'u' computation.");
            }

            glm::vec2 I = DELTA_TIME * (1.0f - q) * (sigma * u + beta * u * u) * rij;

            // Check for NaN in impulse vector I
            if (std::isnan(I.x) || std::isnan(I.y)) {
                throw std::runtime_error("Detected NaN in impulse vector 'I' computation.");
            }

            particles[i].velocity -= 0.5f * I;
            otherParticle.velocity += 0.5f * I;
        }
    }
}


void AdjustSprings() {
}

void ApplySpringDisplacements() {
}

void DoubleDensityRelaxation(Particle *particles, Quadtree *tree) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        float rho = 0;
        float rhoNear = 0;

        std::list<Element> elementsInRadius;
        GetElementsInRadius(particles[i].position, h, elementsInRadius, tree);

        // Compute density and near density
        for (auto &element: elementsInRadius) {
            Particle &otherParticle = particles[element.id];
            float distance = glm::length(otherParticle.position - particles[i].position);

            // Check for NaN in distance
            if (std::isnan(distance)) {
                throw std::runtime_error("Detected NaN in 'distance' computation.");
            }

            if (distance == 0 || distance > h)
                continue;

            float q = distance / h;

            // Check for NaN in q
            if (std::isnan(q)) {
                throw std::runtime_error("Detected NaN in 'q' computation.");
            }

            rho += (1 - q) * (1 - q);
            rhoNear += (1 - q) * (1 - q) * (1 - q);
        }

        // Compute pressure and near pressure
        float P = k * (rho - rho0);
        float PNear = k * rhoNear;

        // Check for NaN in P and PNear
        if (std::isnan(P) || std::isnan(PNear)) {
            throw std::runtime_error("Detected NaN in 'P' or 'PNear' computation.");
        }

        // Compute and apply displacements
        glm::vec2 dx = {0, 0};
        for (auto &element: elementsInRadius) {
            Particle &otherParticle = particles[element.id];

            glm::vec2 rij = otherParticle.position - particles[i].position;
            float distance = glm::length(rij);

            // Check for NaN in distance
            if (std::isnan(distance)) {
                throw std::runtime_error("Detected NaN in 'distance' computation during displacement.");
            }

            if (distance > h)
                continue;

            float q = distance * hinv;

            // Check for NaN in q
            if (std::isnan(q)) {
                throw std::runtime_error("Detected NaN in 'q' computation during displacement.");
            }

            glm::vec2 D = DELTA_TIME * DELTA_TIME *
                          (P * (1 - q) + PNear * (1 - q) * (1 - q)) * rij;

            // Check for NaN in displacement vector D
            if (std::isnan(D.x) || std::isnan(D.y)) {
                throw std::runtime_error("Detected NaN in displacement vector 'D' computation.");
            }

            otherParticle.position += 0.5f * D;
            dx -= 0.5f * D;
        }

        // Check for NaN in dx before applying it
        if (std::isnan(dx.x) || std::isnan(dx.y)) {
            throw std::runtime_error("Detected NaN in final displacement 'dx' computation.");
        }

        particles[i].position += dx;
    }
}

void ResolveCollisions() {
}

void Simulate(Particle *particles, Quadtree *tree) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        Particle &particle = particles[i];

        // Apply gravity
        particle.velocity -= DELTA_TIME * ACCELERATION_DAMPING * particle.velocity;

        if (gravity) {
            particle.velocity += DELTA_TIME * glm::vec2(0, 10);
        }

        const float MAX_VELOCITY = 100.0f;
        particle.velocity.x = std::clamp(particle.velocity.x, -MAX_VELOCITY, MAX_VELOCITY);
        particle.velocity.y = std::clamp(particle.velocity.y, -MAX_VELOCITY, MAX_VELOCITY);
    }

    // Modify velocities with pairwise velocity impulses
    ApplyViscosity(particles, tree);

    glm::vec2 previousPositions[NUM_PARTICLES];
    for (int i = 0; i < NUM_PARTICLES; i++) {
        // record previous velocity
        previousPositions[i] = particles[i].position;
        // advance to predicted position
        particles[i].position += DELTA_TIME * particles[i].velocity;
    }

    // Add and remove springs, change rest lengths
    AdjustSprings(); // TODO

    // Modify positions according to springs, double density relaxation and
    // collisions
    ApplySpringDisplacements(); // TODO
    DoubleDensityRelaxation(particles, tree);
    ResolveCollisions(); // TODO

    for (int i = 0; i < NUM_PARTICLES; i++) {
        // Use previous position to compute next velocity
        particles[i].velocity =
            (particles[i].position - previousPositions[i]) / DELTA_TIME;
    }

    // BOUNDARY CONDITIONS
    for (int i = 0; i < NUM_PARTICLES; i++) {
        Particle &particle = particles[i];
        if (particle.position.y - h <= 0 || particle.position.y + h >= BOUNDS.y) {
            particle.velocity.y *= -1 * 0.5;
            particle.position.y +=
                std::min(particle.position.y - h - BOUNDARY_EPSILON, 0.0f);
            particle.position.y +=
                std::min(BOUNDS.y - particle.position.y + h + BOUNDARY_EPSILON, 0.0f);
            particle.position.y =
                std::clamp(
                    particle.position.y, 0 + h + BOUNDARY_EPSILON,
                    BOUNDS.y - h - BOUNDARY_EPSILON
                );
        }

        if (particle.position.x - h <= 0 || particle.position.x + h >= BOUNDS.x) {
            particle.velocity.x *= -1 * 0.5;
            particle.position.x +=
                std::min(particle.position.x - h - BOUNDARY_EPSILON, 0.0f);
            particle.position.x +=
                std::min(BOUNDS.x - particle.position.x + h + BOUNDARY_EPSILON, 0.0f);
            particle.position.x =
                std::clamp(
                    particle.position.x, 0 + h + BOUNDARY_EPSILON,
                    BOUNDS.x - h - BOUNDARY_EPSILON
                );
        }
    }
}

int main() {
    // Create a random engine
    std::random_device
        rd; // Seed with a random value from hardware (if available)
    std::mt19937 gen(rd()); // Mersenne Twister engine for randomness

    // Define a uniform distribution in the desired range (e.g., 0.0 to 1.0)
    std::uniform_real_distribution<> uniform(
        0.0, 1.0
    ); // Generates random floats in range [0.0, 1.0]

    Particle particles[NUM_PARTICLES];
    for (int i = 0; i < NUM_PARTICLES; i++) {
        Particle new_particle;
        new_particle.position =
            glm::vec2(uniform(gen) * BOUNDS[0], uniform(gen) * BOUNDS[1]);
        new_particle.velocity =
            glm::vec2(0 * (uniform(gen) - 0.5), 0 * (uniform(gen) - 0.5));

        particles[i] = new_particle;
    }

    // Visualization
    //--------------------------------------------------------------------------------------
    const glm::vec2 screenSize = glm::vec2(1920, 1080);
    const glm::vec2 VIS_SCALE = screenSize / BOUNDS;

    InitWindow(screenSize[0], screenSize[1], "ABC");
    ToggleFullscreen();
    rlImGuiSetup(true);

    Camera2D camera = {0};
    camera.zoom = 1.0f;
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        glm::vec2 mousePosition = {GetMousePosition().x, GetMousePosition().y};
        glm::vec2 mouseDelta = {GetMouseDelta().x, GetMouseDelta().y};

        // Simulate
        Quadtree tree;
        tree.aabb = {glm::vec2(0, 0), BOUNDS};
        for (int i = 0; i < NUM_PARTICLES; i++) {
            Element element = {i, {particles[i].position - 0.05f, glm::vec2(0.1, 0.1)}};
            Insert(&tree, element);
        }
        Simulate(particles, &tree);

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            ApplyInput(particles, mousePosition / VIS_SCALE, mouseDelta / VIS_SCALE);
        }

        // Draw
        BeginDrawing();
        rlImGuiBegin();
        ClearBackground({1, 0, 0, 255});
        BeginMode2D(camera);

        static float maxVelocity = 10.0f;
        static float midVelocity = 5.0f;
        static float minVelocity = 0.0f;
        for (int i = 0; i < NUM_PARTICLES; i++) {
            Particle &particle = particles[i];

            glm::vec3 red = {255, 0, 0};
            glm::vec3 green = {0, 255, 0};
            glm::vec3 blue = {0, 0, 255};

            float vel =
                std::clamp(glm::length(particle.velocity), minVelocity, maxVelocity);

            glm::vec3 finalColor =
                (1 - std::abs(vel - maxVelocity) / maxVelocity) * red +
                (1 - std::abs(vel - midVelocity) / maxVelocity) * green +
                (1 - std::abs(vel - minVelocity) / maxVelocity) * blue;

            DrawCircle(
                particle.position[0] * VIS_SCALE.x,
                particle.position[1] * VIS_SCALE.y, std::max(h / 8.0f * VIS_SCALE.x, 1.0f),
                {
                    (unsigned char) finalColor.x, (unsigned char) finalColor.y,
                    (unsigned char) finalColor.z, 255
                }
            );
        }

        DrawCircleLinesV(
            Vector2{mousePosition.x, mousePosition.y},
            2 * h * VIS_SCALE.x, WHITE
        );

        std::list<Element> elementsInRadius;
        GetElementsInRadius(mousePosition / VIS_SCALE, 2 * h, elementsInRadius, &tree);

        for (auto element: elementsInRadius) {
            DrawCircle(
                particles[element.id].position.x * VIS_SCALE.x, particles[element.id].position.y * VIS_SCALE.y,
                h / 4.0f * VIS_SCALE.x, WHITE
            );
        }

        if (drawQuadtree) {
            DrawQuadtree(&tree, VIS_SCALE);
        }

        // draw the debug panel
        ImGui::Begin("Parameters");
        ImGui::Text("%s", fmt::format("FPS: {}", GetFPS()).c_str());
        ImGui::Text("%s", fmt::format("Num particles in radius: {}", elementsInRadius.size()).c_str());
        ImGui::SliderFloat("Pressure Multipler (k): ", &k, 0.0f, 500.0f);
        ImGui::SliderFloat("Target density: ", &rho0, 0.0f, 20.2f);
        ImGui::Text("Visualization and Physics");
        ImGui::Checkbox("Gravity: ", &gravity);
        ImGui::Checkbox("Draw quadtree: ", &drawQuadtree);
        ImGui::SliderFloat("Max Velocity Vis", &maxVelocity, 0.0f, 10.0f);
        ImGui::SliderFloat("Mid Velocity Vis", &midVelocity, 0.0f, 10.0f);
        ImGui::SliderFloat(
            "Acceleration Damping", &ACCELERATION_DAMPING, 0.0f,
            1.0f
        );
        ImGui::End();

        EndMode2D();
        rlImGuiEnd();
        EndDrawing();
    }

    return 0;
}
