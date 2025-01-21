#include <fmt/base.h>
#include<vector>
#include<fmt/core.h>
#include<random>
#include<algorithm>

#include "glm/vec2.hpp"
#include "glm/vec3.hpp"
#include "glm/geometric.hpp"
#include "raylib.h"
#include "rlImGui.h"
#include "rlgl.h"
#include "raymath.h"

#include "imgui.h"
#include "imgui_impl_raylib.h"


const int DIMENSIONS = 2;

float PARTICLE_RADIUS = 0.128f;
float SMOOTHING_RADIUS = 1.0f;  // try to keep 25 neighbours within range
float PRESSURE_MULTIPLIER = 256.0f;
float TARGET_DENSITY = 2.0f;
bool COLLISION = true;

// Physics
const int   NUM_PARTICLES = 1000;
const float PARTICLE_MASS    = 0.5f;
const float DELTA_TIME       = 0.046f;
const glm::vec2 BOUNDS = glm::vec2(100, 70);
float ACCELERATION_DAMPING = 0.216f;

// Boundary vars
const float BOUNDARY_EPSILON = 0.001f;

std::vector<std::vector<int>> accelerationGrid;
const glm::vec2 accGridSize = {std::ceil(BOUNDS.x / SMOOTHING_RADIUS), std::ceil(BOUNDS.y / SMOOTHING_RADIUS)};

// Create a random engine
std::random_device rd;          // Seed with a random value from hardware (if available)
std::mt19937 gen;         // Mersenne Twister engine for randomness

// Define a uniform distribution in the desired range (e.g., 0.0 to 1.0)
std::uniform_real_distribution<> uniform(0.0, 1.0); // Generates random floats in range [0.0, 1.0]


struct Particle {
  glm::vec2 position;
  glm::vec2 velocity;
  glm::vec2 acceleration;
  float density;
  float pressure;
};


float SmoothingKernel(const float distance, const float maxDistance) {
  float h1 = 1.0f / SMOOTHING_RADIUS;
  float sigma = 0.0f;
  if (DIMENSIONS == 1) {
    sigma = h1 * 2.0f / 3.0f;
  } else if (DIMENSIONS == 2) {
    sigma = h1 * h1 * 10.0f * PI / 7.0f;
  } else if (DIMENSIONS == 3) {
    sigma = h1 * h1 * h1 * PI;
  }

  float value = 0.0f;
  float q = distance * h1;
  float tmp2 = 2.0f - q;
  if (q > 2.0) {
    return 0.0f;
  }else if (q > 1.0) {
    return sigma * 0.25 * tmp2 * tmp2 * tmp2;
  } else {
    return sigma * 1 - 1.5 * q * q * (1 - 0.5 * q);
  }
}

float SmoothingKernelDerivative(const float distance, const float maxDistance) {
  float h1 = 1.0f / SMOOTHING_RADIUS;
  float sigma = 0.0f;
  if (DIMENSIONS == 1) {
    sigma = h1 * 2.0f / 3.0f;
  } else if (DIMENSIONS == 2) {
    sigma = h1 * h1 * 10.0f * PI / 7.0f;
  } else if (DIMENSIONS == 3) {
    sigma = h1 * h1 * h1 * PI;
  }

  // compute sigma * dw_dq
  float value = 0.0f;
  float q = distance * h1;
  float tmp2 = 2. - q;
  if (distance > 1e-12) {
    if (q > 2.0) {
      return 0.0f;
    } else if (q > 1.0) {
      return sigma * -0.75 * tmp2 * tmp2;
    } else {
      return sigma * -3.0 * q * (1 - 0.75 * q);
    }
  }
}

void UpdateDensity(std::vector<Particle>& particles, const float maxDistance) {
  for (int i=0; i<NUM_PARTICLES; i++) {
    particles[i].density = PARTICLE_MASS * SmoothingKernel(0, SMOOTHING_RADIUS);
    for (int j=i + 1; j<NUM_PARTICLES; j++) {
      glm::vec2 diff = particles[i].position - particles[j].position;
      float densityIJ = PARTICLE_MASS * SmoothingKernel(glm::length(diff), SMOOTHING_RADIUS);
      particles[i].density += densityIJ;
      particles[j].density += densityIJ;
    }
  }
}


void UpdatePressure(std::vector<Particle>& particles, const float maxDistance) {
  for (int i=0; i<NUM_PARTICLES; i++) {
    particles[i].pressure = PRESSURE_MULTIPLIER * (particles[i].density - TARGET_DENSITY);
  }
}

void UpdateAcceleration(std::vector<Particle> &particles,
                        const float maxDistance) {
  // Damping
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].acceleration =
        -ACCELERATION_DAMPING * particles[i].velocity + glm::vec2(0, 10);
  }

  // Pressure
  for (int i = 0; i < NUM_PARTICLES; i++) {
    Particle &particle = particles[i];
    for (int j = i + 1; j < NUM_PARTICLES; j++) {
      if (i == j)
        continue;

      Particle &otherParticle = particles[j];
      glm::vec2 diff = particles[i].position - particles[j].position;
      float pa = -PARTICLE_MASS *
                 (particles[i].pressure /
                      (std::pow(particles[i].density, 2.0f) + 0.0001f) +
                  particles[j].pressure /
                      (std::pow(particles[j].density, 2.0f) + 0.0001f)) *
                 SmoothingKernelDerivative(glm::length(diff), SMOOTHING_RADIUS);
      particles[i].acceleration += pa * diff / (glm::length(diff) + 0.0001f);
      particles[j].acceleration -= pa * diff / (glm::length(diff) + 0.0001f);

      if (COLLISION) {
        // Particle Collision Resolution
        glm::vec2 diffVector = otherParticle.position - particle.position;
        float distance = glm::length(diffVector);
        if (distance < 2 * PARTICLE_RADIUS) {
          float penetration = 2 * PARTICLE_RADIUS - distance;

          // Update position
          glm::vec2 correctionDisplacement =
              0.5f * penetration * diffVector / (distance + 0.0001f);

          glm::vec2 correctionVelocity =
              correctionDisplacement / DELTA_TIME;
          correctionDisplacement += penetration/2 * glm::vec2(2 * (0.5 - uniform(gen)), 2 * (0.5 - uniform(gen)));

          otherParticle.position += correctionDisplacement;
          particle.position -= correctionDisplacement;

          // // Update velocity
          // otherParticle.velocity += correctionVelocity;
          // particle.velocity -= correctionVelocity;
        }
      }
    }
  }
}

void ApplyInput(std::vector<Particle>& particles, glm::vec2 mousePosition, glm::vec2 mouseDelta) {
  for (int i=0; i<NUM_PARTICLES; i++) {
    float distance = glm::length(mousePosition - particles[i].position);
    if (distance < 10.0f) {
      particles[i].acceleration = 10.0f * mouseDelta / (glm::length(mouseDelta) + 0.0001f);
    }
  }
}


void UpdatePosition(std::vector<Particle>& particles, const float maxDistance) {
  for (int i=0; i<NUM_PARTICLES; i++) {
    Particle &particle = particles[i];

    particle.velocity += particle.acceleration * DELTA_TIME;
    particle.position += particle.velocity     * DELTA_TIME;

    // BOUNDARY CONDITIONS
    if (particle.position.y - PARTICLE_RADIUS <= 0 || particle.position.y + PARTICLE_RADIUS >= BOUNDS.y){
      particle.velocity.y = -ACCELERATION_DAMPING * particle.velocity.y;
      particle.velocity.y *= -1;
      particle.position.y += std::min(particle.position.y - PARTICLE_RADIUS - BOUNDARY_EPSILON, 0.0f);
      particle.position.y += std::min(BOUNDS.y - particle.position.y + PARTICLE_RADIUS + BOUNDARY_EPSILON, 0.0f);
      particle.position.y = std::clamp(particle.position.y, 0 + PARTICLE_RADIUS + BOUNDARY_EPSILON, BOUNDS.y - PARTICLE_RADIUS - BOUNDARY_EPSILON);
    }

    if (particle.position.x - PARTICLE_RADIUS <= 0 || particle.position.x + PARTICLE_RADIUS >= BOUNDS.x){
      particle.velocity.x = -ACCELERATION_DAMPING * particle.velocity.x;
      particle.velocity.x *= -1;
      particle.position.x += std::min(particle.position.x - PARTICLE_RADIUS - BOUNDARY_EPSILON, 0.0f);
      particle.position.x += std::min(BOUNDS.x - particle.position.x + PARTICLE_RADIUS + BOUNDARY_EPSILON, 0.0f);
      particle.position.x = std::clamp(particle.position.x, 0 + PARTICLE_RADIUS + BOUNDARY_EPSILON, BOUNDS.x - PARTICLE_RADIUS - BOUNDARY_EPSILON);
    }


  }
}

int main() {

  // Create a random engine
  std::random_device rd;          // Seed with a random value from hardware (if available)
  std::mt19937 gen(rd());         // Mersenne Twister engine for randomness

  // Define a uniform distribution in the desired range (e.g., 0.0 to 1.0)
  std::uniform_real_distribution<> uniform(0.0, 1.0); // Generates random floats in range [0.0, 1.0]

  std::vector<Particle> particles;
  for (int i = 0; i < NUM_PARTICLES; i++) {
    Particle new_particle;
    new_particle.position = glm::vec2(uniform(gen) * BOUNDS[0],
                                      uniform(gen) * BOUNDS[1]);
    new_particle.velocity = glm::vec2(0 * (uniform(gen) - 0.5), 0 * (uniform(gen) - 0.5));

    particles.push_back(new_particle);
  }

  // Visualization
  //--------------------------------------------------------------------------------------
  const glm::vec2 screenSize = glm::vec2(1920, 1080);
  const glm::vec2 VIS_SCALE = screenSize / BOUNDS;

  InitWindow(screenSize[0], screenSize[1], "ABC");
  ToggleFullscreen();
  rlImGuiSetup(true);

  Camera2D camera = { 0 };
  camera.zoom = 1.0f;
  SetTargetFPS(60);

  while (!WindowShouldClose()){
    glm::vec2 mousePosition = {GetMousePosition().x, GetMousePosition().y};
    glm::vec2 mouseDelta = {GetMouseDelta().x, GetMouseDelta().y};

    // Simulate
    UpdateDensity(particles, SMOOTHING_RADIUS);
    UpdatePressure(particles, SMOOTHING_RADIUS);
    UpdateAcceleration(particles, SMOOTHING_RADIUS);

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      fmt::println("Position: {}, {}, Delta: {}, {}", mousePosition.x, mousePosition.y, mouseDelta.x, mouseDelta.y);
      ApplyInput(particles, mousePosition / VIS_SCALE, mouseDelta / VIS_SCALE);
    }

    UpdatePosition(particles, SMOOTHING_RADIUS);

    // Draw
    BeginDrawing();
    rlImGuiBegin();
    ClearBackground({1, 0, 0, 255});
    BeginMode2D(camera);

    static float maxVelocity = 10.0f;
    static float midVelocity = 5.0f;
    static float minVelocity = 0.0f;
    for(auto& particle: particles) {
      glm::vec3 red = {255, 0, 0};
      glm::vec3 green = {0, 255, 0};
      glm::vec3 blue = {0, 0, 255};

      float vel = std::clamp(glm::length(particle.velocity), minVelocity, maxVelocity);
      fmt::println("{}", vel);
      glm::vec3 finalColor = 
          (1 - std::abs(vel - maxVelocity) / maxVelocity) * red +
          (1 - std::abs(vel - midVelocity) / maxVelocity) * green +
          (1 - std::abs(vel - minVelocity) / maxVelocity) * blue;

      DrawCircle(particle.position[0] * VIS_SCALE.x,
                 particle.position[1] * VIS_SCALE.y,
                 PARTICLE_RADIUS * VIS_SCALE.x,
                 {(unsigned char)finalColor.x, (unsigned char)finalColor.y,
                  (unsigned char)finalColor.z, 255});
    }

    DrawCircleLinesV(Vector2{mousePosition.x, mousePosition.y}, 2 * SMOOTHING_RADIUS * VIS_SCALE.x, WHITE);

    // // Visualize density squares
    // const float maxDistance = SMOOTHING_RADIUS;
    // for (int y=0; y<BOUNDS.y; y++) {
    //   for (int x = 0; x < BOUNDS.x; x++) {
    //     float density = CalculateDensity(glm::vec2(x + 0.5, y + 0.5),
    //     maxDistance, particles); density = std::clamp(density, 0.0f, 1.0f);
    //     Color color = {0, 0, (unsigned char)(255 * density), 100};

    //     DrawRectangleV({x * VIS_SCALE.x, y * VIS_SCALE.y},
    //                    {VIS_SCALE.x, VIS_SCALE.y}, color);
    //   }
    // }

    // draw the debug panel
    ImGui::Begin("Parameters");
    ImGui::Text("%s", fmt::format("FPS: {}", GetFPS()).c_str());
    ImGui::SliderFloat("Pressure Radius: ", &PARTICLE_RADIUS, 0.01f, 5.0f);
    ImGui::SliderFloat("Smoothing Radius: ", &SMOOTHING_RADIUS, 0.01f, 30.0f);
    ImGui::SliderFloat("Pressure Multipler (k): ", &PRESSURE_MULTIPLIER, 0.0f, 500.0f);
    ImGui::SliderFloat("Target density: ", &TARGET_DENSITY, 0.0f, 20.2f);
    ImGui::Text("Visualization and Physics");
    ImGui::Checkbox("Collision", &COLLISION);
    ImGui::SliderFloat("Max Velocity Vis", &maxVelocity, 0.0f, 10.0f);
    ImGui::SliderFloat("Mid Velocity Vis", &midVelocity, 0.0f, 10.0f);
    ImGui::SliderFloat("Acceleration Damping", &ACCELERATION_DAMPING, 0.0f, 1.0f);
    ImGui::End();

    EndMode2D();
    rlImGuiEnd();
    EndDrawing();
  }

  return 0;
}
