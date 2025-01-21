#include <algorithm>
#include <cmath>
#include <fmt/base.h>
#include <fmt/core.h>
#include <random>
#include <tuple>
#include <vector>

#include "glm/geometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "rlgl.h"

#include "imgui.h"
#include "imgui_impl_raylib.h"


//
//  ┌─────┌─────┌─────┌─────┐─────►x      ┌─────┌─────┌─────┌─────┐─────►x
//  │     │     │     │     │             │     │     │     │     │
//  │ 0,0 │ 1,0 │ 2,0 │ 3,0 │             │  0  │  1  │  2  │  3  │
//  │     │     │     │     │             │     │     │     │     │
//  ┌─────┌─────┌─────┌─────┐             ┌─────┌─────┌─────┌─────┐
//  │     │     │     │     │             │     │     │     │     │
//  │ 0,1 │ 1,1 │ 2,1 │ 3,1 │             │  4  │  5  │  6  │  7  │
//  │     │     │     │     │             │     │     │     │     │
//  ┌─────┌─────┌─────┌─────┐             ┌─────┌─────┌─────┌─────┐
//  │     │     │     │     │             │     │     │     │     │
//  │ 0,2 │ 1,2 │ 2,2 │ 3,2 │             │  8  │  9  │  10 │  11 │
//  │     │     │     │     │             │     │     │     │     │
//  ┌─────┌─────┌─────┌─────┐             ┌─────┌─────┌─────┌─────┐
//  │     │     │     │     │             │     │     │     │     │
//  │ 0,3 │ 1,3 │ 2,3 │ 3,3 │             │  12 │  13 │  14 │  15 │
//  │     │     │     │     │             │     │     │     │     │
//  └─────└─────└─────└─────┘             └─────└─────└─────└─────┘
//  │                                     │
//  │        NX = 4                       │
//  │        NY = 4                       │
//  ▼   NUM_CELLS = (NX, NY)              ▼
//  y                                     y
//
//
//
//
//                                                       ▲     ▲
//  ┌─────┌─────┌─────┌─────┐─────►x      ┌──┬──┌──┬──┌──┼──┌──┼──┐─────►x
//  │     │     │     │     │             │  ▼  │  ▼  │     │     │
// ◄┼     ┼►    ┼►    ┼►   ◄┼             │  v00│  v10│  v20│  v30│
//  │u00  │u10  │u20  │u30  │u40          │     │  ▲  │     │  ▲  │
//  ┌─────┌─────┌─────┌─────┐             ┌──┬──┌──┼──┌──┬──┌──┼──┐
//  │     │     │     │     │             │  ▼  │     │  ▼  │     │
//  ┼►   ◄┼     ┼►   ◄┼     ┼►            │  v01│  v11│  v21│  v31│
//  │u01  │u11  │u21  │u31  │u41          │  ▲  │     │     │     │
//  ┌─────┌─────┌─────┌─────┐             ┌──┼──┌──┬──┌──┬──┌──┬──┐
//  │     │     │     │     │             │     │  ▼  │  ▼  │  ▼  │
//  ┼►    ┼►    ┼►   ◄┼    ◄┼             │  v02│  v12│  v22│  v32│
//  │u02  │u12  │u22  │u32  │u42          │  ▲  │     │  ▲  │  ▲  │
//  ┌─────┌─────┌─────┌─────┐             ┌──┼──┌──┬──┌──┼──┌──┼──┐
//  │     │     │     │     │             │     │  ▼  │     │     │
// ◄┼    ◄┼    ◄┼     ┼►    ┼►            │  v03│  v13│  v23│  v33│
//  │u03  │u13  │u23  │u33  │u43          │     │  ▲  │     │  ▲  │
//  └─────└─────└─────└─────┘             └──┬──└──┼──└──┬──└──┼──┘
//  │                                     │  ▼           ▼
//  │                                     │  v04   v14   v24   v34
//  │                                     │
//  ▼   NUM_U = (NX+1, NY)                ▼   NUM_V = (NX, NY+1)
//  y                                     y
//

glm::ivec2 NUM_CELLS = {100, 100};
glm::ivec2 NUM_U = {NUM_CELLS.x + 1, NUM_CELLS.y};
glm::ivec2 NUM_V = {NUM_CELLS.x, NUM_CELLS.y + 1};

const int DIVERGENCE_MAX_ITERATIONS = 100;
const float DIVERGENCE_TOLERANCE = 0.0001f;
const float DISIPATION = 0.00001f;

// Create a random engine
std::random_device rd; // Seed with a random value from hardware (if available)
std::mt19937 gen;      // Mersenne Twister engine for randomness

// Define a uniform distribution in the desired range (e.g., 0.0 to 1.0)
std::uniform_real_distribution<>
    uniform(0.0, 1.0); // Generates random floats in range [0.0, 1.0]

// Convert 2D index to 1D index (Column-Major)
int Index1D(int x, int y) { return y * NUM_CELLS.x + x; }
int Index1D(glm::ivec2 index2D) { return index2D.y * NUM_CELLS.x + index2D.x; }

// Convert 1D index to 2D indices (Column-Major)
void Index2D(int index, int &x, int &y) {
  x = index % NUM_CELLS.x;
  y = index / NUM_CELLS.x;
}

void RemoveDivergence(std::vector<float> &u, std::vector<float> &v) {
  float max = 0.0f;
  for (int itt = 0; itt < DIVERGENCE_MAX_ITERATIONS; itt++) {
    for (int i = 0; i < NUM_CELLS.x * NUM_CELLS.y; i++) {
      int x, y;
      Index2D(i, x, y);

      // Solve for cell i
      // uleft + uright + vup + vdown = 0
      float uleft = u[Index1D(x, y)];
      float uright = u[Index1D(x + 1, y)];
      float vup = v[Index1D(x, y)];
      float vdown = v[Index1D(x, y + 1)];
      float residual = uleft - uright + vup - vdown;

      // Update max residual
      max = std::max(max, std::abs(residual));

      // Compute adjustment factor
      float adjustment = 0.25f * residual;

      float speed = 1.0f;
      uleft -= adjustment * speed;
      uright += adjustment * speed;
      vup -= adjustment * speed;
      vdown += adjustment * speed;
      u[Index1D(x, y)] = uleft;
      u[Index1D(x + 1, y)] = uright;
      v[Index1D(x, y)] = vup;
      v[Index1D(x, y + 1)] = vdown;

      // boundary conditions
      if (x == 0) {
        u[Index1D(x, y)] = 0.0f;
      } else if ((x + 1) == NUM_CELLS.x) {
        u[Index1D(NUM_U.x - 1, y)] = 0.0f;
      }
      if (y == 0) {
        v[Index1D(x, y)] = 0.0f;
      } else if ((y + 1) == NUM_CELLS.y) {
        v[Index1D(x, NUM_V.y - 1)] = 0.0f;
      }
    }

    if (max < DIVERGENCE_TOLERANCE) {
      break; // break iterations
    }
  }
}

void ComputeVectorField(const std::vector<float> &u,
                        const std::vector<float> &v,
                        std::vector<glm::vec2> &outField) {
  for (int i = 0; i < NUM_CELLS.x * NUM_CELLS.y; i++) {
    int x, y;
    Index2D(i, x, y);
    outField[Index1D(x, y)] =
        glm::vec2(0.5 * (u[Index1D(x, y)] + u[Index1D(x + 1, y)]),
                  0.5 * (v[Index1D(x, y)] + v[Index1D(x, y + 1)]));
  }
}

void Advection(const std::vector<glm::vec2> &vecField, std::vector<float> &outU,
               std::vector<float> &outV) {
  std::vector<float> newU(outU.size(), 0);
  std::vector<float> newV(outV.size(), 0);

  float speed = 1.0f;

  for (int i = 0; i < NUM_CELLS.x * NUM_CELLS.y; i++) {
    int x, y;
    Index2D(i, x, y);

    glm::vec2 currentPosition = {x, y};
    glm::vec2 velocity = vecField[Index1D(x, y)] * speed;

    glm::vec2 destinationCoord =
        currentPosition - velocity; // subtracting to implement pull mechanism
                                    // instead of pushing
    glm::ivec2 candidateIndex = {std::round(currentPosition.x + velocity.x),
                                 std::round(currentPosition.y + velocity.y)};
    glm::vec2 q = {destinationCoord.x - candidateIndex.x,
                   destinationCoord.y - candidateIndex.y};

    // Identify candidate indices with quadrant
    glm::ivec2 candidateIndex2 = candidateIndex;
    glm::ivec2 candidateIndex3 = candidateIndex;
    glm::ivec2 candidateIndex4 = candidateIndex;
    if (q.x >= 0 && q.y >= 0) {
      candidateIndex2.x += 1;
      candidateIndex3.y += 1;
      candidateIndex4.x += 1;
      candidateIndex4.y += 1;
    }
    if (q.x <= 0 && q.y >= 0) {
      candidateIndex2.x -= 1;
      candidateIndex3.y += 1;
      candidateIndex4.x -= 1;
      candidateIndex4.y += 1;
    }
    if (q.x >= 0 && q.y <= 0) {
      candidateIndex2.x += 1;
      candidateIndex3.y -= 1;
      candidateIndex4.x += 1;
      candidateIndex4.y -= 1;
    }
    if (q.x <= 0 && q.y <= 0) {
      candidateIndex2.x -= 1;
      candidateIndex3.y -= 1;
      candidateIndex4.x -= 1;
      candidateIndex4.y -= 1;
    }

    // Distance from candidates
    float d1 = glm::distance((glm::vec2)candidateIndex, destinationCoord);
    float d2 = glm::distance((glm::vec2)candidateIndex2, destinationCoord);
    float d3 = glm::distance((glm::vec2)candidateIndex3, destinationCoord);
    float d4 = glm::distance((glm::vec2)candidateIndex4, destinationCoord);
    float sumDist = d1 + d2 + d3 + d4;
    // Norm distances (weights)
    d1 /= (sumDist + EPSILON);
    d2 /= (sumDist + EPSILON);
    d3 /= (sumDist + EPSILON);
    d4 /= (sumDist + EPSILON);

    newU[Index1D(x, y)] =
        d1 * (0.5 * outU[Index1D(candidateIndex)] +
              0.5 * outU[Index1D(candidateIndex + glm::ivec2(1, 0))]) +
        d2 * (0.5 * outU[Index1D(candidateIndex2)] +
              0.5 * outU[Index1D(candidateIndex2 + glm::ivec2(1, 0))]) +
        d3 * (0.5 * outU[Index1D(candidateIndex3)] +
              0.5 * outU[Index1D(candidateIndex3 + glm::ivec2(1, 0))]) +
        d4 * (0.5 * outU[Index1D(candidateIndex4)] +
              0.5 * outU[Index1D(candidateIndex4 + glm::ivec2(1, 0))]);

    newV[Index1D(x, y)] =
        d1 * (0.5 * outV[Index1D(candidateIndex)] +
              0.5 * outV[Index1D(candidateIndex + glm::ivec2(0, 1))]) +
        d2 * (0.5 * outV[Index1D(candidateIndex2)] +
              0.5 * outV[Index1D(candidateIndex2 + glm::ivec2(0, 1))]) +
        d3 * (0.5 * outV[Index1D(candidateIndex3)] +
              0.5 * outV[Index1D(candidateIndex3 + glm::ivec2(0, 1))]) +
        d4 * (0.5 * outV[Index1D(candidateIndex4)] +
              0.5 * outV[Index1D(candidateIndex4 + glm::ivec2(0, 1))]);
  }

  outU = newU;
  outV = newV;
}

void Disipate(std::vector<float>& u, std::vector<float>& v) {
  for (int i=0; i<NUM_CELLS.x * NUM_CELLS.y; i++) {
    u[i] *= (1 - DISIPATION);
    v[i] *= (1 - DISIPATION);
  }
}

void InputAlternate10Seconds(std::vector<float> &u, std::vector<float> &v, float time) {
  if (time > 10) {
    return;
  }
  float speed = 0.1f;
  for (int i = 0; i < NUM_U.x * NUM_U.y; i++) {
    int x, y;
    Index2D(i, x, y);
    if (x > 0.4 * NUM_U.x && x < 0.6 * NUM_U.x &&
        std::abs(y - NUM_U.y / 2) <= 1) {
      u[i] = std::sin(time) * speed;
      v[i] = speed * 2.0 * (0.5 - uniform(gen));
    }
  }
}

void InputPulse(std::vector<float> &u, std::vector<float> &v, float time) {
  if (1.0 + std::sin(time) < 0.99) {
    return;
  }

  float speed = 0.1f;
  for (int i = 0; i < NUM_U.x * NUM_U.y; i++) {
    int x, y;
    Index2D(i, x, y);
    if (x > 0.4 * NUM_U.x && x < 0.6 * NUM_U.x &&
        std::abs(y - NUM_U.y / 2) <= 1) {
      u[i] = speed;
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
      0.0, 1.0); // Generates random floats in range [0.0, 1.0]

  // Initialize Sim
  //--------------------------------------------------------------------------------------
  std::vector<float> u(NUM_U.x * NUM_U.y, 0);
  std::vector<float> v(NUM_V.x * NUM_V.y, 0);
  std::vector<glm::vec2> vecField(NUM_CELLS.x * NUM_CELLS.y, {0, 0});

  // Visualization
  //--------------------------------------------------------------------------------------
  const glm::vec2 screenSize = glm::vec2(1920, 1080);

  SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
  InitWindow(screenSize.x, screenSize.y, "ABC");
  SetTargetFPS(144);
  ToggleFullscreen();
  rlImGuiSetup(true);

  Camera2D camera = {0};
  camera.zoom = 1.0f;
  SetTargetFPS(60);

  while (!WindowShouldClose()) {
    glm::vec2 mousePosition = {GetMousePosition().x, GetMousePosition().y};
    glm::vec2 mouseDelta = {GetMouseDelta().x, GetMouseDelta().y};

    // Simulate
    InputPulse(u, v, GetTime());
    ComputeVectorField(u, v, vecField);
    Advection(vecField, u, v);
    Disipate(u, v);
    RemoveDivergence(u, v);

    // if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
    //   fmt::println("Position: {}, {}, Delta: {}, {}", mousePosition.x,
    //   mousePosition.y, mouseDelta.x, mouseDelta.y); ApplyInput(particles,
    //   mousePosition / VIS_SCALE, mouseDelta / VIS_SCALE);
    // }

    // Draw
    BeginDrawing();
    rlImGuiBegin();
    ClearBackground({1, 0, 0, 255});
    BeginMode2D(camera);

    static int cellColorIs = 0;
    static float visSpeedMult = 500.0f;

    float visStepSizeX = screenSize.y / NUM_CELLS.x;
    float visStepSizeY = screenSize.y / NUM_CELLS.y;

    glm::vec3 red = {150, 50, 50};
    glm::vec3 blue = {50, 50, 150};
    glm::vec3 magenta = {100, 50, 100};
    glm::vec3 cyan = {50, 150, 50};

    // Background tiles
    for (int y = 0; y < NUM_CELLS.y; y++) {
      for (int x = 0; x < NUM_CELLS.x; x++) {
        glm::vec2 vel = vecField[Index1D(x, y)] * visSpeedMult;
        vel = {std::clamp(vel.x, -1.0f, 1.0f), std::clamp(vel.y, -1.0f, 1.0f)};
        float speed = glm::length(vecField[Index1D(x, y)]) * visSpeedMult;

        glm::vec2 visCoordTL = {x * visStepSizeX, y * visStepSizeY};
        glm::vec2 visCoord =
            visCoordTL + glm::vec2(visStepSizeX / 2, visStepSizeY / 2);

        float padding = 0;

        if (cellColorIs == 0) {
          glm::vec3 velocityColor =
              std::max(0.0f, glm::dot(vel, {1, 0})) * red +
              std::max(0.0f, glm::dot(-vel, {1, 0})) * blue +
              std::max(0.0f, glm::dot(vel, {0, 1})) * magenta +
              std::max(0.0f, glm::dot(-vel, {0, 1})) * cyan;

          DrawRectangle(visCoordTL.x + padding / 2, visCoordTL.y + padding / 2,
                        visStepSizeX - padding, visStepSizeY - padding,
                        {(unsigned char)velocityColor.r,
                         (unsigned char)velocityColor.g,
                         (unsigned char)velocityColor.b, 255});
        }

        if (cellColorIs == 1) {
          glm::vec3 speedColor = speed * red + (1 - speed) * blue;

          DrawRectangle(visCoordTL.x + padding / 2, visCoordTL.y + padding / 2,
                        visStepSizeX - padding, visStepSizeY - padding,
                        {(unsigned char)speedColor.r,
                         (unsigned char)speedColor.g,
                         (unsigned char)speedColor.b, 255});
        }

        if (cellColorIs == 2) {
          glm::vec3 velocityColor =
              std::max(0.0f, glm::dot(vel, {1, 0})) * red +
              std::max(0.0f, glm::dot(-vel, {1, 0})) * blue +
              std::max(0.0f, glm::dot(vel, {0, 1})) * magenta +
              std::max(0.0f, glm::dot(-vel, {0, 1})) * cyan;

          float scale = 0.5f;
          DrawLineEx({visCoord.x, visCoord.y},
                     {visCoord.x + scale * visStepSizeX * vel.x,
                      visCoord.y + scale * visStepSizeY * vel.y},
                     6.0f * scale,
                     {(unsigned char)velocityColor.r,
                      (unsigned char)velocityColor.g,
                      (unsigned char)velocityColor.b, 255});
          DrawCircle(visCoord.x, visCoord.y, 6.0f * scale,
                     {(unsigned char)velocityColor.r,
                      (unsigned char)velocityColor.g,
                      (unsigned char)velocityColor.b, 255});
        }
      }
    }

    // draw the debug panel
    ImGui::Begin("Parameters");
    ImGui::Text("%s", fmt::format("FPS: {}", GetFPS()).c_str());
    ImGui::DragFloat("Vis. Speed Multiplier", &visSpeedMult);
    ImGui::Text("Visualization type:");
    ImGui::SameLine();
    ImGui::RadioButton("Velocity", &cellColorIs, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Speed", &cellColorIs, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Arrows", &cellColorIs, 2);
    ImGui::End();

    EndMode2D();
    rlImGuiEnd();
    EndDrawing();
  }

  return 0;
}
