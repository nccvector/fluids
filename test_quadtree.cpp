#include <algorithm>
#include <chrono>
#include <fmt/base.h>
#include <fmt/core.h>
#include <random>
#include <cmath> // for std::isnan
#include <iostream>
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

// Create a random engine
std::random_device rd; // Seed with a random value from hardware (if available)
std::mt19937 gen; // Mersenne Twister engine for randomness

// Define a uniform distribution in the desired range (e.g., 0.0 to 1.0)
std::uniform_real_distribution<>
uniform(0.0, 1.0); // Generates random floats in range [0.0, 1.0]

const int NUM_ELEMENTS = 100000;

int main() {
    // Visualization
    //--------------------------------------------------------------------------------------
    const glm::vec2 screenSize = glm::vec2(1920, 1080);

    InitWindow(screenSize[0], screenSize[1], "ABC");
    ToggleFullscreen();
    rlImGuiSetup(true);

    Camera2D camera = {0};
    camera.zoom = 1.0f;
    SetTargetFPS(60);

    // Create randon elements
    std::vector<Element> elements(NUM_ELEMENTS, Element());
    for (auto &element: elements) {
        element.aabb = {glm::vec2(uniform(gen), uniform(gen)) * screenSize, glm::vec2(10, 10)};
    }

    // Create tree
    Quadtree tree;
    tree.aabb = {glm::vec2(0, 0), screenSize};

    // Build tree
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (auto &element: elements) {
        Insert(&tree, element);
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    long buildTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    int count = 0;

    while (!WindowShouldClose()) {
        glm::vec2 mousePosition = {GetMousePosition().x, GetMousePosition().y};
        glm::vec2 mouseDelta = {GetMouseDelta().x, GetMouseDelta().y};

        // Draw
        BeginDrawing();
        rlImGuiBegin();
        ClearBackground({1, 0, 0, 255});
        BeginMode2D(camera);


        DrawCircleLinesV(
            Vector2{mousePosition.x, mousePosition.y},
            100, WHITE
        );

        DrawQuadtree(&tree, glm::vec2(1.0f));

        std::list<Element> elementsInRadius;

        begin = std::chrono::steady_clock::now();
        GetElementsInRadius(mousePosition, 100, elementsInRadius, &tree);
        end = std::chrono::steady_clock::now();
        long queryTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        for (auto element: elementsInRadius) {
            DrawRectangleLines(
                element.aabb.minCoord.x,
                element.aabb.minCoord.y,
                element.aabb.size.x,
                element.aabb.size.y,
                {255, 55, 55, 255} // Color of the rectangle's outline
            );
        }

        // draw the debug panel
        ImGui::Begin("Parameters");
        ImGui::Text("%s", fmt::format("FPS (includes drawing time): {}", GetFPS()).c_str());
        ImGui::Text("%s", fmt::format("Object count: {}", elements.size()).c_str());
        ImGui::Text("%s", fmt::format("Build time: {}ms", buildTime).c_str());
        ImGui::Text("%s", fmt::format("Query time: {}us", queryTime).c_str());
        ImGui::Separator();
        ImGui::InputInt("Max elements per node: ", &MAX_PARTICLES_PER_NODE);
        ImGui::SameLine();
        if (ImGui::Button("REBUILD")) {
            // Rebuild tree
            tree = Quadtree{};
            tree.aabb = {glm::vec2(0, 0), screenSize};
            begin = std::chrono::steady_clock::now();
            for (auto &element: elements) {
                Insert(&tree, element);
            }
            end = std::chrono::steady_clock::now();
            buildTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        }
        ImGui::End();

        EndMode2D();
        rlImGuiEnd();
        EndDrawing();
    }

    return 0;
}
