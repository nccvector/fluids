//
// Created by vector on 1/28/25.
//

#include <algorithm>
#include <cmath> // for std::isnan
#include <list>

#include "glm/geometric.hpp"
#include "glm/vec2.hpp"
#include "raylib.h"

// int MAX_PARTICLES_PER_NODE = 32;  // chosen using test_quadtree analysis
int MAX_PARTICLES_PER_NODE = 4;  // chosen using test_quadtree analysis

struct AABB {
    glm::vec2 minCoord;
    glm::vec2 size;
};

struct Element {
    int id;
    AABB aabb;
};

enum Cardinal {
    TOP_LEFT = 0,
    TOP_RIGHT = 1,
    BOTTOM_LEFT = 2,
    BOTTOM_RIGHT = 3,
};

struct Quadtree {
    AABB aabb;
    std::list<Element> elements;
    Quadtree *children[4] = {nullptr, nullptr, nullptr, nullptr};
};

// Function to subdivide the current node
void Subdivide(Quadtree *tree) {
    glm::vec2 halfSize = tree->aabb.size * 0.5f;

    tree->children[TOP_LEFT] = new Quadtree{
        .aabb = {tree->aabb.minCoord, halfSize}, .children = {nullptr, nullptr, nullptr, nullptr}
    };
    tree->children[TOP_RIGHT] = new Quadtree{
        .aabb = {tree->aabb.minCoord + glm::vec2(halfSize.x, 0), halfSize},
        .children = {nullptr, nullptr, nullptr, nullptr}
    };
    tree->children[BOTTOM_LEFT] = new Quadtree{
        .aabb = {tree->aabb.minCoord + glm::vec2(0, halfSize.y), halfSize},
        .children = {nullptr, nullptr, nullptr, nullptr}
    };
    tree->children[BOTTOM_RIGHT] = new Quadtree{
        .aabb = {tree->aabb.minCoord + glm::vec2(halfSize.x, halfSize.y), halfSize},
        .children = {nullptr, nullptr, nullptr, nullptr}
    };
}

// Function to check if a point is within an AABB
bool CheckPointInsideAABB(const AABB &box, const glm::vec2 &point) {
    return point.x >= box.minCoord.x &&
           point.x <= box.minCoord.x + box.size.x &&
           point.y >= box.minCoord.y &&
           point.y <= box.minCoord.y + box.size.y;
}

bool TestAABBOverlap(const AABB &box1, const AABB &box2) {
    return (box1.minCoord.x < (box2.minCoord.x + box2.size.x) &&
            (box1.minCoord.x + box1.size.x) > box2.minCoord.x &&
            box1.minCoord.y < (box2.minCoord.y + box2.size.y) &&
            (box1.minCoord.y + box1.size.y) > box2.minCoord.y);
}


bool TestAABBCircleOverlap(const AABB &box, const glm::vec2 &circlePos, float circleRadius) {
    // Find the closest point on the AABB to the circle's center
    glm::vec2 closestPoint = {
        std::max(box.minCoord.x, std::min(circlePos.x, box.minCoord.x + box.size.x)),
        std::max(box.minCoord.y, std::min(circlePos.y, box.minCoord.y + box.size.y))
    };

    // Calculate the distance between the circle's center and this closest point
    float distanceToClosestPoint = glm::length(circlePos - closestPoint);

    // Check if the distance is less than or equal to the radius
    return distanceToClosestPoint <= circleRadius;
}

bool Insert(Quadtree *tree, const Element &element) {
    // If the particle is outside the bounds of this node, return false
    if (!TestAABBOverlap(tree->aabb, element.aabb)) {
        return false;
    }

    // If there's space in this node and it has no children, add the particle here
    if (tree->elements.size() < MAX_PARTICLES_PER_NODE && tree->children[0] == nullptr) {
        tree->elements.push_back(element);
        return true;
    }

    // If the node is at capacity, subdivide if not already subdivided
    if (tree->children[0] == nullptr) {
        Subdivide(tree);
    }

    // Try inserting the particle into child nodes
    for (int i = 0; i < 4; i++) {
        if (Insert(tree->children[i], element)) {
            return true;
        }
    }

    // If we couldn't insert the particle for some reason, return false
    return false;
}

void GetElementsInRadius(const glm::vec2 &position, float radius, std::list<Element> &result, Quadtree *node) {
    // If the node is null, return immediately
    if (node == nullptr) {
        return;
    }

    if (!TestAABBCircleOverlap(node->aabb, position, radius)) return;

    // Check particles in this node
    for (auto &particle: node->elements) {
        if (!TestAABBCircleOverlap(particle.aabb, position, radius)) continue;
        result.push_back(particle);
    }

    // Recursively check child nodes
    for (int i = 0; i < 4; i++) {
        GetElementsInRadius(position, radius, result, node->children[i]);
    }
}

void DrawQuadtree(Quadtree *node, const glm::vec2 &scale) {
    if (node == nullptr) {
        return;
    }

    // Draw the current node's AABB as a rectangle
    DrawRectangleLines(
        node->aabb.minCoord.x * scale.x,
        node->aabb.minCoord.y * scale.y,
        node->aabb.size.x * scale.x,
        node->aabb.size.y * scale.y,
        {255, 255, 255, 55} // Color of the rectangle's outline
    );

    // Draw elements in this node
    for (auto &element: node->elements) {
        // Draw the current node's AABB as a rectangle
        DrawRectangleLines(
            element.aabb.minCoord.x * scale.x,
            element.aabb.minCoord.y * scale.y,
            element.aabb.size.x * scale.x,
            element.aabb.size.y * scale.y,
            {55, 255, 55, 55} // Color of the rectangle's outline
        );
    }

    // Recursively draw children
    for (int i = 0; i < 4; i++) {
        DrawQuadtree(node->children[i], scale);
    }
}
