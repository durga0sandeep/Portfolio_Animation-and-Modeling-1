#pragma once
#include <glm/glm.hpp>

/**
 * @brief Type of spring endpoint.
 */
enum class SpringEndType
{
    RigidPoint,   // index into cube mass points
    Anchor        // fixed world point
};

/**
 * @brief Represents one end of a spring.
 */
struct SpringEnd
{
    SpringEndType type;
    int index;            // rigid point index (ignored for anchor)
    glm::vec3 worldPos;   // used only if Anchor
};

/**
 * @brief Represents a spring connecting two endpoints.
 */
struct Spring
{
    SpringEnd a;
    SpringEnd b;

    float k;   // stiffness
    float c;   // damping
    float L0;  // rest length
};
