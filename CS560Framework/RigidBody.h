#pragma once
#include <glm/glm.hpp>
#include <vector>


struct RigidBody
{
    // Mass properties
    float M = 1.0f;
    glm::mat3 I_body = glm::mat3(1.0f);
    glm::mat3 I_body_inv = glm::mat3(1.0f);
    glm::mat3 I_world_inv = glm::mat3(1.0f);

    // State (world)
    glm::vec3 x = glm::vec3(0.0f);     // position (COM)
    glm::mat3 R = glm::mat3(1.0f);     // rotation matrix
    glm::vec3 P = glm::vec3(0.0f);     // linear momentum
    glm::vec3 L = glm::vec3(0.0f);     // angular momentum

    // Derived
    glm::vec3 v = glm::vec3(0.0f);     // linear velocity
    glm::vec3 w = glm::vec3(0.0f);     // angular velocity

    // Local (body) points (e.g., 64 lattice points)
    std::vector<glm::vec3> v0;

    glm::vec3 forceAccum = glm::vec3(0.0f);
    glm::vec3 torqueAccum = glm::vec3(0.0f);

    void clearAccumulators();
    void applyForceAtPoint(const glm::vec3& F, const glm::vec3& worldPoint);


    // Helpers
    void buildLattice4x4x4(float sideLength);
    void computeMassAndInertiaFromPoints(float totalMass);
    glm::vec3 pointWorld(int i) const;

    // Core update (for now: forces/torques are applied externally in later layers)
    void updateDerived();
    void integrateStateSymplecticEuler(float dt); // updates x, R using current v,w
    void reOrthonormalizeR();                     // prevent drift
};

glm::mat3 skew(const glm::vec3& w);

