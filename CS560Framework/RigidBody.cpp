#include "RigidBody.h"

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>

/**
 * @brief Constructs a skew-symmetric matrix from a vector.
 *
 * For a vector w = (wx, wy, wz), returns the 3x3 matrix [w]x such that
 * [w]x * v = w × v.
 *
 * @param w The input 3D vector
 * @return glm::mat3 Skew-symmetric matrix
 */
glm::mat3 skew(const glm::vec3& w)
{
    // Column-major construction (each vec3 is a column)
    // [  0   -wz   wy
    //    wz   0   -wx
    //   -wy  wx    0 ]
    return glm::mat3(
        glm::vec3(0.0f, w.z, -w.y),  // col 0
        glm::vec3(-w.z, 0.0f, w.x),  // col 1
        glm::vec3(w.y, -w.x, 0.0f)   // col 2
    );
}



/**
 * @brief Builds a 4x4x4 lattice of mass points within a cube.
 *
 * Points are positioned in [-s/2, s/2] along each axis.
 * @param sideLength Length of the cube side
 */
void RigidBody::buildLattice4x4x4(float sideLength)
{
    v0.clear();
    v0.reserve(64);

    // 4 points along each axis in [-s/2, s/2]
    const float s = sideLength;
    const float coords[4] = {
        -0.5f * s,
        -0.16666667f * s,
         0.16666667f * s,
         0.5f * s
    };

    for (int k = 0; k < 4; ++k)
        for (int j = 0; j < 4; ++j)
            for (int i = 0; i < 4; ++i)
                v0.push_back(glm::vec3(coords[i], coords[j], coords[k]));
}


/**
 * @brief Computes total mass and inertia tensor from lattice points.
 *
 * The inertia tensor is computed about the center of mass in the body frame.
 * A small diagonal regularization is applied to avoid singularities.
 * @param totalMass Desired total mass of the rigid body
 */
void RigidBody::computeMassAndInertiaFromPoints(float totalMass)
{
    M = std::max(1e-6f, totalMass);

    const int n = (int)v0.size();
    if (n == 0)
    {
        I_body = glm::mat3(1.0f);
        I_body_inv = glm::mat3(1.0f);
        return;
    }

    const float mi = M / float(n);

    // Compute inertia about COM in body frame (points are already centered if lattice is symmetric)
    glm::mat3 I(0.0f);
    for (const auto& r : v0)
    {
        const float rr = glm::dot(r, r);
        // (r·r)I - r r^T
        glm::mat3 term(0.0f);
        term[0][0] = rr; term[1][1] = rr; term[2][2] = rr;
        term -= glm::outerProduct(r, r);
        I += mi * term;
    }

    I_body = I;

    // Invert safely
    // (Assuming GLM inverse exists; if singular, add a tiny diagonal regularization.)
    glm::mat3 Ireg = I_body;
    const float eps = 1e-8f;
    Ireg[0][0] += eps; Ireg[1][1] += eps; Ireg[2][2] += eps;
    I_body_inv = glm::inverse(Ireg);
}

/**
 * @brief Gets the world position of the i-th lattice point.
 * @param i Index of the lattice point
 * @return glm::vec3 Position in world coordinates
 */
glm::vec3 RigidBody::pointWorld(int i) const
{
    return x + R * v0[i];
}

/**
 * @brief Updates derived quantities: linear velocity, angular velocity, and world-space inverse inertia.
 */
void RigidBody::updateDerived()
{
    v = P / M;
    I_world_inv = R * I_body_inv * glm::transpose(R);
    w = I_world_inv * L;
}


/**
 * @brief Integrates the rigid body's state using Symplectic Euler.
 *
 * Updates position x and rotation matrix R based on current velocities.
 * Assumes linear momentum P and angular momentum L have already been updated from forces/torques.
 * @param dt Time step
 */
void RigidBody::integrateStateSymplecticEuler(float dt)
{
    // Assumes P and L were already updated from forces/torques (next layers).
    updateDerived();

    // Step 6 in flowchart:
    x += dt * v;

    // dR/dt = [w]x R  (Step 5), integrate with Euler:
    R += dt * (skew(w) * R);

    reOrthonormalizeR();
    // Step 7 (I_world_inv) will be updated next time updateDerived() is called.
}

/**
 * @brief Re-orthonormalizes the rotation matrix R.
 *
 * Uses Gram-Schmidt on the columns to maintain a proper rotation matrix.
 */
void RigidBody::reOrthonormalizeR()
{
    // Gram-Schmidt on columns (GLM is column-major)
    glm::vec3 c0 = glm::normalize(R[0]);
    glm::vec3 c1 = R[1] - glm::dot(R[1], c0) * c0;
    c1 = glm::normalize(c1);
    glm::vec3 c2 = glm::cross(c0, c1);

    R[0] = c0;
    R[1] = c1;
    R[2] = c2;
}

/**
 * @brief Clears the accumulated force and torque.
 */
void RigidBody::clearAccumulators()
{
    forceAccum = glm::vec3(0.0f);
    torqueAccum = glm::vec3(0.0f);
}

/**
 * @brief Applies a force at a given point in world coordinates.
 *
 * Updates the accumulated force and torque on the rigid body.
 * @param F Force vector in world coordinates
 * @param worldPoint Point of application in world coordinates
 */
void RigidBody::applyForceAtPoint(const glm::vec3& F, const glm::vec3& worldPoint)
{
    forceAccum += F;
    torqueAccum += glm::cross(worldPoint - x, F);
}
