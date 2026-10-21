#pragma once
#include "Mesh.h"
#include "PathFollower.h"
#include <glm/glm.hpp>

class AnimationController
{
public:
    enum SpeedPhase {
        EASE_IN,      // Accelerating - play walk
        CONSTANT,     // Max speed - play run
        EASE_OUT      // Decelerating - play walk
    };


    AnimationController(Mesh& mesh, PathFollower& follower,
        float walkSpeed = 2.0f, float runSpeed = 5.0f,
        float blendTime = 0.3f);

    void update(float deltaTime);

    // Setters
    void setBlendDuration(float duration) { blendDuration = duration; }
    void setDebugOutput(bool enabled) { debugOutput = enabled; }
    void setWalkSpeed(float speed) { walkAnimSpeed = speed; }
    void setRunSpeed(float speed) { runAnimSpeed = speed; }

    // Getters
    SpeedPhase getCurrentPhase() const { return currentPhase; }

    void SetMeshPrgId(int Id);

private:
    int shaderProgram;

    Mesh& characterMesh;
    PathFollower& pathFollower;

    SpeedPhase currentPhase;
    SpeedPhase previousPhase;
    float blendDuration;

    // Animation speed calibration
    float walkAnimSpeed;   // Units per second at normal walk animation speed
    float runAnimSpeed;    // Units per second at normal run animation speed

    bool debugOutput;

    bool isMoving = false;

    void handlePhaseTransition();
    void updateAnimationSpeed(float currentSpeed);
    SpeedPhase determineCurrentPhase();

};

