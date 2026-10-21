#include "AnimationController.h"
#include <iostream>
#include <glm/glm.hpp>


AnimationController::AnimationController(Mesh& mesh, PathFollower& follower,
    float walkSpeed, float runSpeed, float blendTime)
    : characterMesh(mesh), pathFollower(follower),
    currentPhase(EASE_IN), previousPhase(EASE_IN),
    blendDuration(blendTime), debugOutput(true),
    walkAnimSpeed(walkSpeed), runAnimSpeed(runSpeed)
{
}

AnimationController::SpeedPhase AnimationController::determineCurrentPhase() {
    float currentTime = pathFollower.getCurrentTime();
    float easeInEnd = pathFollower.getEaseInDuration();
    float easeOutStart = pathFollower.getTotalDuration() - pathFollower.getEaseOutDuration();

    if (currentTime < easeInEnd) {
        return EASE_IN;
    }
    else if (currentTime < easeOutStart) {
        return CONSTANT;
    }
    else {
        return EASE_OUT;
    }
}

void AnimationController::handlePhaseTransition() {
    switch (currentPhase) {
    case EASE_IN:
        if (debugOutput) {
            std::cout << "Phase: EASE_IN (Accelerating) - Blending to WALK" << std::endl;
        }
        characterMesh.BlendToAnimation("walk", blendDuration);
        break;

    case CONSTANT:
        if (debugOutput) {
            std::cout << "Phase: CONSTANT (Max Speed) - Blending to RUN" << std::endl;
        }
        characterMesh.BlendToAnimation("run", blendDuration);
        break;

    case EASE_OUT:
        if (debugOutput) {
            std::cout << "Phase: EASE_OUT (Decelerating) - Blending to WALK" << std::endl;
        }
        characterMesh.BlendToAnimation("walk", blendDuration);
        break;
    }
}

void AnimationController::updateAnimationSpeed(float currentSpeed) {
    
    float animSpeedMultiplier;
	float referenceSpeed = (currentPhase == CONSTANT) ? runAnimSpeed : walkAnimSpeed;

    if (referenceSpeed > 0.0f) {
        animSpeedMultiplier = currentSpeed / referenceSpeed;
    }
    else {
        animSpeedMultiplier = 1.0f;  // fallback only
    }

    animSpeedMultiplier = glm::clamp(animSpeedMultiplier, 0.01f, 5.0f);

    
    //const AnimationClip* currentClip = characterMesh.GetCurrentClip();
    //if (!currentClip) return;

    //float animDuration = characterMesh.GetAnimationDuration(
    //    currentPhase == CONSTANT ? "run" : "walk"
    //);

    //if (currentPhase == CONSTANT) {
    //    // During constant speed phase (running)
    //    animSpeedMultiplier = currentSpeed / runAnimSpeed;
    //}
    //else {
    //    // During ease-in or ease-out (walking)
    //    animSpeedMultiplier = currentSpeed / walkAnimSpeed;
    //}

    //// Clamp to reasonable values to avoid extreme speeds
    //animSpeedMultiplier = glm::clamp(animSpeedMultiplier, 0.1f, 3.0f);
    //

    //characterMesh.SetAnimationSpeed(animSpeedMultiplier);

    if (debugOutput) {
        static int frameCount = 0;
        if (frameCount % 60 == 0) {  // Print every 60 frames
            std::cout << "Movement Speed: " << currentSpeed
                << " | Anim Speed Multiplier: " << animSpeedMultiplier << "x" << std::endl;
        }
        frameCount++;
    }
}


void AnimationController::update(float deltaTime) {
    // Get current path state
    PathFollower::PathState state = pathFollower.update(deltaTime);

    // Determine current phase
    currentPhase = determineCurrentPhase();

    // Handle phase transitions with smooth blending
    if (currentPhase != previousPhase) {
        handlePhaseTransition();
        previousPhase = currentPhase;
    }

    // Update animation playback speed to match movement speed
    updateAnimationSpeed(state.speed);

    // Draw the character with speed-adjusted animation
    characterMesh.Draw(shaderProgram, deltaTime, pathFollower);

}

void AnimationController::SetMeshPrgId(int Id)
{
    shaderProgram = Id;
}
















