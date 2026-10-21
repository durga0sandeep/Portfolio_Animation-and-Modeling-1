#include "AnimationController.h"
#include <iostream>
#include <glm/glm.hpp>


AnimationController::AnimationController(Mesh& mesh, PathFollower& follower,
    float walkSpeed, float runSpeed, float blendTime)
    : characterMesh(mesh), pathFollower(follower),
    currentPhase(CONSTANT), previousPhase(CONSTANT),
    blendDuration(blendTime), debugOutput(true),
	walkAnimSpeed(walkSpeed), runAnimSpeed(runSpeed), shaderProgram(0), isMoving(false)
{

    characterMesh.BlendToAnimation("idle", 0.0f);
    characterMesh.SetAnimationSpeed(1.0f);

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

    characterMesh.SetAnimationSpeed(1.0f);
}

void AnimationController::updateAnimationSpeed(float currentSpeed) {
    
    /*
    float animSpeedMultiplier;
	float referenceSpeed = (currentPhase == CONSTANT) ? runAnimSpeed : walkAnimSpeed;

    if (referenceSpeed > 0.0f) {
        animSpeedMultiplier = currentSpeed / referenceSpeed;
    }
    else {
        animSpeedMultiplier = 1.0f;  // fallback only
    }

    animSpeedMultiplier = glm::clamp(animSpeedMultiplier, 0.01f, 5.0f);
    characterMesh.SetAnimationSpeed(animSpeedMultiplier);
    */

    const float desiredRunWorldSpeed = runAnimSpeed;  // use your variable or a constant you tuned

    float m = currentSpeed / desiredRunWorldSpeed;

    m = glm::clamp(m, 0.85f, 1.25f);

    characterMesh.SetAnimationSpeed(m);

    /*
    if (debugOutput) {
        static int frameCount = 0;
        if (frameCount % 60 == 0) {  // Print every 60 frames
            std::cout << "Movement Speed: " << currentSpeed
                << " | Anim Speed Multiplier: " << animSpeedMultiplier << "x" << std::endl;
        }
        frameCount++;
    }
    */

}


void AnimationController::update(float deltaTime) {
    PathFollower::PathState state = pathFollower.update(deltaTime);

    const float t = pathFollower.getCurrentTime();
    const float T = pathFollower.getTotalDuration();
    //const bool finished = (t >= T - 1e-5f) || (state.normalizedArcLength >= 0.9999f);
    const bool finished = (t >= T - deltaTime) || (state.normalizedArcLength >= 0.9999f);
    
    
    if (finished)
    {
        if (isMoving)
        {
            characterMesh.BlendToAnimation("idle", 0.05f); // or 0.0f if you want an instant cut
            characterMesh.SetAnimationSpeed(1.0f);
            isMoving = false;

            if (debugOutput) std::cout << "IDLE (forced at path end)\n";
        }

        characterMesh.Draw(shaderProgram, deltaTime, pathFollower);
        return;
    }

    const float RUN_ENTER = 0.04f;
    const float RUN_EXIT = 0.07f;

    //std::cout << "DEBUGGING THE SPEED  " << state.speed << " +++ " << std::endl;

    if (!isMoving)
    {
        // Enter run only if clearly moving
        if (state.speed > RUN_ENTER)
        {
            characterMesh.BlendToAnimation("run", blendDuration);
            isMoving = true;
        }
    }
    else
    {
        // Exit run only if clearly slow
        if (state.speed < RUN_EXIT)
        {
            characterMesh.BlendToAnimation("idle", blendDuration);
            characterMesh.SetAnimationSpeed(1.0f);
            isMoving = false;
        }
        else
        {
            // STEP 3 — animation speed while running
            updateAnimationSpeed(state.speed);
        }
    }

    characterMesh.Draw(shaderProgram, deltaTime, pathFollower);


}

void AnimationController::SetMeshPrgId(int Id)
{
    shaderProgram = Id;
}
















