#pragma once
#include "PathGenerator.h"



class PathFollower {
public:
    enum OrientationMode {
        TANGENT_FORWARD,           // Face direction of motion
        TANGENT_WITH_BANKING,      // Face direction + bank into turns
        LOOK_AHEAD_ON_PATH,        // Look at point ahead on path (RECOMMENDED)
        LOOK_AHEAD_WITH_BANKING,   // Look ahead + banking (BEST FOR CURVED PATHS)
        FACE_CENTER,               // Always face path center
        FACE_POINT_OF_INTEREST     // Face external point
    };

    struct PathState {
        glm::vec3 position;
        glm::vec3 forward;
        glm::vec3 up;
        glm::vec3 right;
        float speed;
        int currentSegment;
        float normalizedArcLength;
    };

private:
    PathGenerator* pathGen;

    // Animation timing
    float totalDuration;       // Total time for one loop
    float easeInDuration;      // Time for ease-in
    float easeOutDuration;     // Time for ease-out
    float maxSpeed;            // Maximum speed (constant section)

    // Current state
    float currentTime;
    bool isAnimating;
    bool loopAnimation;

    // Orientation
    OrientationMode orientationMode;
    glm::vec3 centerOfInterest;
    float lookAheadDistance;   // For look-ahead mode
    float bankingFactor;       // For banking mode

    // Previous frame data for smooth derivatives
    glm::vec3 prevPosition;
    glm::vec3 prevTangent;

public:
    PathFollower(PathGenerator* pg, float duration = 10.0f);

    // Core update function
    PathState update(float deltaTime);

    // Ease-in/ease-out distance function
    float easeInOutDistance(float t);

    // Get position and tangent at global arc length
    void getPositionAndTangent(float globalArcLength, glm::vec3& pos, glm::vec3& tangent);

    // Orientation calculation
    glm::mat4 calculateOrientation(const glm::vec3& position,
        const glm::vec3& tangent,
        const glm::vec3& prevTangent);

    // Setters
    void setDuration(float duration, float easeInRatio = 0.2f, float easeOutRatio = 0.2f);
    void setOrientationMode(OrientationMode mode);
    void setCenterOfInterest(const glm::vec3& center);
    void setLookAheadDistance(float distance);
    void setBankingFactor(float factor);
    void setLooping(bool loop);

    // Animation control
    void start();
    void pause();
    void reset();

    // Getters
    //float getCurrentTime() const { return currentTime; }
    //float getTotalDuration() const { return totalDuration; }
    bool isPlaying() const { return isAnimating; }


    //N-NEW
    float getCurrentTime() const { return currentTime; }
    float getTotalDuration() const { return totalDuration; }
    float getEaseInDuration() const { return easeInDuration; }
    float getEaseOutDuration() const { return easeOutDuration; }
    float getMaxSpeed() const { return maxSpeed; }
    bool getIsAnimating() const { return isAnimating; }
    PathState getCurrentState();
};

