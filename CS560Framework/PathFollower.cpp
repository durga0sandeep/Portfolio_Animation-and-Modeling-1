#include "PathFollower.h"
#include <gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

/**
 * @brief Construct a new PathFollower object.
 *
 * Initializes the follower with a given PathGenerator, total duration,
 * default orientation mode, and initial state.
 *
 * @param pg Pointer to the PathGenerator object representing the path.
 * @param duration Total time to traverse the path (seconds).
 */
PathFollower::PathFollower(PathGenerator* pg, float duration)
    : pathGen(pg), totalDuration(duration), currentTime(0.0f),
    isAnimating(false),
    loopAnimation(false), // 1. Set looping to false
    orientationMode(TANGENT_FORWARD), // 2. Simple forward orientation
    centerOfInterest(0.0f, 0.0f, 0.0f),
    lookAheadDistance(1.0f), bankingFactor(0.0f), // 3. Remove banking
    prevPosition(0.0f), prevTangent(1.0f, 0.0f, 0.0f)
{
    setDuration(duration, 0.2f, 0.2f);
}

/**
 * @brief Set the total duration of the animation and ease-in/ease-out ratios.
 *
 * Calculates the maximum speed for the follower based on a trapezoidal velocity profile.
 *
 * @param duration Total duration of the animation (seconds).
 * @param easeInRatio Fraction of duration for ease-in phase [0,1].
 * @param easeOutRatio Fraction of duration for ease-out phase [0,1].
 */
void PathFollower::setDuration(float duration, float easeInRatio, float easeOutRatio) {
    totalDuration = duration;
    easeInDuration = duration * easeInRatio;
    easeOutDuration = duration * easeOutRatio;

    // Calculate max speed based on total distance = 1.0 (normalized)
    // Distance = easeInDist + constantDist + easeOutDist = 1.0
    // For ease-in: d = 0.5 * a * t^2, with final velocity v = a*t
    // For constant: d = v * t
    // For ease-out: d = v*t - 0.5*a*t^2

    float constantDuration = totalDuration - easeInDuration - easeOutDuration;

    // Using trapezoidal velocity profile
    // Total distance = 1.0 = 0.5*v*easeIn + v*constant + 0.5*v*easeOut
    // 1.0 = v * (0.5*easeIn + constant + 0.5*easeOut)
    maxSpeed = 1.0f / (0.5f * easeInDuration + constantDuration + 0.5f * easeOutDuration);
}

/**
 * @brief Compute the normalized distance along the path for a given time.
 *
 * Implements ease-in (quadratic acceleration), constant speed, and ease-out (quadratic deceleration).
 *
 * @param t Current time along the path [0, totalDuration].
 * @return float Normalized distance along the path [0, 1].
 */
float PathFollower::easeInOutDistance(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= totalDuration) return 1.0f;

    float distance = 0.0f;

    // Phase 1: Ease-in (quadratic acceleration)
    if (t < easeInDuration) {
        float ratio = t / easeInDuration;
        distance = 0.5f * maxSpeed * easeInDuration * ratio * ratio;
    }
    // Phase 2: Constant speed
    else if (t < totalDuration - easeOutDuration) {
        float easeInDistance = 0.5f * maxSpeed * easeInDuration;
        float constantTime = t - easeInDuration;
        distance = easeInDistance + maxSpeed * constantTime;
    }
    // Phase 3: Ease-out (quadratic deceleration)
    else {
        float easeInDistance = 0.5f * maxSpeed * easeInDuration;
        float constantDuration = totalDuration - easeInDuration - easeOutDuration;
        float constantDistance = maxSpeed * constantDuration;

        float easeOutTime = t - (totalDuration - easeOutDuration);
        float ratio = easeOutTime / easeOutDuration;
        float easeOutDistance = maxSpeed * easeOutDuration * (ratio - 0.5f * ratio * ratio);

        distance = easeInDistance + constantDistance + easeOutDistance;
    }

    return distance;
}

/**
 * @brief Get the position and tangent vector at a specified normalized arc length.
 *
 * @param globalArcLength Normalized arc length along the entire path [0,1].
 * @param[out] pos Output world-space position on the path.
 * @param[out] tangent Output tangent vector (normalized) at the position.
 */
void PathFollower::getPositionAndTangent(float globalArcLength,
    glm::vec3& pos,
    glm::vec3& tangent) {
    int numSegments = pathGen->getNumPoints() / 3;

    // Change: Clamp instead of Wrap
    // globalArcLength = fmod(globalArcLength, 1.0f); <-- Remove this
    globalArcLength = glm::clamp(globalArcLength, 0.0f, 0.999f); // Keep within [0, 1]

    int segment = (int)(globalArcLength * numSegments);
    segment = glm::clamp(segment, 0, numSegments - 1);

    float localArcLength = (globalArcLength * numSegments) - segment;
    float t = pathGen->parameterFromArcLength(segment, localArcLength);

    // ... rest of the function remains the same to calculate pos and tangent ...
    int i0 = (segment * 3);
    int i1 = (segment * 3 + 1);
    int i2 = (segment * 3 + 2);
    int i3 = (segment * 3 + 3) % pathGen->getNumPoints();

    glm::vec3 p0 = pathGen->getControlPoint(i0);
    glm::vec3 p1 = pathGen->getControlPoint(i1);
    glm::vec3 p2 = pathGen->getControlPoint(i2);
    glm::vec3 p3 = pathGen->getControlPoint(i3);

    pos = pathGen->evaluateCubicBezier(p0, p1, p2, p3, t);

    // Simple derivative for tangent
    float u = 1.0f - t;
    tangent = 3.0f * u * u * (p1 - p0) + 6.0f * u * t * (p2 - p1) + 3.0f * t * t * (p3 - p2);
    
    if (glm::length(tangent) > 0.001f) tangent = glm::normalize(tangent);

}

/**
 * @brief Calculate the orientation matrix at a given position based on mode.
 *
 * @param position World-space position of the follower.
 * @param tangent Tangent vector along the path.
 * @param prevTangent Tangent vector from the previous frame (used for banking).
 * @return glm::mat4 Orientation matrix (right, up, forward, position).
 */
glm::mat4 PathFollower::calculateOrientation(const glm::vec3& position,
    const glm::vec3& tangent,
    const glm::vec3& prevTangent) {
    glm::vec3 forward = tangent;
    glm::vec3 up = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 right;

    switch (orientationMode) {
    case TANGENT_FORWARD: {
        // Simple tangent following
        right = glm::normalize(glm::cross(forward, up));
        up = glm::cross(right, forward);
        break;
    }

    case TANGENT_WITH_BANKING: {
        // Add banking based on curvature
        right = glm::normalize(glm::cross(forward, up));

        // Calculate curvature from tangent change
        glm::vec3 tangentDelta = tangent - prevTangent;
        float curvature = glm::length(tangentDelta);

        // Determine banking direction
        glm::vec3 curvatureDir = glm::cross(prevTangent, tangent);
        float bankingAngle = curvature * bankingFactor;

        if (glm::dot(curvatureDir, up) < 0.0f) {
            bankingAngle = -bankingAngle;
        }

        // Apply banking rotation
        glm::mat4 bankingRot = glm::rotate(glm::mat4(1.0f), bankingAngle,glm::normalize(forward));
        up = glm::vec3(bankingRot * glm::vec4(up, 0.0f));
        right = glm::normalize(glm::cross(forward, up));
        up = glm::cross(right, forward);
        break;
    }

    case LOOK_AHEAD_ON_PATH: {
        // Look at a point ahead on the path curve
        float currentArcLength = easeInOutDistance(currentTime);

        // Calculate look-ahead distance as percentage of total path
        float lookAheadPercent = lookAheadDistance / 100.0f;
        float lookAheadArc = currentArcLength + lookAheadPercent;

        // Get the look-ahead position on the path
        glm::vec3 lookAheadPos, lookAheadTangent;
        getPositionAndTangent(lookAheadArc, lookAheadPos, lookAheadTangent);

        // Face toward the look-ahead point
        forward = lookAheadPos - position;
        if (glm::length(forward) > 0.001f) {
            forward = glm::normalize(forward);
        }
        else {
            forward = tangent; // Fallback to tangent if too close
        }

        right = glm::normalize(glm::cross(forward, up));
        up = glm::cross(right, forward);
        break;
    }

    case LOOK_AHEAD_WITH_BANKING: {
        // Combine look-ahead with banking for smooth curved motion
        float currentArcLength = easeInOutDistance(currentTime);

        // Calculate look-ahead distance
        float lookAheadPercent = lookAheadDistance / 100.0f;
        float lookAheadArc = currentArcLength + lookAheadPercent;

        // Get the look-ahead position
        glm::vec3 lookAheadPos, lookAheadTangent;
        getPositionAndTangent(lookAheadArc, lookAheadPos, lookAheadTangent);

        // Face toward the look-ahead point
        forward = lookAheadPos - position;
        if (glm::length(forward) > 0.001f) {
            forward = glm::normalize(forward);
        }
        else {
            forward = tangent;
        }

        // Calculate banking based on how the forward direction is changing
        glm::vec3 prevForward = prevPosition;
        if (glm::length(position - prevPosition) > 0.001f) {
            prevForward = glm::normalize(position - prevPosition);
        }
        else {
            prevForward = prevTangent;
        }

        // Calculate turn direction and amount
        glm::vec3 forwardDelta = forward - prevForward;
        float turnAmount = glm::length(forwardDelta);

        // Determine banking direction (left or right turn)
        right = glm::normalize(glm::cross(forward, up));
        glm::vec3 turnDir = glm::cross(prevForward, forward);
        float bankingAngle = turnAmount * bankingFactor;

        if (glm::dot(turnDir, up) < 0.0f) {
            bankingAngle = -bankingAngle;
        }

        // Apply banking rotation around forward axis
        glm::mat4 bankingRot = glm::rotate(glm::mat4(1.0f), bankingAngle, forward);
        up = glm::vec3(bankingRot * glm::vec4(up, 0.0f));

        // Recalculate right and up to maintain orthogonality
        right = glm::normalize(glm::cross(forward, up));
        up = glm::normalize(glm::cross(right, forward));
        break;
    }

    case FACE_CENTER: {
        // Always face the path center
        forward = glm::normalize(centerOfInterest - position);
        right = glm::normalize(glm::cross(forward, up));
        up = glm::cross(right, forward);
        break;
    }

    case FACE_POINT_OF_INTEREST: {
        // Face external point while moving
        forward = glm::normalize(centerOfInterest - position);
        right = glm::normalize(glm::cross(forward, up));
        up = glm::cross(right, forward);
        break;
    }
    }

    // Build orientation matrix
    glm::mat4 orientation(1.0f);
    orientation[0] = glm::vec4(right, 0.0f);
    orientation[1] = glm::vec4(up, 0.0f);
    orientation[2] = glm::vec4(forward, 0.0f);
    orientation[3] = glm::vec4(position, 1.0f);

    return orientation;
}

/**
 * @brief Update the follower's state over time.
 *
 * Advances the current time by deltaTime and calculates the PathState,
 * including position, orientation, speed, and normalized arc length.
 *
 * @param deltaTime Time step (seconds) to advance.
 * @return PathState Struct containing current position, orientation axes, speed, and normalized arc length.
 */
PathFollower::PathState PathFollower::update(float deltaTime)
{
    PathState state;
    if (isAnimating)
    {
        currentTime += deltaTime;

        if (currentTime >= totalDuration)
        {
            currentTime = totalDuration;
            isAnimating = false;   // Movement is officially finished
        }
    }

    if (!isAnimating && currentTime >= totalDuration)
    {
        float arcLength = easeInOutDistance(totalDuration);

        glm::vec3 position, tangent;
        getPositionAndTangent(arcLength, position, tangent);

        glm::mat4 orientMatrix =
            calculateOrientation(position, tangent, prevTangent);

        state.position = position;
        state.forward = glm::vec3(orientMatrix[2]);
        state.up = glm::vec3(orientMatrix[1]);
        state.right = glm::vec3(orientMatrix[0]);

        state.speed = 0.0f;
        state.normalizedArcLength = arcLength;

        // Preserve last valid frame data
        prevPosition = position;
        prevTangent = tangent;
        return state;
    }

    float arcLength = easeInOutDistance(currentTime);

    glm::vec3 position, tangent;
    getPositionAndTangent(arcLength, position, tangent);

    glm::mat4 orientMatrix =
        calculateOrientation(position, tangent, prevTangent);

    state.position = position;
    state.forward = glm::vec3(orientMatrix[2]);
    state.up = glm::vec3(orientMatrix[1]);
    state.right = glm::vec3(orientMatrix[0]);

    if (currentTime < easeInDuration)
    {
        state.speed = maxSpeed * (currentTime / easeInDuration);
    }
    else if (currentTime < totalDuration - easeOutDuration)
    {
        state.speed = maxSpeed;
    }
    else
    {
        float easeOutTime =
            currentTime - (totalDuration - easeOutDuration);

        state.speed = maxSpeed *
            (1.0f - easeOutTime / easeOutDuration);
    }

    state.normalizedArcLength = arcLength;

    prevPosition = position;
    prevTangent = tangent;

    return state;
}


/**
 * @brief Set the orientation mode of the follower.
 *
 * @param mode One of the OrientationMode enum values (e.g., TANGENT_FORWARD, LOOK_AHEAD_WITH_BANKING, etc.).
 */
void PathFollower::setOrientationMode(OrientationMode mode) {
    orientationMode = mode;
}


/**
 * @brief Set the center of interest point for orientation modes that face a point.
 *
 * @param center 3D point to face (world-space coordinates).
 */
void PathFollower::setCenterOfInterest(const glm::vec3& center) {
    centerOfInterest = center;
}


/**
 * @brief Set the look-ahead distance used for look-ahead orientation modes.
 *
 * @param distance Look-ahead distance in percentage of path length.
 */
void PathFollower::setLookAheadDistance(float distance) {
    lookAheadDistance = distance;
}

/**
 * @brief Set the banking factor for banking orientation modes.
 *
 * @param factor Banking intensity multiplier.
 */
void PathFollower::setBankingFactor(float factor) {
    bankingFactor = factor;
}

/**
 * @brief Enable or disable looping of the path animation.
 *
 * @param loop True to enable looping, false to disable.
 */
void PathFollower::setLooping(bool loop) {
    //loopAnimation = loop;
    loopAnimation = false;
}

/**
 * @brief Start the path-following animation.
 */
void PathFollower::start() {
    isAnimating = true;
}

/**
 * @brief Pause the path-following animation.
 */
void PathFollower::pause() {
    isAnimating = false;
}


/**
 * @brief Reset the path-following animation to the beginning.
 */
void PathFollower::reset() {
    currentTime = 0.0f;
    isAnimating = false;
}

/**
 * @brief Get the current state of the follower without updating time.
 *
 * Returns the current position, orientation, speed, and normalized arc length.
 *
 * @return PathState Current state of the follower.
 */
PathFollower::PathState PathFollower::getCurrentState() {

    PathState state;
    float arcLength = easeInOutDistance(currentTime);
    glm::vec3 position, tangent;
    const_cast<PathFollower*>(this)->getPositionAndTangent(arcLength, position, tangent);

    glm::mat4 orientMatrix = const_cast<PathFollower*>(this)->calculateOrientation(position, tangent, prevTangent);

    state.position = position;
    state.forward = glm::vec3(orientMatrix[2]);
    state.up = glm::vec3(orientMatrix[1]);
    state.right = glm::vec3(orientMatrix[0]);

    // Calculate current speed
    if (currentTime < easeInDuration) {
        state.speed = maxSpeed * (currentTime / easeInDuration);
    }
    else if (currentTime < totalDuration - easeOutDuration) {
        state.speed = maxSpeed;
    }
    else {
        float easeOutTime = currentTime - (totalDuration - easeOutDuration);
        state.speed = maxSpeed * (1.0f - easeOutTime / easeOutDuration);
    }

    state.normalizedArcLength = arcLength;
    return state;
}