#include "PathFollower.h"
#include <gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

/**
 * @brief Construct a PathFollower instance.
 *
 * @param pg Pointer to the PathGenerator that provides the curve.
 * @param duration Total duration of the animation along the path.
 */
PathFollower::PathFollower(PathGenerator* pg, float duration)
    : pathGen(pg), totalDuration(duration), currentTime(0.0f),
    isAnimating(false), loopAnimation(true),
    orientationMode(LOOK_AHEAD_WITH_BANKING),
    centerOfInterest(0.0f, 0.0f, 0.0f),
    lookAheadDistance(3.0f), bankingFactor(0.4f),
    prevPosition(0.0f), prevTangent(1.0f, 0.0f, 0.0f)
{
    setDuration(duration, 0.2f, 0.2f);
}

/**
 * @brief Set the duration of the animation with ease-in and ease-out ratios.
 *
 * Calculates maximum speed to maintain the correct distance along the path.
 *
 * @param duration Total animation duration.
 * @param easeInRatio Fraction of total duration for ease-in (0..1).
 * @param easeOutRatio Fraction of total duration for ease-out (0..1).
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
 * @brief Compute normalized arc length [0,1] for a given time using ease-in/out.
 *
 * @param t Current time [0, totalDuration].
 * @return float Normalized arc length along the path.
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
 * @brief Compute position and tangent on the path for a normalized global arc length.
 *
 * @param globalArcLength Normalized arc length along the full path [0,1].
 * @param[out] pos Calculated position on the path.
 * @param[out] tangent Normalized tangent vector at the position.
 */
void PathFollower::getPositionAndTangent(float globalArcLength,
    glm::vec3& pos,
    glm::vec3& tangent) {
    int numSegments = pathGen->getNumPoints() / 3;

    // Clamp and wrap
    globalArcLength = fmod(globalArcLength, 1.0f);
    if (globalArcLength < 0.0f) globalArcLength += 1.0f;

    // Find which segment we're in
    int segment = (int)(globalArcLength * numSegments);
    segment = glm::clamp(segment, 0, numSegments - 1);

    // Local arc length within segment [0, 1]
    float localArcLength = (globalArcLength * numSegments) - segment;

    // Get parameter t for this segment
    float t = pathGen->parameterFromArcLength(segment, localArcLength);

    // Get control points
    int i0 = (segment * 3) % pathGen->getNumPoints();
    int i1 = (segment * 3 + 1) % pathGen->getNumPoints();
    int i2 = (segment * 3 + 2) % pathGen->getNumPoints();
    int i3 = (segment * 3 + 3) % pathGen->getNumPoints();

    glm::vec3 p0 = pathGen->getControlPoint(i0);
    glm::vec3 p1 = pathGen->getControlPoint(i1);
    glm::vec3 p2 = pathGen->getControlPoint(i2);
    glm::vec3 p3 = pathGen->getControlPoint(i3);

    // Evaluate position
    pos = pathGen->evaluateCubicBezier(p0, p1, p2, p3, t);

    // Calculate tangent (derivative of cubic Bezier)
    float u = 1.0f - t;
    tangent = 3.0f * u * u * (p1 - p0) +
        6.0f * u * t * (p2 - p1) +
        3.0f * t * t * (p3 - p2);

    if (glm::length(tangent) > 0.001f) {
        tangent = glm::normalize(tangent);
    }
}

/**
 * @brief Calculate the orientation matrix based on the current mode.
 *
 * Modes include tangent following, look-ahead, banking, or facing points.
 *
 * @param position Current position on the path.
 * @param tangent Current tangent vector.
 * @param prevTangent Previous tangent vector for banking calculations.
 * @return glm::mat4 4x4 orientation matrix.
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
 * @brief Update the path follower state over time.
 *
 * @param deltaTime Time step in seconds.
 * @return PathState Current state including position, orientation, speed, and arc length.
 */
PathFollower::PathState PathFollower::update(float deltaTime) {
    PathState state;

    if (isAnimating) {
        currentTime += deltaTime;

        if (currentTime >= totalDuration) {
            if (loopAnimation) {
                currentTime = fmod(currentTime, totalDuration);
            }
            else {
                currentTime = totalDuration;
                isAnimating = false;
            }
        }
    }

    // Get normalized arc length based on ease-in/ease-out
    float arcLength = easeInOutDistance(currentTime);

    // Get position and tangent
    glm::vec3 position, tangent;
    getPositionAndTangent(arcLength, position, tangent);

    // Calculate orientation
    glm::mat4 orientMatrix = calculateOrientation(position, tangent, prevTangent);

    // Fill state
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

    // Store for next frame
    prevPosition = position;
    prevTangent = tangent;

    return state;
}

/**
 * @brief Set orientation mode for the follower.
 *
 * @param mode New orientation mode.
 */
void PathFollower::setOrientationMode(OrientationMode mode) {
    orientationMode = mode;
}


/**
 * @brief Set the center of interest for FACE_CENTER or FACE_POINT_OF_INTEREST modes.
 *
 * @param center Center position to look at.
 */
void PathFollower::setCenterOfInterest(const glm::vec3& center) {
    centerOfInterest = center;
}

/**
 * @brief Set the look-ahead distance (used in look-ahead modes).
 *
 * @param distance Look-ahead distance.
 */
void PathFollower::setLookAheadDistance(float distance) {
    lookAheadDistance = distance;
}

/**
 * @brief Set the banking factor (how strongly the object banks on curves).
 *
 * @param factor Banking factor scalar.
 */
void PathFollower::setBankingFactor(float factor) {
    bankingFactor = factor;
}

/**
 * @brief Enable or disable looping of the animation.
 *
 * @param loop True to loop, false to stop at end.
 */
void PathFollower::setLooping(bool loop) {
    loopAnimation = loop;
}

/**
 * @brief Start/resume animation.
 */
void PathFollower::start() {
    isAnimating = true;
}

/**
 * @brief Pause the animation.
 */
void PathFollower::pause() {
    isAnimating = false;
}


/**
 * @brief Reset animation to start.
 */
void PathFollower::reset() {
    currentTime = 0.0f;
    isAnimating = false;
}

/**
 * @brief Get the current state without advancing time.
 *
 * @return PathState Current position, orientation, speed, and arc length.
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