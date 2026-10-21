#include "PathGenerator.h"
#include <cmath>

/**
 * @brief Calculate the binomial coefficient "n choose k".
 *
 * @param n Total number of items.
 * @param k Number of items to choose.
 * @return int Binomial coefficient value. Returns 0 if k > n.
 */
int PathGenerator::binomial(int n, int k) {
    if (k > n) return 0;
    if (k == 0 || k == n) return 1;

    int result = 1;
    for (int i = 1; i <= k; i++) {
        result *= (n - i + 1);
        result /= i;
    }
    return result;
}


/**
 * @brief Update the vertices of the curve in the VBO using cubic Bezier segments.
 *
 * This function iterates over all cubic Bezier segments formed by control points,
 * evaluates points along each segment, and updates the OpenGL VBO with the vertex data.
 */
void PathGenerator::updateCurveVertices() {
    curveVertices.clear();

    // With 9 points, we create 3 segments:
    // Seg 1: [0, 1, 2, 3]
    // Seg 2: [3, 4, 5, 6]
    // Seg 3: [6, 7, 8, 0] <-- The Wrap Around
    //int numSegments = NUM_POINTS / 3;
    int numSegments = (NUM_POINTS - 1) / 3;


    for (int seg = 0; seg < numSegments; seg++) {
        int i0 = (seg * 3) % NUM_POINTS;
        int i1 = (seg * 3 + 1) % NUM_POINTS;
        int i2 = (seg * 3 + 2) % NUM_POINTS;
        int i3 = (seg * 3 + 3) % NUM_POINTS;

        glm::vec3 p0 = controlPoints[i0];
        glm::vec3 p1 = controlPoints[i1];
        glm::vec3 p2 = controlPoints[i2];
        glm::vec3 p3 = controlPoints[i3];

        int segmentRes = resolution / numSegments;
        for (int i = 0; i < segmentRes; i++) { // Use < instead of <= to avoid overlap
            float t = (float)i / (float)segmentRes;
            float u = 1.0f - t;
            // Cubic Bezier Formula
            glm::vec3 pt = u * u * u * p0 + 3.0f * u * u * t * p1 + 3.0f * u * t * t * p2 + t * t * t * p3;
            curveVertices.push_back(pt);
        }
    }
    // Add very last point to close the visual loop exactly
    //curveVertices.push_back(controlPoints[0]);

    // Update VBO
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, curveVertices.size() * sizeof(glm::vec3), curveVertices.data(), GL_DYNAMIC_DRAW);
}



/**
 * @brief Evaluate a cubic Bezier curve at a given parameter t using De Casteljau's formula.
 *
 * @param p0 First control point.
 * @param p1 Second control point.
 * @param p2 Third control point.
 * @param p3 Fourth control point.
 * @param t Parameter in [0, 1].
 * @return glm::vec3 Point on the curve at parameter t.
 */
glm::vec3 PathGenerator::evaluateCubicBezier(
    const glm::vec3& p0, const glm::vec3& p1,
    const glm::vec3& p2, const glm::vec3& p3, float t)
{

    float u = 1.0f - t;
    return u * u * u * p0 + 3.0f * u * u * t * p1 + 3.0f * u * t * t * p2 + t * t * t * p3;

}



/**
 * @brief Constructor for PathGenerator.
 *
 * Initializes control points along a straight line, sets up OpenGL VAO/VBO,
 * updates curve vertices, and builds the arc length lookup tables.
 *
 * @param res Resolution of the curve (number of vertices per segment).
 */
PathGenerator::PathGenerator(int res) : resolution(res) {
    // Define start and end points for a straight line
    glm::vec3 startPoint(-20.0f, 0.0f, 0.0f);
    glm::vec3 endPoint(20.0f, 0.0f, 0.0f);

    // Distribute all 9 control points evenly along the line
    // This ensures that even with multiple Bezier segments, the path is perfectly straight
    for (int i = 0; i < NUM_POINTS; i++) {
        float t = (float)i / (float)(NUM_POINTS - 1);
        controlPoints[i] = startPoint + t * (endPoint - startPoint);
    }

    // Standard initialization remains the same
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    updateCurveVertices();
    buildArcLengthTables();
}
    
/**
 * @brief Destructor for PathGenerator.
 *
 * Cleans up OpenGL VAO and VBO resources.
 */
PathGenerator::~PathGenerator() {
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
}

/**
 * @brief Set the position of a control point.
 *
 * Also maintains C1 continuity of the curve by adjusting neighboring handles.
 *
 * @param index Index of the control point to set.
 * @param point New position of the control point (y and z are set to 0).
 */
void PathGenerator::setControlPoint(int index, const glm::vec3& point) {
    if (index < 0 || index >= NUM_POINTS) return;

    glm::vec3 newPoint = point;
    newPoint.y = 0.0f;
    newPoint.z = 0.0f;

    glm::vec3 delta = newPoint - controlPoints[index];
    controlPoints[index] = newPoint;

    // Keep existing C1 continuity logic
    if (index % 3 == 0) {
        int prevHandle = (index - 1 + NUM_POINTS) % NUM_POINTS;
        int nextHandle = (index + 1) % NUM_POINTS;
        controlPoints[prevHandle] += delta;
        controlPoints[nextHandle] += delta;
    }
    else {
        int node = -1;
        int oppositeHandle = -1;

        if ((index + 1) % 3 == 0) {
            node = (index + 1) % NUM_POINTS;
            oppositeHandle = (index + 2) % NUM_POINTS;
        }
        else if ((index - 1) % 3 == 0) {
            node = (index - 1 + NUM_POINTS) % NUM_POINTS;
            oppositeHandle = (index - 2 + NUM_POINTS) % NUM_POINTS;
        }

        if (node != -1) {
            float dx = controlPoints[index].x - controlPoints[node].x;
            controlPoints[oppositeHandle].x = controlPoints[node].x - dx;
            controlPoints[oppositeHandle].y = 0.0f;
            controlPoints[oppositeHandle].z = 0.0f;
        }
    }

    updateCurveVertices();
    buildArcLengthTables();
}

/**
 * @brief Get the position of a control point.
 *
 * @param index Index of the control point.
 * @return glm::vec3 Position of the control point. Returns (0,0,0) if index is invalid.
 */
glm::vec3 PathGenerator::getControlPoint(int index) const {
    if (index >= 0 && index < NUM_POINTS) {
        return controlPoints[index];
    }
    return glm::vec3(0.0f);
}

/**
 * @brief Get the total number of control points.
 *
 * @return int Number of control points (NUM_POINTS).
 */
int PathGenerator::getNumPoints() const {
    return NUM_POINTS;
}

/**
 * @brief Set the resolution of the curve and update vertices and arc length tables.
 *
 * @param res Number of vertices per segment.
 */
void PathGenerator::setResolution(int res) {
    resolution = res;
    updateCurveVertices();
    buildArcLengthTables();
}

/**
 * @brief Draw the curve using OpenGL.
 *
 * Uses GL_LINE_STRIP to draw the vertices stored in the VBO.
 */
void PathGenerator::draw() {
    glBindVertexArray(vao);
    glDrawArrays(GL_LINE_STRIP, 0, curveVertices.size());
    glBindVertexArray(0);
}

/**
 * @brief Compute the arc length of a cubic Bezier segment adaptively.
 *
 * @param p0 First control point.
 * @param p1 Second control point.
 * @param p2 Third control point.
 * @param p3 Fourth control point.
 * @param t0 Start parameter of the segment.
 * @param t1 End parameter of the segment.
 * @param tolerance Tolerance for adaptive subdivision.
 * @return float Approximated arc length of the segment between t0 and t1.
 *
 * Uses recursive subdivision based on chord-length vs. actual segment length.
 */
float PathGenerator::getArcLengthAdaptive(
    const glm::vec3& p0, const glm::vec3& p1,
    const glm::vec3& p2, const glm::vec3& p3,
    float t0, float t1, float tolerance)
{
    // Evaluate at three points
    glm::vec3 pt0 = evaluateCubicBezier(p0, p1, p2, p3, t0);
    glm::vec3 ptMid = evaluateCubicBezier(p0, p1, p2, p3, (t0 + t1) * 0.5f);
    glm::vec3 pt1 = evaluateCubicBezier(p0, p1, p2, p3, t1);

    // Calculate chord length and arc approximation
    float chordLength = glm::length(pt1 - pt0);
    float arcApprox = glm::length(ptMid - pt0) + glm::length(pt1 - ptMid);

    // If the difference is small enough, use the arc approximation
    if (fabs(arcApprox - chordLength) < tolerance) {
        return arcApprox;
    }

    // Otherwise, subdivide recursively
    float tMid = (t0 + t1) * 0.5f;
    float leftArc = getArcLengthAdaptive(p0, p1, p2, p3, t0, tMid, tolerance);
    float rightArc = getArcLengthAdaptive(p0, p1, p2, p3, tMid, t1, tolerance);

    return leftArc + rightArc;
}

/**
 * @brief Build arc length lookup tables for all curve segments.
 *
 * Computes cumulative arc length for each segment and normalizes values to [0, 1].
 * This allows for uniform parameterization along the curve.
 */
void PathGenerator::buildArcLengthTables() {
    arcLengthTables.clear();
    //int numSegments = NUM_POINTS / 3;
    int numSegments = (NUM_POINTS - 1) / 3;


    for (int seg = 0; seg < numSegments; seg++) {
        int i0 = (seg * 3) % NUM_POINTS;
        int i1 = (seg * 3 + 1) % NUM_POINTS;
        int i2 = (seg * 3 + 2) % NUM_POINTS;
        int i3 = (seg * 3 + 3) % NUM_POINTS;

        glm::vec3 p0 = controlPoints[i0];
        glm::vec3 p1 = controlPoints[i1];
        glm::vec3 p2 = controlPoints[i2];
        glm::vec3 p3 = controlPoints[i3];

        SegmentArcLengthTable table;
        table.entries.reserve(arcLengthSamples + 1);

        // Build the table by sampling the curve
        float accumulatedLength = 0.0f;
        glm::vec3 prevPoint = p0;

        // First entry
        table.entries.push_back({ 0.0f, 0.0f });

        // Sample points along the curve
        for (int i = 1; i <= arcLengthSamples; i++) {
            float t = (float)i / (float)arcLengthSamples;
            float tPrev = (float)(i - 1) / (float)arcLengthSamples;

            // Use adaptive method for this segment
            float segmentLength = getArcLengthAdaptive(p0, p1, p2, p3, tPrev, t);
            accumulatedLength += segmentLength;

            table.entries.push_back({ t, accumulatedLength });
        }

        // Store total length
        table.totalLength = accumulatedLength;

        // Normalize all arc lengths to [0, 1]
        if (table.totalLength > 0.0f) {
            for (auto& entry : table.entries) {
                entry.arcLength /= table.totalLength;
            }
        }

        arcLengthTables.push_back(table);
    }
}

/**
 * @brief Find the parameter t along a segment corresponding to a normalized arc length u.
 *
 * @param segmentIndex Index of the segment in the curve.
 * @param normalizedArcLength Arc length normalized to [0, 1].
 * @return float Parameter t in [0, 1] for the specified segment.
 *
 * Uses binary search on the precomputed arc length table for fast lookup.
 */
float PathGenerator::parameterFromArcLength(int segmentIndex, float normalizedArcLength) {
    if (segmentIndex < 0 || segmentIndex >= arcLengthTables.size()) {
        return 0.0f;
    }

    const SegmentArcLengthTable& table = arcLengthTables[segmentIndex];

    // Clamp input to [0, 1]
    normalizedArcLength = glm::clamp(normalizedArcLength, 0.0f, 1.0f);

    // Binary search
    int left = 0;
    int right = table.entries.size() - 1;

    // Handle edge cases
    if (normalizedArcLength <= 0.0f) return table.entries[0].t;
    if (normalizedArcLength >= 1.0f) return table.entries[right].t;

    while (right - left > 1) {
        int mid = (left + right) / 2;

        if (table.entries[mid].arcLength < normalizedArcLength) {
            left = mid;
        }
        else {
            right = mid;
        }
    }

    // Linear interpolation between the two nearest entries
    float arcLength0 = table.entries[left].arcLength;
    float arcLength1 = table.entries[right].arcLength;
    float t0 = table.entries[left].t;
    float t1 = table.entries[right].t;

    float ratio = (normalizedArcLength - arcLength0) / (arcLength1 - arcLength0);
    return t0 + ratio * (t1 - t0);
}



