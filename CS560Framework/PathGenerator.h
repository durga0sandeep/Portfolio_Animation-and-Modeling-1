#pragma once

#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)
using namespace gl;

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>

#include <vector>


//
//Arc lenght value
struct ArcLengthEntry {
    float t;           // Parameter value [0, 1]
    float arcLength;   // Normalized arc length [0, 1]
};

struct SegmentArcLengthTable {
    std::vector<ArcLengthEntry> entries;
    float totalLength;  // Actual length before normalization
};


class PathGenerator {
private:
    static const int NUM_POINTS = 9;

    
    GLuint vao, vbo;
    
    std::vector<glm::vec3> curveVertices;
    
    int resolution;

    std::vector<SegmentArcLengthTable> arcLengthTables;
    int arcLengthSamples = 100;


public:
    
    // Calculate binomial coefficient
    int binomial(int n, int k);
    
    glm::vec3 evaluateCubicBezier(const glm::vec3& p0, const glm::vec3& p1,
        const glm::vec3& p2, const glm::vec3& p3, float t);

    // Update curve vertices in VBO
    void updateCurveVertices();

    glm::vec3 controlPoints[NUM_POINTS];
    // Constructor and Destructor
    PathGenerator(int res = 100);
    ~PathGenerator();

    // Set and Get Control Points
    void setControlPoint(int index, const glm::vec3& point);
    glm::vec3 getControlPoint(int index) const;

    // Get number of points
    int getNumPoints() const;

    // Set resolution of the curve
    void setResolution(int res);

    // Draw the curve
    void draw();



    // New methods to add to PathGenerator.h:
    void buildArcLengthTables();
    float getArcLengthAdaptive(const glm::vec3& p0, const glm::vec3& p1,
        const glm::vec3& p2, const glm::vec3& p3,
        float t0, float t1, float tolerance = 0.01f);
    float parameterFromArcLength(int segmentIndex, float normalizedArcLength);

};








