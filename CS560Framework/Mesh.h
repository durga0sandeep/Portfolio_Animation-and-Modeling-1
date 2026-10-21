#ifndef MESH_H
#define MESH_H

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <map>
#include <string>
#include "RigidBody.h"


#define MAX_NUM_BONES_PER_VERTEX 4

// Forward declarations
//class PathFollower;
//class Texture;

// Structure to hold bone data per vertex
struct BoneData {
    int IDs[MAX_NUM_BONES_PER_VERTEX];
    float Weights[MAX_NUM_BONES_PER_VERTEX];

    BoneData() {
        for (int i = 0; i < MAX_NUM_BONES_PER_VERTEX; i++) {
            IDs[i] = 0;
            Weights[i] = 0.0f;
        }
    }

    void AddBoneData(int BoneID, float Weight) {
        for (int i = 0; i < MAX_NUM_BONES_PER_VERTEX; i++) {
            if (Weights[i] == 0.0f) {
                IDs[i] = BoneID;
                Weights[i] = Weight;
                return;
            }
        }
    }
};

// Bone information
struct BoneInfo {
    aiMatrix4x4 OffsetMatrix;
    aiMatrix4x4 FinalTransformation;

    BoneInfo() {}
    BoneInfo(const aiMatrix4x4& offset) : OffsetMatrix(offset) {}
};

// Keyframe structures
struct KeyPosition {
    glm::vec3 position;
    float timeStamp;
};

struct KeyRotation {
    glm::quat rotation;
    float timeStamp;
};

struct KeyScale {
    glm::vec3 scale;
    float timeStamp;
};

// Bone animation data
struct BoneAnimation {
    std::vector<KeyPosition> positions;
    std::vector<KeyRotation> rotations;
    std::vector<KeyScale> scales;
    std::vector<float> positionTimestamps;
    std::vector<float> rotationTimestamps;
    std::vector<float> scaleTimestamps;
};

// Complete animation track
struct Animation {
    float duration;
    float ticksPerSecond;
    std::vector<BoneAnimation> boneAnimations;
};

// Structure to define animation clips within a single track
struct AnimationClip {
    std::string name;
    int trackIndex;   // Which animation track this comes from
    float startTime;  // Start time in ticks
    float endTime;    // End time in ticks
    bool loop;        // Should this animation loop?
};

class Mesh {
private:
    
    //N-NEW
    float m_animationSpeedMultiplier;  

    // Assimp
    Assimp::Importer Importer;
    
    aiMatrix4x4 m_inverseTrans;


    // Mesh data
    std::vector<glm::vec4> m_pnt;
    std::vector<glm::vec3> m_norm;
    std::vector<glm::vec2> m_tex;
    std::vector<glm::vec3> m_tan;
    std::vector<unsigned int> m_indices;

    // Bone data
    std::vector<BoneData> m_bones;
    
    std::map<std::string, glm::mat4> nodeGlobalTransforms;


public:
    // Constructor/Destructor
    Mesh();
    ~Mesh();

    // Scale
    float scaleFactor = 0.1f;

    const aiScene* m_scene;
    // OpenGL
    unsigned int m_vao;

    //N-NEW
    void SetAnimationSpeed(float speedMultiplier);
    float GetAnimationSpeed() const;

    // Loading
    void LoadMesh(const std::string& path);

    void Draw(int programId, long double dt);

    void SetRigibody(RigidBody& Rb);

private:

    RigidBody* gCubeRef;
        
    // Initialization
    void InitMesh(const std::string& path);
    void MakeVAO();

};

// Matrix conversion functions
glm::mat4 convert(const aiMatrix4x4& m);
glm::mat4 convert(const aiMatrix3x3& m);
aiMatrix4x4 convert(const glm::mat4& m);

#endif // MESH_H