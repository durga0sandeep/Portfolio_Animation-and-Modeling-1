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
#include "PathFollower.h"


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
    
    // Assimp
    Assimp::Importer Importer;
    const aiScene* m_scene;
    aiMatrix4x4 m_inverseTrans;

    // OpenGL
    unsigned int m_vao;

    // Mesh data
    std::vector<glm::vec4> m_pnt;
    std::vector<glm::vec3> m_norm;
    std::vector<glm::vec2> m_tex;
    std::vector<glm::vec3> m_tan;
    std::vector<unsigned int> m_indices;

    // Bone data
    std::vector<BoneData> m_bones;
    std::vector<BoneInfo> m_boneInfo;
    std::map<std::string, int> m_name_index;
    std::map<std::string, glm::mat4> nodeGlobalTransforms;

    glm::mat4 m_currentModelTransform;


    // Store all animation tracks from the FBX
    std::vector<Animation> m_animationTracks;

    // Define animation clips (can reference any track)
    std::map<std::string, AnimationClip> m_animationClips;

    // Current animation state
    std::string m_currentAnimationName;
    float m_currentAnimationTime;

    // Blending state
    bool m_isBlending;
    std::string m_targetAnimationName;
    float m_blendTime;
    float m_blendDuration;
    float m_targetAnimationTime;

    float m_animationSpeedMultiplier;
    std::map<std::string, float> m_animationYOffsets;



    // Scale
    float scaleFactor = 0.1f;

public:
    // Constructor/Destructor
    Mesh();
    ~Mesh();

    //N-NEW
    void SetAnimationSpeed(float speedMultiplier);
    float GetAnimationSpeed() const;


    void SetCurrentModelTransform(const glm::mat4& transform);

    // Loading
    void LoadMesh(const std::string& path);
    void LoadAllAnimations();

    // Animation clip definition
    void DefineAnimationClip(const std::string& clipName, int trackIndex,
        float startFrame, float endFrame, bool loop = true);
    void AddFullTrackAsClip(int trackIndex, const std::string& clipName, bool loop = true);
    void AddAllTracksAsClips(bool loop = true);

    // Animation control
    void SetAnimation(const std::string& animName);
    void ClearAnimationClips();
    std::vector<std::string> GetAnimationNames() const;
    bool HasAnimation(const std::string& animName) const;
    float GetAnimationDuration(const std::string& animName) const;
    int GetTrackCount() const { return m_animationTracks.size(); }
    std::string GetTrackName(int index) const;

    // Blending methods
    void BlendToAnimation(const std::string& targetAnim, float blendDuration);
    bool IsBlending() const { return m_isBlending; }
    float GetBlendProgress() const;

    // Rendering
    void Draw(int programId, long double dt, PathFollower& pathfollow);
    void DebugSkeleton(const glm::mat4& viewProj, GLuint shader);

    // Debug
    void PrintValues();
    void DataPrint();

    //    
    static void ExtractTRS(const glm::mat4& m, glm::vec3& translation, glm::quat& rotation, glm::vec3& scale);

    const AnimationClip* GetCurrentClip() const;

    void SetAnimationYOffset(const std::string& animName, float offset);
    float GetAnimationYOffset(const std::string & animName) const;

private:
    // Initialization
    void InitMesh(const std::string& path);
    void InitBone(const aiMesh* mesh, int index);
    void MakeVAO();

    // Animation
    // void LoadSingleAnimation(int animIndex);
    void CalFinalTransformations();
    void ReadNodeHierarchy(float animationTime, const aiNode* node,
        const glm::mat4& parentTransform, const Animation& track);

    // Interpolation
    glm::vec3 CalcInterpolatedPosition(float time, const BoneAnimation& anim);
    glm::quat CalcInterpolatedRotation(float time, const BoneAnimation& anim);
    glm::vec3 CalcInterpolatedScaling(float time, const BoneAnimation& anim);

    // Blending
    void UpdateBlending(float deltaTime);
    glm::mat4 BlendBoneTransforms(const glm::mat4& transform1,
        const glm::mat4& transform2,
        float blendFactor);

    // Helpers
    int GetBoneId(const aiBone* b);
    float GetClipLocalTime(float globalTime, const AnimationClip& clip) const;
    glm::vec3 toVec3(const aiVector3D& v);
    glm::quat toQuat(const aiQuaternion& q);

    // Debug skeleton
    void SetupLineVAO(unsigned int& VAO, unsigned int& VBO);
    void DrawLine(const glm::vec3& start, const glm::vec3& end,
        unsigned int shader, unsigned int VAO, unsigned int VBO);
    void DrawSkeletonRecursive(const aiNode* node, const glm::mat4& modelTransform,
        unsigned int shader, unsigned int lineVAO, unsigned int lineVBO);
};

// Matrix conversion functions
glm::mat4 convert(const aiMatrix4x4& m);
glm::mat4 convert(const aiMatrix3x3& m);
aiMatrix4x4 convert(const glm::mat4& m);

#endif // MESH_H