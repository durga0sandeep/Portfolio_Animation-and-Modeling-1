#pragma once

/**
 * @file Mesh.h
 * @brief Mesh class supporting skeletal animation, skinning, and animation clips using Assimp and OpenGL.
 */

#include <map>
#include <vector>
#include "texture.h"
#include "shapes.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <map>
#include <string>

 

// Maximum number of bones affecting a single vertex
#define MAX_NUM_BONES_PER_VERTEX 4

//Bone Ids and weights (up to 4)
struct BoneData
{
    int IDs[MAX_NUM_BONES_PER_VERTEX] = { 0 };
    float Weights[MAX_NUM_BONES_PER_VERTEX] = { 0.0f };

    BoneData()
    {
        for (int i = 0; i < MAX_NUM_BONES_PER_VERTEX; i++) {
            IDs[i] = 0;
            Weights[i] = 0.0f;
        }
    }

    void AddBoneData(int _id, float _w)
    {
        int arr_size = sizeof(IDs) / sizeof(IDs[0]);
        for (int i = 0; i < arr_size; i++) {
            if (Weights[i] == 0.0) {
                IDs[i] = _id;
                Weights[i] = _w;
                return;
            }
        }

        assert(0);
    }
};

/**
 * @brief Stores transformation data for a bone.
 */
struct BoneInfo
{
    aiMatrix4x4 OffsetMatrix;
    aiMatrix4x4 FinalTransformation;

    BoneInfo() {}
    BoneInfo(const aiMatrix4x4& offset) : OffsetMatrix(offset) { }
};


/**
 * @brief Position keyframe
 */
struct  KeyPosition
{
    glm::vec3 position;
    float timeStamp;
};

/**
 * @brief Rotation keyframe
 */
struct KeyRotation
{
    glm::quat rotation;
    float timeStamp;
};

/**
 * @brief Scale keyframe
 */
struct KeyScale {
    glm::vec3 scale;
    float timeStamp;
};

/**
 * @brief Animation data for a single bone.
 */
struct BoneAnimation {

    std::vector<KeyPosition> positions;
    std::vector<KeyRotation> rotations;
    std::vector<KeyScale> scales;
    std::vector<float> positionTimestamps;
    std::vector<float> rotationTimestamps;
    std::vector<float> scaleTimestamps;

};

/**
 * @brief Stores animation data for an entire skeleton.
 */
struct Animation {
    float duration;
    float ticksPerSecond;

    // all animated bones indexed by boneID
    std::vector<BoneAnimation> boneAnimations;
};

/**
 * @brief Defines a playable animation clip.
 */
struct AnimationClip {
    std::string name;
    int trackIndex;   // Which animation track this comes from
    float startTime;  // Start time in ticks
    float endTime;    // End time in ticks
    bool loop;        // Should this animation loop?
};


/**
 * @brief Mesh class supporting skeletal animation and rendering.
 */
class Mesh
{
public:
	Mesh();
	~Mesh();

    void LoadMesh(const std::string& path);
	void Draw(int programId, long double dt);
    void MakeVAO();

    void PrintValues();//for printing the bone names
    

    void DebugSkeleton(const glm::mat4& viewProj, GLuint shader);
    void DrawSkeletonRecursive(const aiNode* node, const glm::mat4& modelTransform,
        GLuint shader, GLuint lineVAO, GLuint lineVBO);
    void SetupLineVAO(GLuint& VAO, GLuint& VBO);
    void DrawLine(const glm::vec3& start, const glm::vec3& end,
        GLuint shader, GLuint lineVAO, GLuint lineVBO);


    //
    //Animations
    void LoadAnimation();
    void CalFinalTransformations( );
    void ReadNodeHierarchy(float animationTime, const aiNode* node, const glm::mat4& parentTransform);
    glm::vec3 CalcInterpolatedPosition(float time, const BoneAnimation& anim);
    glm::quat CalcInterpolatedRotation(float time, const BoneAnimation& anim);
    glm::vec3 CalcInterpolatedScaling(float time, const BoneAnimation& anim);


    glm::vec3 toVec3(const aiVector3D& v);
    glm::quat toQuat(const aiQuaternion& q);

    void DataPrint();
    void LoadAllAnimations();

    void DefineAnimationClip(const std::string& clipName, int trackIndex,
        float startFrame, float endFrame, bool loop);
    void SetAnimation(const std::string& animName);
    void SetAnimationYOffset(const std::string& animName, float offset);
    float GetAnimationYOffset(const std::string& animName) const;
    void ReadNodeHierarchy(float animationTime, const aiNode* node,
        const glm::mat4& parentTransform, const Animation& track);

    const AnimationClip* GetCurrentClip() const;
    float GetClipLocalTime(float globalTime, const AnimationClip& clip) const;

private:

    void InitMesh(const std::string& path);
    void InitBone(const aiMesh* mesh, int index);
    int GetBoneId(const aiBone* b);


	Assimp::Importer Importer;
	const aiScene* m_scene = NULL;
    aiMatrix4x4 m_inverseTrans;
    std::map<std::string, int> m_name_index;
    
    GLuint m_vao = 0;

	std::vector<glm::vec4> m_pnt;
	std::vector<glm::vec3> m_norm;
	std::vector<glm::vec2> m_tex;
	std::vector<glm::vec3> m_tan;
	std::vector<int> m_indices;

    std::vector<BoneData> m_bones;
    std::vector<BoneInfo> m_boneInfo;
    std::map<std::string, glm::mat4> nodeGlobalTransforms;
    
    glm::mat4 m_currentModelTransform;

    std::map<std::string, float> m_animationYOffsets;

    //
    //Animation
    aiAnimation* anim;
    Animation AnimationData;
    std::string m_rootNodeName;
    int boneCount;

    int FinalCalRef = m_name_index.size();
    int RefCalValue = -1;

    std::vector<Animation> m_animationTracks;

    std::map<std::string, AnimationClip> m_animationClips;

    std::string m_currentAnimationName;
    float m_currentAnimationTime;


};




