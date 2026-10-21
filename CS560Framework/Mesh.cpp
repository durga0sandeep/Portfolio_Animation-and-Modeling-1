#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)

using namespace gl;

#pragma warning(disable: 4190)
#include <glu.h>

#define CHECKERROR {GLenum err = glGetError(); if (err != GL_NO_ERROR) { fprintf(stderr, "OpenGL error (at line shapes.cpp:%d): %s\n", __LINE__, gluErrorString(err)); exit(-1);} }

#define GLM_ENABLE_EXPERIMENTAL
#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE

#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <glm/gtx/quaternion.hpp>
//#include <glm/gtx/matrix_decompose.hpp>

#include <iostream>

#include "Mesh.h"
#include "transform.h"
#include "shader.h"
#include "texture.h"

/**
 * @brief Converts Assimp 4x4 matrix to GLM 4x4 matrix
 */
glm::mat4 convert(const aiMatrix4x4& m)
{
    glm::mat4 res;
    res[0][0] = m.a1; res[0][1] = m.b1; res[0][2] = m.c1; res[0][3] = m.d1;
    res[1][0] = m.a2; res[1][1] = m.b2; res[1][2] = m.c2; res[1][3] = m.d2;
    res[2][0] = m.a3; res[2][1] = m.b3; res[2][2] = m.c3; res[2][3] = m.d3;
    res[3][0] = m.a4; res[3][1] = m.b4; res[3][2] = m.c4; res[3][3] = m.d4;
    return res;
}

/**
 * @brief Converts Assimp 3x3 matrix to GLM 4x4 matrix
 */
glm::mat4 convert(const aiMatrix3x3& m)
{
    glm::mat4 res;
    res[0][0] = m.a1; res[0][1] = m.a2; res[0][2] = m.a3; res[0][3] = 0.0;
    res[1][0] = m.b1; res[1][1] = m.b2; res[1][2] = m.b3; res[1][3] = 0.0;
    res[2][0] = m.c1; res[2][1] = m.c2; res[2][2] = m.c3; res[2][3] = 0.0;
    res[3][0] = 0.0; res[3][1] = 0.0; res[3][2] = 0.0; res[3][3] = 1.0;
    return res;
}

/**
 * @brief Converts GLM 4x4 matrix to Assimp 4x4 matrix
 */
aiMatrix4x4 convert(const glm::mat4& m)
{
    aiMatrix4x4 res;
    res.a1 = m[0][0]; res.b1 = m[0][1]; res.c1 = m[0][2]; res.d1 = m[0][3];
    res.a2 = m[1][0]; res.b2 = m[1][1]; res.c2 = m[1][2]; res.d2 = m[1][3];
    res.a3 = m[2][0]; res.b3 = m[2][1]; res.c3 = m[2][2]; res.d3 = m[2][3];
    res.a4 = m[3][0]; res.b4 = m[3][1]; res.c4 = m[3][2]; res.d4 = m[3][3];
    return res;
}

/**
 * @brief Default constructor for Mesh class
 */
Mesh::Mesh()
    : m_isBlending(false), m_blendTime(0.0f), m_blendDuration(0.0f),
    m_currentAnimationTime(0.0f), m_targetAnimationTime(0.0f), m_animationSpeedMultiplier(1.0f)
{
}

/**
 * @brief Destructor for Mesh class
 */
Mesh::~Mesh()
{
}

void Mesh::SetAnimationSpeed(float speedMultiplier)
{
    m_animationSpeedMultiplier = speedMultiplier;
}

float Mesh::GetAnimationSpeed() const
{
    return m_animationSpeedMultiplier;
}



/**
 * @brief Loads a 3D mesh from file using Assimp
 */
void Mesh::LoadMesh(const std::string& path, bool AnimFlag)
{
    m_pnt.clear();
    m_norm.clear();
    m_tex.clear();
    m_tan.clear();
    m_indices.clear();
    m_bones.clear();
    m_boneInfo.clear();
    m_name_index.clear();
    nodeGlobalTransforms.clear();
    m_animationTracks.clear();
    m_animationClips.clear();


    Importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
    m_scene = Importer.ReadFile(path.c_str(), aiProcess_Triangulate
        | aiProcess_GenSmoothNormals
        | aiProcess_FlipUVs
        | aiProcess_JoinIdenticalVertices);

    std::cout << std::endl;
    std::cout << "The number of children " << m_scene->mRootNode->mNumChildren << " " << std::endl;
    std::cout << std::endl;

    if (!m_scene) {
        printf("Error parsing '%s': '%s'\n", path.c_str(), Importer.GetErrorString());
        return;
    }

    m_inverseTrans = m_scene->mRootNode->mTransformation;
    m_inverseTrans = m_inverseTrans.Inverse();

    InitMesh(path);
    
    if (AnimFlag == true) {
        LoadAllAnimations();
    }

    MakeVAO();


    if (AnimFlag == true) {
        std::cout << " " << std::endl;
        std::cout << " " << std::endl;
        std::cout << "Details: " << std::endl;
        std::cout << "Model loaded: " << path << std::endl;
        std::cout << "Meshes: " << m_scene->mNumMeshes << std::endl;
        std::cout << "Animation Tracks: " << m_animationTracks.size() << std::endl;
        std::cout << " " << std::endl;
        std::cout << " " << std::endl;
    }
}

/**
 * @brief Loads all animation tracks from the FBX file
 */
void Mesh::LoadAllAnimations()
{
    std::cout << "\n=== Loading Animation Tracks ===" << std::endl;
    std::cout << "Total tracks found: " << m_scene->mNumAnimations << std::endl;

    if (m_scene->mNumAnimations == 0) {
        std::cout << "No animations found in scene" << std::endl;
        return;
    }

    int boneCount = m_name_index.size();

    for (unsigned int trackIdx = 0; trackIdx < m_scene->mNumAnimations; trackIdx++) {
        aiAnimation* anim = m_scene->mAnimations[trackIdx];

        std::string trackName(anim->mName.C_Str());
        if (trackName.empty()) {
            trackName = "Track_" + std::to_string(trackIdx);
        }

        std::cout << "\nTrack " << trackIdx << ": " << trackName << std::endl;
        std::cout << "  Duration: " << anim->mDuration << " ticks" << std::endl;
        std::cout << "  TicksPerSecond: " << anim->mTicksPerSecond << std::endl;
        std::cout << "  Channels: " << anim->mNumChannels << std::endl;

        Animation animation;
        animation.duration = (float)anim->mDuration;
        animation.ticksPerSecond = (float)(anim->mTicksPerSecond != 0
            ? anim->mTicksPerSecond
            : 25.0f);

        animation.boneAnimations.resize(boneCount);

        for (unsigned int c = 0; c < anim->mNumChannels; c++) {
            aiNodeAnim* channel = anim->mChannels[c];
            std::string boneName = channel->mNodeName.C_Str();

            auto it = m_name_index.find(boneName);
            if (it == m_name_index.end()) {
                continue;
            }

            int boneID = it->second;
            BoneAnimation& boneAnim = animation.boneAnimations[boneID];

            for (unsigned int i = 0; i < channel->mNumPositionKeys; i++) {
                aiVectorKey key = channel->mPositionKeys[i];
                boneAnim.positionTimestamps.push_back(key.mTime);
                boneAnim.positions.push_back({ toVec3(key.mValue), (float)key.mTime });
            }

            for (unsigned int i = 0; i < channel->mNumRotationKeys; i++) {
                aiQuatKey key = channel->mRotationKeys[i];
                boneAnim.rotationTimestamps.push_back(key.mTime);
                boneAnim.rotations.push_back({ toQuat(key.mValue), (float)key.mTime });
            }

            for (unsigned int i = 0; i < channel->mNumScalingKeys; i++) {
                aiVectorKey key = channel->mScalingKeys[i];
                boneAnim.scaleTimestamps.push_back(key.mTime);
                boneAnim.scales.push_back({ toVec3(key.mValue), (float)key.mTime });
            }
        }

        m_animationTracks.push_back(animation);
    }

    std::cout << "\n=== Animation tracks loaded successfully ===" << std::endl;
}

/**
 * @brief Defines an animation clip from a portion of a track
 */
void Mesh::DefineAnimationClip(const std::string& clipName, int trackIndex,
    float startFrame, float endFrame, bool loop)
{
    if (trackIndex >= m_animationTracks.size()) {
        std::cout << "Error: Track index " << trackIndex << " out of range!" << std::endl;
        return;
    }

    AnimationClip clip;
    clip.name = clipName;
    clip.trackIndex = trackIndex;
    clip.startTime = startFrame;
    clip.endTime = endFrame;
    clip.loop = loop;

    m_animationClips[clipName] = clip;

    float durationSeconds = (endFrame - startFrame) / m_animationTracks[trackIndex].ticksPerSecond;
    std::cout << "Defined clip '" << clipName << "' from track " << trackIndex
        << ": frames " << startFrame << "-" << endFrame
        << " (" << durationSeconds << "s)" << std::endl;
}

/**
 * @brief Adds an entire animation track as a single clip
 */
void Mesh::AddFullTrackAsClip(int trackIndex, const std::string& clipName, bool loop)
{
    if (trackIndex >= m_animationTracks.size()) {
        std::cout << "Error: Track index " << trackIndex << " out of range!" << std::endl;
        return;
    }

    const Animation& track = m_animationTracks[trackIndex];

    std::string name = clipName;
    if (name.empty()) {
        name = GetTrackName(trackIndex);
    }

    AnimationClip clip;
    clip.name = name;
    clip.trackIndex = trackIndex;
    clip.startTime = 0.0f;
    clip.endTime = track.duration;
    clip.loop = loop;

    m_animationClips[name] = clip;

    float durationSeconds = track.duration / track.ticksPerSecond;
    std::cout << "Added full track " << trackIndex << " as clip '" << name
        << "' (" << durationSeconds << "s)" << std::endl;
}

/**
 * @brief Automatically adds all tracks as individual animation clips
 */
void Mesh::AddAllTracksAsClips(bool loop)
{
    std::cout << "\nAdding all tracks as clips..." << std::endl;

    for (size_t i = 0; i < m_animationTracks.size(); i++) {
        AddFullTrackAsClip(i, "", loop);
    }
}

/**
 * @brief Gets the name of an animation track
 */
std::string Mesh::GetTrackName(int index) const
{
    if (index >= m_scene->mNumAnimations) {
        return "Unknown";
    }

    std::string name(m_scene->mAnimations[index]->mName.C_Str());
    if (name.empty()) {
        name = "Animation_" + std::to_string(index);
    }
    return name;
}

/**
 * @brief Clears all defined animation clips
 */
void Mesh::ClearAnimationClips()
{
    m_animationClips.clear();
    m_currentAnimationName = "";
    m_currentAnimationTime = 0.0f;
}

/**
 * @brief Sets the current animation by name
 */
void Mesh::SetAnimation(const std::string& animName)
{
    auto it = m_animationClips.find(animName);
    if (it == m_animationClips.end()) {
        std::cout << "Animation '" << animName << "' not found!" << std::endl;
        return;
    }

    m_currentAnimationName = animName;
    m_currentAnimationTime = 0.0f;
    m_isBlending = false;
    std::cout << "Switched to animation: " << animName << std::endl;
}

/**
 * @brief Gets all available animation clip names
 */
std::vector<std::string> Mesh::GetAnimationNames() const
{
    std::vector<std::string> names;
    for (const auto& pair : m_animationClips) {
        names.push_back(pair.first);
    }
    return names;
}

/**
 * @brief Checks if an animation clip exists
 */
bool Mesh::HasAnimation(const std::string& animName) const
{
    return m_animationClips.find(animName) != m_animationClips.end();
}

/**
 * @brief Gets the duration of a specific animation clip
 */
float Mesh::GetAnimationDuration(const std::string& animName) const
{
    auto it = m_animationClips.find(animName);
    if (it == m_animationClips.end()) {
        return 0.0f;
    }

    const AnimationClip& clip = it->second;
    const Animation& track = m_animationTracks[clip.trackIndex];
    return (clip.endTime - clip.startTime) / track.ticksPerSecond;
}

/**
 * @brief Gets the current animation clip
 */
const AnimationClip* Mesh::GetCurrentClip() const
{
    auto it = m_animationClips.find(m_currentAnimationName);
    if (it != m_animationClips.end()) {
        return &it->second;
    }
    return nullptr;
}

/**
 * @brief Converts animation time to clip-local time considering clip range
 */
float Mesh::GetClipLocalTime(float globalTime, const AnimationClip& clip) const
{
    return clip.startTime + globalTime;
}

/**
 * @brief Initiates a blend from current animation to target animation
 */
void Mesh::BlendToAnimation(const std::string& targetAnim, float blendDuration)
{

    if (!HasAnimation(targetAnim)) {
    //    std::cout << "Cannot blend to '" << targetAnim << "' - animation not found!" << std::endl;
        return;
    }

    if (m_currentAnimationName == targetAnim) {
      //  std::cout << "Already playing '" << targetAnim << "'" << std::endl;
        return;
    }

    /*
    std::cout << "Starting blend from '" << m_currentAnimationName
        << "' to '" << targetAnim << "' over "
        << blendDuration << " seconds" << std::endl;
    */

    m_isBlending = true;
    m_targetAnimationName = targetAnim;
    m_blendDuration = blendDuration;
    m_blendTime = 0.0f;
    m_targetAnimationTime = 0.0f;
}



/**
 * @brief Gets the current blend progress
 */
float Mesh::GetBlendProgress() const
{
    if (!m_isBlending || m_blendDuration <= 0.0f) {
        return 1.0f;
    }
    return std::min(1.0f, m_blendTime / m_blendDuration);
}

/**
 * @brief Updates blending state
 */
void Mesh::UpdateBlending(float deltaTime)
{
    if (!m_isBlending) return;

    m_blendTime += deltaTime;

    if (m_blendTime >= m_blendDuration) {
        m_currentAnimationName = m_targetAnimationName;
        m_currentAnimationTime = m_targetAnimationTime;
        m_isBlending = false;

        std::cout << "Blend complete - now playing '" << m_currentAnimationName << "'" << std::endl;
    }
}



void Mesh::ExtractTRS(const glm::mat4& m, glm::vec3& translation, glm::quat& rotation, glm::vec3& scale)
{
    // Translation (column-major)
    translation = glm::vec3(m[3]);

    // Scale
    scale.x = glm::length(glm::vec3(m[0]));
    scale.y = glm::length(glm::vec3(m[1]));
    scale.z = glm::length(glm::vec3(m[2]));

    // Guard against zero scale
    if (scale.x == 0.0f || scale.y == 0.0f || scale.z == 0.0f)
    {
        rotation = glm::quat(1, 0, 0, 0);
        return;
    }

    // Rotation matrix (remove scale)
    glm::mat3 rotMat;
    rotMat[0] = glm::vec3(m[0]) / scale.x;
    rotMat[1] = glm::vec3(m[1]) / scale.y;
    rotMat[2] = glm::vec3(m[2]) / scale.z;

    rotation = glm::quat_cast(rotMat);
}


/**
 * @brief Blends two bone transformation matrices
 */
glm::mat4 Mesh::BlendBoneTransforms(
    const glm::mat4& transform1,
    const glm::mat4& transform2,
    float blendFactor)
{
    glm::vec3 translation1, translation2;
    glm::vec3 scale1, scale2;
    glm::quat rotation1, rotation2;

    ExtractTRS(transform1, translation1, rotation1, scale1);
    ExtractTRS(transform2, translation2, rotation2, scale2);
    
    // Ensure shortest quaternion path
    if (glm::dot(rotation1, rotation2) < 0.0f)
        rotation2 = -rotation2;

    glm::vec3 blendedTranslation = glm::mix(translation1, translation2, blendFactor);
    glm::quat blendedRotation = glm::slerp(rotation1, rotation2, blendFactor);
    glm::vec3 blendedScale = glm::mix(scale1, scale2, blendFactor);

    glm::mat4 result(1.0f);
    result = glm::translate(result, blendedTranslation);
    result *= glm::toMat4(blendedRotation);
    result = glm::scale(result, blendedScale);

    return result;
}



/**
 * @brief Renders the animated mesh
 */
void Mesh::Draw(int programId, long double dt, PathFollower& pathfollow)
{
    glm::vec3 diffuseColor{ 0.7, 0.7, 0.5 };
    glm::vec3 specularColor{ 0.5, 0.5, 0.5 };
    float shininess = 1.0f;

    float deltaTime = (float)dt;
    if (deltaTime > 0.1f) {
        deltaTime = 0.1f;
    }

    m_LastDeltaTime = deltaTime;


    //N-NEW
//    deltaTime *= m_animationSpeedMultiplier;

    const AnimationClip* currentClip = GetCurrentClip();
    if (!currentClip) {
        std::cout << "No animation clip set!" << std::endl;
        return;
    }

    const Animation& currentTrack = m_animationTracks[currentClip->trackIndex];

    if (m_isBlending) {
        UpdateBlending(deltaTime);

        const AnimationClip* targetClip = nullptr;
        auto it = m_animationClips.find(m_targetAnimationName);
        if (it != m_animationClips.end()) {
            targetClip = &it->second;
        }

        if (targetClip) {
            const Animation& targetTrack = m_animationTracks[targetClip->trackIndex];

            float targetClipDuration = (targetClip->endTime - targetClip->startTime) / targetTrack.ticksPerSecond;
            m_targetAnimationTime += deltaTime * m_animationSpeedMultiplier;
            
            if (targetClip->loop && m_targetAnimationTime > targetClipDuration) {
                m_targetAnimationTime = fmod(m_targetAnimationTime, targetClipDuration);
            }
        }
    }

    m_currentAnimationTime += deltaTime * m_animationSpeedMultiplier;

    float clipDuration = (currentClip->endTime - currentClip->startTime) / currentTrack.ticksPerSecond;

    if (currentClip->loop) {
        if (m_currentAnimationTime > clipDuration) {
            m_currentAnimationTime = fmod(m_currentAnimationTime, clipDuration);
        }
    }
    else {
        if (m_currentAnimationTime > clipDuration) {
            m_currentAnimationTime = clipDuration;
        }
    }

    PathFollower::PathState state = pathfollow.getCurrentState();

    glm::mat4 objectTr = glm::mat4(1.0f);
    objectTr[0] = glm::vec4(state.right * scaleFactor, 0.0f);
    objectTr[1] = glm::vec4(state.up * scaleFactor, 0.0f);
    objectTr[2] = glm::vec4(state.forward * scaleFactor, 0.0f);
    objectTr[3] = glm::vec4(state.position, 1.0f);


    m_ModelTr = objectTr;
    m_ModelTrInv = glm::inverse(objectTr);


    CalFinalTransformations();

    CHECKERROR;
    glBindVertexArray(m_vao);
    CHECKERROR;

    for (int i = 0; i < m_boneInfo.size(); ++i) {
        std::string name = "gBones[" + std::to_string(i) + "]";
        int loc = glGetUniformLocation(programId, name.c_str());
        glUniformMatrix4fv(loc, 1, GL_TRUE, &m_boneInfo[i].FinalTransformation[0][0]);
        CHECKERROR;
    }

    int vertices = 0;
    int indices = 0;
    for (unsigned int i = 0; i < m_scene->mNumMeshes; ++i) {
        int matIndex = m_scene->mMeshes[i]->mMaterialIndex;

        int loc = glGetUniformLocation(programId, "diffuse");
        glUniform3fv(loc, 1, &diffuseColor[0]);

        loc = glGetUniformLocation(programId, "specular");
        glUniform3fv(loc, 1, &specularColor[0]);

        loc = glGetUniformLocation(programId, "shininess");
        glUniform1f(loc, shininess);
        CHECKERROR;
        
        /*
        PathFollower::PathState state = pathfollow.getCurrentState();

        glm::mat4 objectTr = glm::mat4(1.0f);
        objectTr[0] = glm::vec4(state.right * scaleFactor, 0.0f);
        objectTr[1] = glm::vec4(state.up * scaleFactor, 0.0f);
        objectTr[2] = glm::vec4(state.forward * scaleFactor, 0.0f);
        objectTr[3] = glm::vec4(state.position, 1.0f);
        */

        //std::cout << "Character Position: " << state.position.x << ", " << state.position.y << ", " << state.position.z << std::endl;
        //std::cout << "Forward: " << state.forward.x << ", " << state.forward.y << ", " << state.forward.z << std::endl;

        loc = glGetUniformLocation(programId, "ModelTr");
        glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(objectTr));

        glm::mat4 inv = m_ModelTrInv; //glm::inverse(objectTr);
        loc = glGetUniformLocation(programId, "NormalTr");
        glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(inv));

        int curr_indices = m_scene->mMeshes[i]->mNumFaces * 3;
        glDrawElementsBaseVertex(GL_TRIANGLES, curr_indices, GL_UNSIGNED_INT,
            (void*)(sizeof(unsigned int) * indices), vertices);

        CHECKERROR;

        vertices += m_scene->mMeshes[i]->mNumVertices;
        indices += curr_indices;
    }

    CHECKERROR;
    glBindVertexArray(0);
}

/**
 * @brief Creates and configures the Vertex Array Object (VAO) for rendering
 */
void Mesh::MakeVAO()
{
    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);

    GLuint Pbuff;
    glGenBuffers(1, &Pbuff);
    glBindBuffer(GL_ARRAY_BUFFER, Pbuff);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * m_pnt.size(),
        &m_pnt[0][0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if (m_norm.size() > 0) {
        GLuint Nbuff;
        glGenBuffers(1, &Nbuff);
        glBindBuffer(GL_ARRAY_BUFFER, Nbuff);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * m_norm.size(),
            &m_norm[0][0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    if (m_tex.size() > 0) {
        GLuint Tbuff;
        glGenBuffers(1, &Tbuff);
        glBindBuffer(GL_ARRAY_BUFFER, Tbuff);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * m_tex.size(),
            &m_tex[0][0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    if (m_tan.size() > 0) {
        GLuint Dbuff;
        glGenBuffers(1, &Dbuff);
        glBindBuffer(GL_ARRAY_BUFFER, Dbuff);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * m_tan.size(),
            &m_tan[0][0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    GLuint Bbuff;
    glGenBuffers(1, &Bbuff);
    glBindBuffer(GL_ARRAY_BUFFER, Bbuff);
    glBufferData(GL_ARRAY_BUFFER, sizeof(BoneData) * m_bones.size(), &m_bones[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(4);
    glVertexAttribIPointer(4, MAX_NUM_BONES_PER_VERTEX, GL_INT, sizeof(BoneData), (const GLvoid*)0);
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, MAX_NUM_BONES_PER_VERTEX, GL_FLOAT, GL_FALSE, sizeof(BoneData),
        (const GLvoid*)(MAX_NUM_BONES_PER_VERTEX * sizeof(int)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    GLuint Ibuff;
    glGenBuffers(1, &Ibuff);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Ibuff);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * m_indices.size(),
        &m_indices[0], GL_STATIC_DRAW);

    glBindVertexArray(0);
}

/**
 * @brief Initializes mesh data from the loaded scene
 */
void Mesh::InitMesh(const std::string& path)
{
    int num_vertices = 0;

    for (unsigned int i = 0; i < m_scene->mNumMeshes; i++) {
        aiMesh* mesh = m_scene->mMeshes[i];
        std::cout << "* Mesh  " << i << mesh->mName.C_Str() << std::endl;
    }

    for (unsigned int i = 0; i < m_scene->mNumMeshes; ++i) {
        const aiMesh* m = m_scene->mMeshes[i];

        std::cout << std::endl;
        std::cout << "Check the number of calls " << m->mNumVertices << std::endl;

        for (unsigned int j = 0; j < m->mNumVertices; ++j) {
            aiVector3D& p = m->mVertices[j];
            m_pnt.push_back(glm::vec4(p.x, p.y, p.z, 1.0));

            if (m->mNormals) {
                aiVector3D& n = m->mNormals[j];
                m_norm.push_back(glm::vec3(n.x, n.y, n.z));
            }

            if (m->mTangents) {
                aiVector3D& tan = m->mTangents[j];
                m_tan.push_back(glm::vec3(tan.x, tan.y, tan.z));
            }

            const aiVector3D& tx = m->HasTextureCoords(0) ? m->mTextureCoords[0][j] : aiVector3D(0.0f, 0.0f, 0.0f);
            m_tex.push_back(glm::vec2(tx.x, tx.y));

            m_bones.push_back(BoneData());
        }

        std::cout << std::endl;
        std::cout << "BONE DATA " << m_bones.size() << std::endl;

        InitBone(m, num_vertices);

        for (unsigned int j = 0; j < m->mNumFaces; ++j) {
            const aiFace& tri = m->mFaces[j];
            m_indices.push_back(tri.mIndices[0]);
            m_indices.push_back(tri.mIndices[1]);
            m_indices.push_back(tri.mIndices[2]);
        }

        num_vertices += m->mNumVertices;
    }
}

/**
 * @brief Initializes bone data for a specific mesh
 */
void Mesh::InitBone(const aiMesh* mesh, int index)
{
    std::cout << "Number of bones " << mesh->mNumBones << " No.of vertices" << index << std::endl;

    for (unsigned int i = 0; i < mesh->mNumBones; ++i)
    {
        const aiBone* b = mesh->mBones[i];
        int bone_id = GetBoneId(b);

        if (bone_id >= (int)m_boneInfo.size())
        {
            m_boneInfo.resize(bone_id + 1);
            m_boneInfo[bone_id] = BoneInfo(b->mOffsetMatrix);
        }

        for (unsigned int j = 0; j < b->mNumWeights; ++j)
        {
            const aiVertexWeight& wt = b->mWeights[j];
            int vertexId = index + wt.mVertexId;
            m_bones[vertexId].AddBoneData(bone_id, wt.mWeight);
        }
    }
}


/**
 * @brief Gets or creates a unique ID for a bone
 */
int Mesh::GetBoneId(const aiBone* b)
{
    std::string name(b->mName.C_Str());

    auto it = m_name_index.find(name);
    if (it == m_name_index.end())
        m_name_index[name] = (int)m_name_index.size();

    return m_name_index[name];
}

/**
 * @brief Prints all bone names and their corresponding IDs
 */
void Mesh::PrintValues()
{
    std::cout << "Values of the map:" << std::endl;
    for (std::map<std::string, int>::iterator it = m_name_index.begin(); it != m_name_index.end(); ++it) {
        std::cout << " key ---> " << it->first << std::endl;
        std::cout << " value --> " << it->second << std::endl;
    }
}

/**
 * @brief Calculates final bone transformations for the current animation time
 */
void Mesh::CalFinalTransformations()
{
    const AnimationClip* currentClip = GetCurrentClip();
    if (!currentClip) return;

    const Animation& currentTrack = m_animationTracks[currentClip->trackIndex];

    float timeInTicks = m_currentAnimationTime * currentTrack.ticksPerSecond;
    float currentAnimationTime = GetClipLocalTime(timeInTicks, *currentClip);
    currentAnimationTime = std::max(currentClip->startTime, std::min(currentAnimationTime, currentClip->endTime));

    glm::mat4 identity = glm::mat4(1.0f);
    glm::mat4 globalInverseTransform = convert(m_inverseTrans);

    if (m_isBlending) {
        std::vector<glm::mat4> currentBoneTransforms(m_boneInfo.size());
        std::vector<glm::mat4> targetBoneTransforms(m_boneInfo.size());

        ReadNodeHierarchy(currentAnimationTime, m_scene->mRootNode, identity, currentTrack);
        for (size_t i = 0; i < m_boneInfo.size(); i++) {
            currentBoneTransforms[i] = convert(m_boneInfo[i].FinalTransformation);
        }

        auto it = m_animationClips.find(m_targetAnimationName);
        if (it != m_animationClips.end()) {
            const AnimationClip& targetClip = it->second;
            const Animation& targetTrack = m_animationTracks[targetClip.trackIndex];

            float targetTimeInTicks = m_targetAnimationTime * targetTrack.ticksPerSecond;
            float targetAnimTime = GetClipLocalTime(targetTimeInTicks, targetClip);
            targetAnimTime = std::max(targetClip.startTime, std::min(targetAnimTime, targetClip.endTime));

            ReadNodeHierarchy(targetAnimTime, m_scene->mRootNode, identity, targetTrack);
            for (size_t i = 0; i < m_boneInfo.size(); i++) {
                targetBoneTransforms[i] = convert(m_boneInfo[i].FinalTransformation);
            }

            float blendFactor = GetBlendProgress();
            for (size_t i = 0; i < m_boneInfo.size(); i++) {
                glm::mat4 blendedTransform = BlendBoneTransforms(
                    currentBoneTransforms[i],
                    targetBoneTransforms[i],
                    blendFactor
                );
                m_boneInfo[i].FinalTransformation = convert(blendedTransform);
            }

            static int frameCount = 0;
            if (frameCount % 60 == 0) {
                std::cout << "BLENDING [" << (blendFactor * 100.0f) << "%]: "
                    << currentClip->name << " -> " << targetClip.name
                    << " | Track " << currentClip->trackIndex
                    << " -> Track " << targetClip.trackIndex << std::endl;
            }
            frameCount++;
        }
    }
    else {
        ReadNodeHierarchy(currentAnimationTime, m_scene->mRootNode, identity, currentTrack);

        /*
        static int frameCount = 0;
        if (frameCount % 60 == 0) {
            std::cout << "Clip: " << currentClip->name
                << " | Track: " << currentClip->trackIndex
                << " | Time: " << m_currentAnimationTime
                << " | Frame Time: " << currentAnimationTime
                << " | Range: " << currentClip->startTime << "-" << currentClip->endTime << std::endl;
        }
        frameCount++;
        */
    }

    ApplyIK();

    for (size_t i = 0; i < m_boneInfo.size(); i++) {
        glm::mat4 finalTrans = convert(m_boneInfo[i].FinalTransformation);
        glm::mat4 offset = convert(m_boneInfo[i].OffsetMatrix);
        glm::mat4 result = globalInverseTransform * finalTrans * offset;
        m_boneInfo[i].FinalTransformation = convert(result);
    }


}



/**
 * @brief Recursively traverses the scene graph to calculate bone transformations
 */
void Mesh::ReadNodeHierarchy(float animationTime, const aiNode* node,
    const glm::mat4& parentTransform, const Animation& track)
{
    std::string nodeName(node->mName.C_Str());
    glm::mat4 nodeTransformation = convert(node->mTransformation);

    auto it = m_name_index.find(nodeName);
    if (it != m_name_index.end())
    {
        int boneIndex = it->second;

        if (boneIndex >= 0 &&
            boneIndex < (int)track.boneAnimations.size() &&
            boneIndex < (int)m_boneInfo.size())
        {
            const BoneAnimation& boneAnim = track.boneAnimations[boneIndex];

            glm::vec3 translation = CalcInterpolatedPosition(animationTime, boneAnim);
            glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), translation);

            glm::quat rotation = CalcInterpolatedRotation(animationTime, boneAnim);
            glm::mat4 rotationMatrix = glm::toMat4(rotation);

            glm::vec3 scale = CalcInterpolatedScaling(animationTime, boneAnim);
            glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), scale);

            nodeTransformation = translationMatrix * rotationMatrix * scaleMatrix;
        }
        // else: node is in map but we can't safely evaluate it for this track/boneInfo
    }

    glm::mat4 globalTransformation = parentTransform * nodeTransformation;
    nodeGlobalTransforms[nodeName] = globalTransformation;

    if (it != m_name_index.end())
    {
        int boneIndex = it->second;

        if (boneIndex >= 0 && boneIndex < (int)m_boneInfo.size())
        {
            m_boneInfo[boneIndex].FinalTransformation = convert(globalTransformation);
        }
    }

    for (unsigned int i = 0; i < node->mNumChildren; i++)
    {
        ReadNodeHierarchy(animationTime, node->mChildren[i], globalTransformation, track);
    }
}

/*
void Mesh::ReadNodeHierarchy(float animationTime, const aiNode* node,
    const glm::mat4& parentTransform, const Animation& track)
{
    std::string nodeName(node->mName.C_Str());
    glm::mat4 nodeTransformation = convert(node->mTransformation);

    auto it = m_name_index.find(nodeName);
    if (it != m_name_index.end()) {
        int boneIndex = it->second;
        const BoneAnimation& boneAnim = track.boneAnimations[boneIndex];

        glm::vec3 translation = CalcInterpolatedPosition(animationTime, boneAnim);
        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), translation);

        glm::quat rotation = CalcInterpolatedRotation(animationTime, boneAnim);
        glm::mat4 rotationMatrix = glm::toMat4(rotation);

        glm::vec3 scale = CalcInterpolatedScaling(animationTime, boneAnim);
        glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), scale);

        nodeTransformation = translationMatrix * rotationMatrix * scaleMatrix;
    }

    glm::mat4 globalTransformation = parentTransform * nodeTransformation;
    nodeGlobalTransforms[nodeName] = globalTransformation;

    if (it != m_name_index.end()) {
        int boneIndex = it->second;
        m_boneInfo[boneIndex].FinalTransformation = convert(globalTransformation);
    }

    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        ReadNodeHierarchy(animationTime, node->mChildren[i], globalTransformation, track);
    }
}
*/


/**
 * @brief Interpolates bone position at a given animation time
 */
glm::vec3 Mesh::CalcInterpolatedPosition(float time, const BoneAnimation& anim)
{
    if (anim.positions.empty()) {
        return glm::vec3(0.0f);
    }

    if (anim.positions.size() == 1)
        return anim.positions[0].position;

    for (size_t i = 0; i < anim.positions.size() - 1; i++) {
        if (time < anim.positions[i + 1].timeStamp) {
            float delta = anim.positions[i + 1].timeStamp - anim.positions[i].timeStamp;
            float factor = (time - anim.positions[i].timeStamp) / delta;
            return glm::mix(anim.positions[i].position, anim.positions[i + 1].position, factor);
        }
    }

    return anim.positions.back().position;
}

/**
 * @brief Interpolates bone rotation at a given animation time
 */
glm::quat Mesh::CalcInterpolatedRotation(float time, const BoneAnimation& anim)
{
    if (anim.rotations.empty())
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    if (anim.rotations.size() == 1)
        return anim.rotations[0].rotation;

    for (size_t i = 0; i < anim.rotations.size() - 1; i++) {
        if (time < anim.rotations[i + 1].timeStamp) {
            float delta = anim.rotations[i + 1].timeStamp - anim.rotations[i].timeStamp;
            float factor = (time - anim.rotations[i].timeStamp) / delta;

            glm::quat q1 = anim.rotations[i].rotation;
            glm::quat q2 = anim.rotations[i + 1].rotation;

            if (glm::dot(q1, q2) < 0.0f) {
                q2 = -q2;
            }

            float t = static_cast<float>(factor);
            glm::quat result = glm::slerp(q1, q2, t);

            return glm::normalize(result);
        }
    }

    return anim.rotations.back().rotation;
}

/**
 * @brief Interpolates bone scale at a given animation time
 */
glm::vec3 Mesh::CalcInterpolatedScaling(float time, const BoneAnimation& anim)
{
    if (anim.scales.empty()) {
        return glm::vec3(1.0f);
    }

    if (anim.scales.size() == 1)
        return anim.scales[0].scale;

    for (size_t i = 0; i < anim.scales.size() - 1; i++) {
        if (time < anim.scales[i + 1].timeStamp) {
            float delta = anim.scales[i + 1].timeStamp - anim.scales[i].timeStamp;
            float factor = (time - anim.scales[i].timeStamp) / delta;
            return glm::mix(anim.scales[i].scale, anim.scales[i + 1].scale, factor);
        }
    }

    return anim.scales.back().scale;
}

/**
 * @brief Converts Assimp vector to GLM vector
 */
glm::vec3 Mesh::toVec3(const aiVector3D& v) {
    return glm::vec3(v.x, v.y, v.z);
}

/**
 * @brief Converts Assimp quaternion to GLM quaternion
 */
glm::quat Mesh::toQuat(const aiQuaternion& q) {
    return glm::quat(q.w, q.x, q.y, q.z);
}

/**
 * @brief Prints all bone names and IDs to console
 */
void Mesh::DataPrint()
{
    std::cout << " PRINTING DATA " << std::endl;

    for (const auto& p : m_name_index) {
        std::cout << p.first << " : " << p.second << std::endl;
    }
}

/**
 * @brief Sets up VAO and VBO for line rendering
 */
void Mesh::SetupLineVAO(GLuint& VAO, GLuint& VBO)
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(glm::vec3), nullptr, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

    glBindVertexArray(0);
}

/**
 * @brief Draws a single line between two points
 */
void Mesh::DrawLine(const glm::vec3& start, const glm::vec3& end, GLuint shader, GLuint VAO, GLuint VBO)
{
    glm::vec3 lineVertices[2] = { start, end };

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(lineVertices), lineVertices);

    glUseProgram(shader);
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);
}

/**
 * @brief Renders the animated skeleton as debug lines
 */
void Mesh::DebugSkeleton(const glm::mat4& viewProj, GLuint shader)
{
    static GLuint lineVAO = 0;
    static GLuint lineVBO = 0;

    if (lineVAO == 0) {
        SetupLineVAO(lineVAO, lineVBO);
    }

    glUseProgram(shader);
    int loc = glGetUniformLocation(shader, "viewProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, &viewProj[0][0]);

    glDisable(GL_DEPTH_TEST);
    glLineWidth(3.0f);

    glm::mat4 modelTr = Translate(0.f, 0.f, -2.5f) *
        Rotate(0, 90.f) *
        Scale(0.1f, 0.1f, 0.1f);

    DrawSkeletonRecursive(m_scene->mRootNode, modelTr, shader, lineVAO, lineVBO);

    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);
}

/**
 * @brief Recursively draws skeleton bones as lines
 */
void Mesh::DrawSkeletonRecursive(const aiNode* node, const glm::mat4& modelTransform,
    GLuint shader, GLuint lineVAO, GLuint lineVBO)
{
    std::string nodeName(node->mName.C_Str());

    auto boneIt = m_name_index.find(nodeName);

    if (boneIt != m_name_index.end()) {
        auto transformIt = nodeGlobalTransforms.find(nodeName);
        if (transformIt != nodeGlobalTransforms.end()) {
            glm::mat4 globalTransform = transformIt->second;
            glm::vec3 currentBonePos = glm::vec3(globalTransform[3]);
            currentBonePos = glm::vec3(modelTransform * glm::vec4(currentBonePos, 1.0f));

            if (node->mParent) {
                std::string parentName(node->mParent->mName.C_Str());
                auto parentBoneIt = m_name_index.find(parentName);

                if (parentBoneIt != m_name_index.end()) {
                    auto parentTransformIt = nodeGlobalTransforms.find(parentName);
                    if (parentTransformIt != nodeGlobalTransforms.end()) {
                        glm::vec3 parentBonePos = glm::vec3(parentTransformIt->second[3]);
                        parentBonePos = glm::vec3(modelTransform * glm::vec4(parentBonePos, 1.0f));

                        DrawLine(parentBonePos, currentBonePos, shader, lineVAO, lineVBO);
                    }
                }
            }
        }
    }

    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        DrawSkeletonRecursive(node->mChildren[i], modelTransform, shader, lineVAO, lineVBO);
    }
}



static float SafeAcos(float x)
{
    return acosf(glm::clamp(x, -1.0f, 1.0f));
}


/**
 * @brief Sets up the inverse kinematics chain with optional constraints
 * @param chain Ordered list of bone names from root to end-effector
 * @param constraints Map of bone names to rotational constraints
 */
void Mesh::SetupIKChain(
    const std::vector<std::string>& chain,
    const std::unordered_map<std::string, IKJointConstraint>& constraints)
{
    m_IKChain = chain;
    m_IKConstraints = constraints;
}

/**
 * @brief Enables or disables IK solving
 * @param enable True to enable IK, false to disable
 */
void Mesh::EnableIK(bool enable)
{
    m_IKEnabled = enable;
    if (!m_IKEnabled) {
        m_IKActive = false;
        m_IKTimer = 0.0f;
    }
}

/**
 * @brief Starts IK to move the end-effector to a world-space target
 * @param worldTarget Target position in world coordinates
 * @param durationSec Duration of IK interpolation in seconds
 */
void Mesh::StartIKReachWorld(const glm::vec3& worldTarget, float durationSec)
{
    m_IKTargetWorld = worldTarget;
    m_IKDuration = glm::max(0.01f, durationSec);
    m_IKTimer = 0.0f;
    m_IKActive = true;

    std::cout << "[IK] Chain:\n";
    for (auto& n : m_IKChain) std::cout << "  " << n << "\n";

}

/**
 * @brief Stops the current IK operation
 */
void Mesh::StopIK()
{
    m_IKActive = false;
    m_IKTimer = 0.0f;
}


/**
 * @brief Get the global transformation matrix of a node by name.
 *
 * @param name Name of the node.
 * @return glm::mat4 The global transform of the node. Returns identity matrix if node is not found.
 */
glm::mat4 Mesh::GetNodeGlobal(const std::string& name) const
{
    auto it = nodeGlobalTransforms.find(name);
    if (it == nodeGlobalTransforms.end()) return glm::mat4(1.0f);
    return it->second;
}

/**
 * @brief Get the world position of a node by name.
 *
 * @param name Name of the node.
 * @return glm::vec3 The position of the node in world/mesh space.
 */
glm::vec3 Mesh::GetNodePosition(const std::string& name) const
{
    glm::mat4 g = GetNodeGlobal(name);
    return glm::vec3(g[3]);
}

/**
 * @brief Rotate all nodes in the IK chain after a given joint.
 *
 * @param jointIndex Index of the joint in the IK chain.
 * @param jointPos Position of the joint in world/mesh space.
 * @param axisGlobal Axis to rotate around in world/mesh space.
 * @param angleRad Rotation angle in radians.
 *
 * This function applies a rotation around a joint and propagates it down
 * the IK chain to affect all child nodes. Updates bone transformations
 * if the node is part of a skinned mesh.
 */
void Mesh::RotateChainFromJoint(
    int jointIndex,
    const glm::vec3& jointPos,
    const glm::vec3& axisGlobal,
    float angleRad)
{
    if (fabs(angleRad) < 1e-7f) return;

    glm::vec3 axis = glm::normalize(axisGlobal);

    glm::mat4 T = glm::translate(glm::mat4(1.0f), jointPos);
    glm::mat4 Ti = glm::translate(glm::mat4(1.0f), -jointPos);
    glm::mat4 R = glm::toMat4(glm::angleAxis(angleRad, axis));

    glm::mat4 M = T * R * Ti;

    // Rotate every node after this joint in the IK chain
    for (int k = jointIndex + 1; k < (int)m_IKChain.size(); ++k) {
        const std::string& nodeName = m_IKChain[k];

        auto it = nodeGlobalTransforms.find(nodeName);
        if (it != nodeGlobalTransforms.end()) {
            it->second = M * it->second;

            // Keep m_boneInfo consistent if this node is a skinned bone
            auto bi = m_name_index.find(nodeName);
            if (bi != m_name_index.end()) {
                int boneIndex = bi->second;
                m_boneInfo[boneIndex].FinalTransformation = convert(it->second);
            }
        }
    }
}

/**
 * @brief Perform the IK solving step for the mesh.
 *
 * This function applies Inverse Kinematics (IK) to move the end-effector
 * towards the target. Supports pole constraints to prevent unnatural elbow
 * bending. Uses interpolation to move smoothly over time.
 */
void Mesh::ApplyIK()
{
    if (!m_IKEnabled || !m_IKActive) return;
    if (m_IKChain.size() < 2) return;

    // DEBUG (TEMP): print chain node positions
    static int dbg = 0;
    if ((dbg++ % 60) == 0) { // print once per ~60 frames
        std::cout << "---- IK Chain positions ----\n";
        for (auto& n : m_IKChain) {
            glm::vec3 p = GetNodePosition(n);
            std::cout << n << ": " << p.x << ", " << p.y << ", " << p.z << "\n";
        }
    }


    // Convert target from world space -> mesh local space
    glm::vec3 targetLocal = glm::vec3(m_ModelTrInv * glm::vec4(m_IKTargetWorld, 1.0f));

    const std::string& endName = m_IKChain.back();
    glm::vec3 effLocal = GetNodePosition(endName);

    // Capture starting effector position once (for interpolation)
    if (m_IKTimer <= 0.0f) {
        m_IKStartEffectorLocal = effLocal;
    }

    // Interpolate desired effector motion (rubric requirement)
    m_IKTimer += m_LastDeltaTime;
    float alpha = glm::clamp(m_IKTimer / m_IKDuration, 0.0f, 1.0f);

    glm::vec3 desiredTarget = glm::mix(m_IKStartEffectorLocal, targetLocal, alpha);
    glm::vec3 biasDir = glm::vec3(0, 0, -1); // try +Z then -Z
    desiredTarget += 0.05f * biasDir;


    // --- Pole constraint (strong elbow side control) ---
    // --- Strong pole constraint to prevent backward elbow ---
    if (m_IKChain.size() >= 5) {
        const std::string& shoulderName = m_IKChain[1];
        const std::string& elbowName = m_IKChain[3];
        const std::string& handName = m_IKChain[4];

        glm::vec3 S = GetNodePosition(shoulderName);
        glm::vec3 E = GetNodePosition(elbowName);
        glm::vec3 H = GetNodePosition(handName);

        glm::vec3 sh = H - S;
        if (glm::length(sh) > 1e-6f) {
            sh = glm::normalize(sh);

            // Choose a pole direction in mesh-local space.
            // Since your elbow Z is going negative (backwards), we want to bias toward +Z.
            glm::vec3 pole = glm::vec3(0, 0, 1);

            glm::vec3 n = glm::cross(sh, pole);
            if (glm::length(n) > 1e-6f) {
                n = glm::normalize(n);

                float side = glm::dot(E - S, n);

                // If elbow is on the wrong side, push the desiredTarget to the correct side
                if (side < 0.0f) {
                    desiredTarget += 0.5f * n; // tune 0.2 to 0.8
                }
            }
        }
    }




    SolveCCD(desiredTarget);

    // Optional: if you want to stop once it reaches the final target:
    // if (alpha >= 1.0f && glm::length(GetNodePosition(endName) - targetLocal) < 0.10f) {
    //     m_IKActive = false;
    // }
}


/**
 * @brief Solve IK using the Cyclic Coordinate Descent (CCD) method.
 *
 * @param targetLocal Target position in mesh-local coordinates.
 *
 * Iteratively rotates joints along the IK chain to bring the end-effector
 * closer to the target. Supports hinge and ball joints with constraints,
 * and includes pole constraint for natural elbow bending.
 */
void Mesh::SolveCCD(const glm::vec3& targetLocal)
{
    const int maxIters = 12;
    const float eps = 0.10f;

    const std::string& endName = m_IKChain.back();

    for (int iter = 0; iter < maxIters; ++iter) {
        glm::vec3 effPos = GetNodePosition(endName);
        if (glm::length(effPos - targetLocal) < eps) break;

        // CCD: iterate joints from end-1 down to root
        for (int j = (int)m_IKChain.size() - 2; j >= 0; --j) {
            const std::string& jointName = m_IKChain[j];

            glm::vec3 jointPos = GetNodePosition(jointName);
            effPos = GetNodePosition(endName);

            glm::vec3 toEff = effPos - jointPos;
            glm::vec3 toTar = targetLocal - jointPos;

            float lenEff = glm::length(toEff);
            float lenTar = glm::length(toTar);
            if (lenEff < 1e-6f || lenTar < 1e-6f) continue;

            toEff /= lenEff;
            toTar /= lenTar;

            IKJointConstraint c;
            auto cit = m_IKConstraints.find(jointName);
            if (cit != m_IKConstraints.end()) c = cit->second;

            float maxStepRad = glm::radians(c.maxStepDeg);

            if (c.type == IKJointConstraint::HINGE) {
                // Hinge axis in global space: jointRotation * hingeAxisLocal
                glm::mat4 jointG = GetNodeGlobal(jointName);
                glm::mat3 jointR = glm::mat3(jointG);
                glm::vec3 hingeAxisGlobal = glm::normalize(jointR * c.hingeAxisLocal);

                // signed angle around hinge axis
                float signedAngle = atan2f(
                    glm::dot(hingeAxisGlobal, glm::cross(toEff, toTar)),
                    glm::dot(toEff, toTar)
                );

                // priority + clamp per step
                signedAngle *= c.weight;
                signedAngle = glm::clamp(signedAngle, -maxStepRad, maxStepRad);

                RotateChainFromJoint(j, jointPos, hingeAxisGlobal, signedAngle);
            }
            else {
                // Ball joint
                glm::vec3 axis = glm::cross(toEff, toTar);
                float axisLen = glm::length(axis);
                if (axisLen < 1e-7f) continue;
                axis /= axisLen;

                float angle = SafeAcos(glm::dot(toEff, toTar));

                // priority + clamp per step
                angle *= c.weight;
                angle = glm::clamp(angle, -maxStepRad, maxStepRad);

                RotateChainFromJoint(j, jointPos, axis, angle);
            }
        }


        // ---- Pole constraint (prevents elbow bending backward) ----
        if (m_IKChain.size() >= 4) {
            // Typical chain indices:
            // 0 spine/shoulderRoot, 1 shoulder, 2 upperarm, 3 forearm(elbow), 4 hand...
            const std::string& shoulderName = m_IKChain[1];
            const std::string& elbowName = m_IKChain[3];
            const std::string& handName = m_IKChain[4]; // or endName if you prefer

            glm::vec3 S = GetNodePosition(shoulderName);
            glm::vec3 E = GetNodePosition(elbowName);
            glm::vec3 H = GetNodePosition(handName);

            glm::vec3 sh = glm::normalize(H - S);
            if (glm::length(sh) > 1e-6f) {

                // Choose a pole direction (tune this!)
                // Try (0,0,1) then (0,0,-1) then (1,0,0) etc.
                glm::vec3 pole = glm::vec3(0, 0, 1);

                // Create a plane normal using sh and pole
                glm::vec3 n = glm::cross(sh, pole);
                float nlen = glm::length(n);
                if (nlen > 1e-6f) {
                    n /= nlen;

                    // Project elbow onto plane to remove "wrong side" bending
                    glm::vec3 Eproj = ProjectPointOntoPlane(E, S, n);

                    // Nudge target slightly so CCD prefers that side
                    // This is a stable trick without rewriting rotations.
                    glm::vec3 correction = (Eproj - E);
                    // small correction only
                    targetLocal + 0.15f * correction; // <-- (see note below)
                }
            }
        }


    }
}


/**
 * @brief Project a point onto a plane.
 *
 * @param p Point to project.
 * @param planePoint A point on the plane.
 * @param planeNormal Normal vector of the plane (assumed normalized).
 * @return glm::vec3 Projected point on the plane.
 */
glm::vec3 Mesh::ProjectPointOntoPlane(const glm::vec3& p, const glm::vec3& planePoint, const glm::vec3& planeNormal)
{
    float d = glm::dot(p - planePoint, planeNormal);
    return p - d * planeNormal;
}

/**
 * @brief Set the IK target position in world coordinates.
 *
 * @param worldTarget Target position in world space.
 */
void Mesh::SetIKTargetWorld(const glm::vec3& worldTarget)
{
    m_IKTargetWorld = worldTarget;
}
