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
#include "RigidBody.h"

/**
 * @brief Converts an Assimp 4x4 matrix to a GLM 4x4 matrix.
 * @param m Assimp aiMatrix4x4
 * @return glm::mat4 Equivalent GLM 4x4 matrix
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
 * @brief Converts an Assimp 3x3 matrix to a GLM 4x4 matrix.
 * @param m Assimp aiMatrix3x3
 * @return glm::mat4 Equivalent GLM 4x4 matrix (homogeneous)
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
 * @brief Converts a GLM 4x4 matrix to an Assimp 4x4 matrix.
 * @param m GLM 4x4 matrix
 * @return aiMatrix4x4 Equivalent Assimp 4x4 matrix
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
 * @brief Default constructor for the Mesh class.
 * Initializes internal pointers to null.
 */
Mesh::Mesh()
    : gCubeRef(nullptr)
{   
}

/**
 * @brief Destructor for the Mesh class.
 * Cleans up allocated resources if needed.
 */
Mesh::~Mesh()
{
}


/**
 * @brief Sets the speed multiplier for animations.
 * @param speedMultiplier Factor by which animation speed is scaled.
 */
void Mesh::SetAnimationSpeed(float speedMultiplier)
{
    m_animationSpeedMultiplier = speedMultiplier;
}

/**
 * @brief Gets the current animation speed multiplier.
 * @return float Current multiplier for animation speed.
 */
float Mesh::GetAnimationSpeed() const
{
    return m_animationSpeedMultiplier;
}



/**
 * @brief Loads a 3D mesh from a file using Assimp.
 * @param path Path to the mesh file (FBX, OBJ, etc.)
 *
 * Initializes the scene, loads mesh data, and sets up VAOs for rendering.
 */
void Mesh::LoadMesh(const std::string& path)
{
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

    MakeVAO();

}


/**
 * @brief Draws the mesh.
 * @param programId OpenGL shader program ID to use
 * @param dt Time delta (seconds) since last frame
 *
 * Applies skeletal animation, sets uniforms for diffuse/specular colors, and renders the mesh.
 */
void Mesh::Draw(int programId, long double dt)
{
    glm::vec3 diffuseColor{ 0.7, 0.7, 0.5 };
    glm::vec3 specularColor{ 0.5, 0.5, 0.5 };
    float shininess = 1.0f;

    /*
    int loc = glGetUniformLocation(programId, "hasAnimation");
    glUniform1i(loc, 1);
    */

    float deltaTime = (float)dt;
    if (deltaTime > 0.1f) {
        deltaTime = 0.1f;
    }

    //N-NEW
    deltaTime *= m_animationSpeedMultiplier;

    CHECKERROR;
    glBindVertexArray(m_vao);
    CHECKERROR;

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

        glm::mat4 objectTr(1.0f);
        objectTr[0] = glm::vec4(gCubeRef->R[0], 0.0f);
        objectTr[1] = glm::vec4(gCubeRef->R[1], 0.0f);
        objectTr[2] = glm::vec4(gCubeRef->R[2], 0.0f);
        objectTr[3] = glm::vec4(gCubeRef->x, 1.0f);

        
        loc = glGetUniformLocation(programId, "ModelTr");
        glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(objectTr));
        //printf("R00=%f R02=%f\n", gCubeRef->R[0][0], gCubeRef->R[0][2]);

        glm::mat4 inv = glm::inverse(objectTr);
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
 * @brief Creates and configures the Vertex Array Object (VAO) for rendering.
 *
 * Sets up position, normal, tangent, texture coordinate, and bone attribute buffers.
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
 * @brief Initializes mesh data from the loaded Assimp scene.
 * @param path Path of the mesh file
 *
 * Extracts vertex positions, normals, tangents, texture coordinates, and bone data.
 * Also constructs the index buffer for rendering triangles.
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
 * @brief Sets the reference to a RigidBody for physics-driven transformations.
 * @param Rb RigidBody object representing the mesh's physical state
 *
 * This is used to apply position and rotation updates from the physics simulation.
 */
void Mesh::SetRigibody(RigidBody& Rb)
{
	gCubeRef = &Rb;
}



