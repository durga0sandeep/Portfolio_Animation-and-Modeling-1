/**
 * @file Mesh.cpp
 * @brief Implementation of the Mesh class supporting skeletal animation,
 *        GPU skinning, animation clips, and debug skeleton rendering.
 *
 * Technologies used:
 *  - OpenGL (glbinding)
 *  - Assimp (model + animation import)
 *  - GLM (math & transformations)
 *
 * Features:
 *  - Multiple animation tracks
 *  - Animation clips (subranges)
 *  - Looping & non-looping animations
 *  - GPU skinning (up to 4 bones per vertex)
 *  - Skeleton debug visualization
 */
	

#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)

using namespace gl;

#pragma warning(disable: 4190)
#include <glu.h>

#define CHECKERROR {GLenum err = glGetError(); if (err != GL_NO_ERROR) { fprintf(stderr, "OpenGL error (at line shapes.cpp:%d): %s\n", __LINE__, gluErrorString(err)); exit(-1);} }

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <iostream>

#include "Mesh.h"
#include "transform.h"
#include "shader.h"
#include "texture.h"
#include <gtx/quaternion.hpp>


#include <cmath>
#include <algorithm>


static aiMatrix4x4 AiIdentity()
{
	return aiMatrix4x4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}


/**
 * @brief Converts Assimp 4x4 matrix to GLM 4x4 matrix
 * @param m The Assimp matrix in row-major format
 * @return glm::mat4 The converted GLM matrix in column-major format
 * @details Performs transpose conversion from row-major to column-major ordering
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
 * @param m The Assimp 3x3 matrix
 * @return glm::mat4 The converted GLM 4x4 matrix with identity in 4th row/column
 */
glm::mat4 convert(const aiMatrix3x3& m)
{
	glm::mat4 res;
	res[0][0] = m.a1; res[0][1] = m.a2; res[0][2] = m.a3; res[0][3] = 0.0;
	res[1][0] = m.b1; res[1][1] = m.b2; res[1][2] = m.b3; res[1][3] = 0.0;
	res[2][0] = m.c1; res[2][1] = m.c2; res[2][2] = m.c3; res[2][3] = 0.0;
	res[3][0] = 0.0; res[3][1] =0.0; res[3][2] = 0.0; res[3][3] = 1.0;
	return res;
}

/**
 * @brief Converts GLM 4x4 matrix to Assimp 4x4 matrix
 * @param m The GLM matrix in column-major format
 * @return aiMatrix4x4 The converted Assimp matrix in row-major format
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
{

}

/**
 * @brief Destructor for Mesh class
 */
Mesh::~Mesh()
{
}


/**
 * @brief Loads a 3D mesh from file using Assimp
 * @param path File path to the 3D model (supports FBX, OBJ, etc.)
 * @details Imports the model with triangulation, smooth normals, flipped UVs,
 *          and vertex joining. Initializes mesh data and creates VAO for rendering.
 */
void Mesh::LoadMesh(const std::string& path)
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
	m_currentAnimationTime = 0.0f;

	Importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
	m_scene = Importer.ReadFile(path.c_str(), aiProcess_Triangulate 
											| aiProcess_GenSmoothNormals 
											| aiProcess_FlipUVs 
											| aiProcess_JoinIdenticalVertices);
	
	std::cout << std::endl;
	std::cout << "The number of children " << m_scene->mRootNode->mNumChildren << " " << std::endl;
	std::cout << std::endl;
	
	if (!m_scene)
	{
		printf("Error parsing '%s': '%s'\n", path.c_str(), Importer.GetErrorString());
		return;
	}


	m_inverseTrans = m_scene->mRootNode->mTransformation;
	m_inverseTrans = m_inverseTrans.Inverse();

	InitMesh(path);
	LoadAllAnimations();
	MakeVAO();

	std::cout << " " << std::endl;
	std::cout << " " << std::endl;
	std::cout << "Details: " << std::endl;
	std::cout << "Model loaded: " << path << std::endl;
	std::cout << "Meshes: " << m_scene->mNumMeshes << std::endl;
	std::cout << "Animations: " << m_scene->mNumAnimations << std::endl;
	std::cout << " " << std::endl;
	std::cout << " " << std::endl;

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
 * @brief Renders the animated mesh
 * @param programId OpenGL shader program ID to use for rendering
 * @param dt Delta time in seconds for animation progression
 * @details Updates animation time, calculates bone transformations, sets uniforms,
 *          and draws all submeshes with their materials
 */
void Mesh::Draw(int programId, long double dt)
{
	glm::vec3 diffuseColor{0.7,0.7,0.5};
	glm::vec3 specularColor{0.5,0.5,0.5};
	float shininess = 1.0f;

	aiMatrix4x4 Identity(1.0, 0.0, 0.0, 0.0,
						 0.0, 1.0, 0.0, 0.0,
						 0.0, 0.0, 1.0, 0.0,
						 0.0, 0.0, 0.0, 1.0);
	

	float deltaTime = (float)dt;
	if (deltaTime > 0.1f) {  
		deltaTime = 0.1f;
	}

	m_currentAnimationTime += deltaTime * 1.0f;//0.2f;
	

	float animationDuration = AnimationData.duration / AnimationData.ticksPerSecond;
	if (m_currentAnimationTime > animationDuration) {
		m_currentAnimationTime = fmod(m_currentAnimationTime, animationDuration);
	}

	CalFinalTransformations();

	CHECKERROR;
	glBindVertexArray(m_vao);
	CHECKERROR;
	


	for (int i = 0; i < m_boneInfo.size(); ++i)
	{
		std::string name = "gBones[" + std::to_string(i) + "]";
		int loc = glGetUniformLocation(programId, name.c_str());
		
		glUniformMatrix4fv(loc, 1, GL_TRUE, &m_boneInfo[i].FinalTransformation[0][0]);
		CHECKERROR;
	}



	int vertices = 0;
	int indices = 0;
	for (unsigned int i = 0; i < m_scene->mNumMeshes; ++i)
	{
		int matIndex = m_scene->mMeshes[i]->mMaterialIndex;

		//m_diffuse[matIndex]->BindTexture(0, programId, "diffuseTex");

		int loc = glGetUniformLocation(programId, "diffuse");
		glUniform3fv(loc, 1, &diffuseColor[0]);

		loc = glGetUniformLocation(programId, "specular");
		glUniform3fv(loc, 1, &specularColor[0]);

		loc = glGetUniformLocation(programId, "shininess");
		glUniform1f(loc, shininess);
		CHECKERROR;

		glm::mat4 objectTr= Translate(0.f,0.f,-2.5f)*Rotate(0, 90.f) *Scale(0.1f, 0.1f, 0.1f);
		loc = glGetUniformLocation(programId, "ModelTr");
		glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(objectTr));

		glm::mat4 inv = glm::inverse(objectTr);
		loc = glGetUniformLocation(programId, "NormalTr");
		glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(inv));

		int curr_indices = m_scene->mMeshes[i]->mNumFaces * 3;
		glDrawElementsBaseVertex(GL_TRIANGLES, curr_indices, GL_UNSIGNED_INT,
								(void*)(sizeof(unsigned int) * indices), vertices);

		vertices += m_scene->mMeshes[i]->mNumVertices;
		indices += curr_indices;
	}
	CHECKERROR;
	glBindVertexArray(0);
}


/**
 * @brief Creates and configures the Vertex Array Object (VAO) for rendering
 * @details Sets up VBOs for positions, normals, texture coordinates, tangents,
 *          bone IDs, bone weights, and index buffer
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
	glBufferData(GL_ARRAY_BUFFER, sizeof(BoneData) * m_bones.size(),&m_bones[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(4);
	glVertexAttribIPointer(4, MAX_NUM_BONES_PER_VERTEX, GL_INT, sizeof(BoneData), (const GLvoid*)0);
	glEnableVertexAttribArray(5);
	glVertexAttribPointer(5, MAX_NUM_BONES_PER_VERTEX, GL_FLOAT,GL_FALSE, sizeof(BoneData),(const GLvoid*)(MAX_NUM_BONES_PER_VERTEX *sizeof(int)));
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	GLuint Ibuff;
	glGenBuffers(1, &Ibuff);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Ibuff);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int)* m_indices.size(),
		&m_indices[0], GL_STATIC_DRAW);

	glBindVertexArray(0);
}

/**
 * @brief Initializes mesh data from the loaded scene
 * @param path File path (used for debugging output)
 * @details Extracts vertices, normals, texture coordinates, tangents, and indices
 *          from all submeshes. Initializes bone data for skeletal animation.
 */
void Mesh::InitMesh(const std::string& path)
{
	int num_vertices = 0;
	//load meshes
	
	for (unsigned int i = 0; i < m_scene->mNumMeshes; i++) {

		aiMesh* mesh = m_scene->mMeshes[i];
		std::cout << "* Mesh  " << i << mesh->mName.C_Str() << std::endl;

	}


	for (unsigned int i = 0; i < m_scene->mNumMeshes; ++i)
	{
		const aiMesh* m = m_scene->mMeshes[i];
		
		//
		//individual mesh
		std::cout << std::endl;
		std::cout << "Check the number of calls " << m->mNumVertices << std::endl;
		//
		for (unsigned int j = 0; j < m->mNumVertices; ++j)
		{
			aiVector3D& p = m->mVertices[j];
			m_pnt.push_back(glm::vec4(p.x, p.y, p.z, 1.0));

			if (m->mNormals)
			{
				aiVector3D& n = m->mNormals[j];
				m_norm.push_back(glm::vec3(n.x, n.y, n.z));
			}

			if (m->mTangents)
			{
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

		for (unsigned int j = 0; j < m->mNumFaces; ++j)
		{
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
 * @param mesh Pointer to the Assimp mesh containing bone information
 * @param index Vertex offset for this mesh in the combined vertex array
 * @details Extracts bone weights and IDs, creates bone info structures,
 *          and associates vertices with their influencing bones
 */
//void Mesh::InitBone(const aiMesh* mesh, int index)
//{
//	//
//	std::cout << "Number of bones " << mesh->mNumBones << " No.of vertices" << index << std::endl;
//	
//	for (unsigned int i = 0; i < mesh->mNumBones; ++i)
//	{
//		const aiBone* b = mesh->mBones[i];
//
//		int bone_id = GetBoneId(b);
//		
//
//		
//		if (bone_id == m_boneInfo.size()) {
//			BoneInfo bi(b->mOffsetMatrix);
//			m_boneInfo.push_back(bi);
//		}
//		
//
//
//		for (unsigned int j = 0; j < b->mNumWeights; ++j)
//		{
//			const aiVertexWeight& wt = b->mWeights[j];
//			int vertexId = index + b->mWeights[j].mVertexId;
//			m_bones[vertexId].AddBoneData(bone_id, wt.mWeight);
//		}
//
//
//	}
//	
//}

void Mesh::InitBone(const aiMesh* mesh, int index)
{
	std::cout << "Number of bones " << mesh->mNumBones
		<< " No.of vertices offset " << index << std::endl;

	for (unsigned int i = 0; i < mesh->mNumBones; ++i)
	{
		const aiBone* b = mesh->mBones[i];
		int bone_id = GetBoneId(b);

		// --- FIX #2: Ensure m_boneInfo is large enough for bone_id ---
		// (Works even if BoneInfo has no default constructor)
		while ((int)m_boneInfo.size() <= bone_id)
		{
			// BoneInfo constructor in your code: BoneInfo bi(offsetMatrix);
			// We add placeholder identity until we reach bone_id.
			m_boneInfo.emplace_back(AiIdentity());
			// If BoneInfo has FinalTransformation, initialize it too (optional).
			m_boneInfo.back().FinalTransformation = AiIdentity();
		}

		// Now it is ALWAYS safe to write into m_boneInfo[bone_id]
		m_boneInfo[bone_id].OffsetMatrix = b->mOffsetMatrix;

		// Good practice: initialize final transform for this bone
		// (so if a bone isn't animated this frame, it's not garbage)
		m_boneInfo[bone_id].FinalTransformation = AiIdentity();

		// --- vertex weights ---
		for (unsigned int j = 0; j < b->mNumWeights; ++j)
		{
			const aiVertexWeight& wt = b->mWeights[j];

			int vertexId = index + (int)wt.mVertexId;

			// Safety guard (optional but helps catch bad data)
			if (vertexId < 0 || vertexId >= (int)m_bones.size())
				continue;

			m_bones[vertexId].AddBoneData(bone_id, wt.mWeight);
		}
	}
}




/**
 * @brief Gets or creates a unique ID for a bone
 * @param b Pointer to the Assimp bone
 * @return int Unique bone ID
 * @details Maps bone names to IDs, creating new IDs for previously unseen bones
 */
/*
int Mesh::GetBoneId(const aiBone* b)
{
	std::string name(b->mName.C_Str());

	//Bone Names
	//std::cout << "bone name : " << name << std::endl;

	auto it = m_name_index.find(name);
	if(it == m_name_index.end())
		m_name_index[name] = (int)m_name_index.size();

	return m_name_index[name];
}
*/
int Mesh::GetBoneId(const aiBone* b)
{
	std::string name(b->mName.C_Str());

	auto it = m_name_index.find(name);
	if (it != m_name_index.end())
		return it->second;

	int newId = (int)m_name_index.size();
	m_name_index[name] = newId;
	return newId;
}

	



/**
 * @brief Prints all bone names and their corresponding IDs
 * @details Debug utility to display the bone name-to-ID mapping
 */
void Mesh::PrintValues()
{
	std::cout << "Values of the map:" << std::endl;
	for (std::map<std::string, int>::iterator it = m_name_index.begin(); it != m_name_index.end(); ++it) {
		std::cout << " key ---> " << it->first << std::endl; // Access the value
		std::cout << " value --> " << it->second << std::endl; // Access the value
	}

}

/**
 * @brief Loads animation data from the scene
 * @details Extracts animation channels including position, rotation, and scale
 *          keyframes for each bone. Sets up the AnimationData structure with
 *          timing and transformation information.
 */
void Mesh::LoadAnimation()
{
	//
	//DEBUGGING 
	std::cout << "ANIMAIONS 1 ---> " << m_scene->mNumAnimations << std::endl;

	for (unsigned int i = 0; i < m_scene->mNumAnimations; i++) {
		std::cout << "No.Of Animations: " << m_scene->mNumAnimations << "\n";
		anim = m_scene->mAnimations[i];
		std::cout << "Animation name: " << anim->mName.C_Str() << "\n";
		std::cout << "Duration: " << anim->mDuration << "\n";
		std::cout << "TicksPerSecond: " << anim->mTicksPerSecond << "\n";
	}

	boneCount = m_name_index.size();
	std::cout << " boneCount ---> " << boneCount << std::endl;
	
	//Loading Animations
	if (m_scene->mAnimations == 0) {
		std::cout << "No animations --- CHECK ERROR " << std::endl;
	}
	//

	anim = m_scene->mAnimations[0];

	std::cout << "ANIMAIONS TICKS ---> " << anim->mTicksPerSecond << std::endl;
	
	Animation animation;
	animation.duration = (float)anim->mDuration;;
	animation.ticksPerSecond = (float)(anim->mTicksPerSecond != 0
								? anim->mTicksPerSecond
								: 25.0f);//But we are given 30 as tick here

	animation.boneAnimations.resize(boneCount);
	
	for (unsigned int c = 0; c < anim->mNumChannels; c++) {

		aiNodeAnim* channel = anim->mChannels[c];
		std::string boneName = channel->mNodeName.C_Str();

		auto it = m_name_index.find(boneName);
		
		if (it == m_name_index.end()) {
			continue;
		}

		int boneID = it->second;
		std::cout << " ID IS " << boneID << std::endl;

		BoneAnimation& boneAnim = animation.boneAnimations[boneID];

		for (unsigned int i = 0; i < channel->mNumPositionKeys; i++) {
			aiVectorKey key = channel->mPositionKeys[i];
			boneAnim.positionTimestamps.push_back(channel->mPositionKeys[i].mTime);
			boneAnim.positions.push_back({ toVec3(key.mValue), (float)key.mTime });
		}

		for (unsigned int i = 0; i < channel->mNumRotationKeys; i++) {
			aiQuatKey key = channel->mRotationKeys[i];
			boneAnim.rotationTimestamps.push_back(channel->mRotationKeys[i].mTime);
			boneAnim.rotations.push_back({ toQuat(key.mValue), (float)key.mTime });
		}

		for (unsigned int i = 0; i < channel->mNumScalingKeys; i++) {
			aiVectorKey key = channel->mScalingKeys[i];
			boneAnim.scaleTimestamps.push_back(channel->mScalingKeys[i].mTime);
			boneAnim.scales.push_back({ toVec3(key.mValue), (float)key.mTime });
		}
	}



	AnimationData = animation;

}



/**
 * @brief Calculates final bone transformations for the current animation time
 * @details Computes animation time in ticks, traverses the scene graph to calculate
 *          bone transforms, and applies offset matrices to get final skinning matrices
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

	for (size_t i = 0; i < m_boneInfo.size(); i++) {
		glm::mat4 finalTrans = convert(m_boneInfo[i].FinalTransformation);
		glm::mat4 offset = convert(m_boneInfo[i].OffsetMatrix);
		glm::mat4 result = globalInverseTransform * finalTrans * offset;
		m_boneInfo[i].FinalTransformation = convert(result);
	}
}



/**
 * @brief Recursively traverses the scene graph to calculate bone transformations
 * @param animationTime Current time in the animation (in ticks)
 * @param node Current node in the scene hierarchy
 * @param parentTransform Accumulated transformation from parent nodes
 * @param track the animation track data
 * @details Interpolates position, rotation, and scale for animated bones, then
 *          recursively processes child nodes to build complete skeleton pose
 */
void Mesh::ReadNodeHierarchy(float animationTime, const aiNode* node,
	const glm::mat4& parentTransform, const Animation& track)
{
	std::string nodeName(node->mName.C_Str());
	glm::mat4 nodeTransformation = convert(node->mTransformation);

	auto it = m_name_index.find(nodeName);
	if (it != m_name_index.end()) {
		int boneIndex = it->second;
		if (boneIndex < 0 ||
			boneIndex >= (int)track.boneAnimations.size() ||
			boneIndex >= (int)m_boneInfo.size())
		{
			// Optional: log once
			// std::cout << "Bone index out of range: " << nodeName
			//           << " idx=" << boneIndex
			//           << " animSize=" << track.boneAnimations.size()
			//           << " boneInfoSize=" << m_boneInfo.size() << "\n";
		}
		else
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
	}

	glm::mat4 globalTransformation = parentTransform * nodeTransformation;
	nodeGlobalTransforms[nodeName] = globalTransformation;

	if (it != m_name_index.end())
	{
		int boneIndex = it->second;
		if (boneIndex >= 0 && boneIndex < (int)m_boneInfo.size())
			m_boneInfo[boneIndex].FinalTransformation = convert(globalTransformation);
	}

	for (unsigned int i = 0; i < node->mNumChildren; i++) {
		ReadNodeHierarchy(animationTime, node->mChildren[i], globalTransformation, track);
	}
}


/**
 * @brief Interpolates bone position at a given animation time
 * @param time Current animation time in ticks
 * @param anim Bone animation data containing position keyframes
 * @return glm::vec3 Interpolated position vector
 * @details Uses linear interpolation between adjacent keyframes
 */
glm::vec3 Mesh::CalcInterpolatedPosition(float time, const BoneAnimation& anim)
{

	if (anim.positions.empty()) {
		return glm::vec3(0.0f);
	}

	if (anim.positions.size() == 1)
		return anim.positions[0].position;

	for (size_t i = 0; i < anim.positions.size() - 1; i++)
	{
		if (time < anim.positions[i + 1].timeStamp)
		{
			float delta = anim.positions[i + 1].timeStamp - anim.positions[i].timeStamp;
			float factor = (time - anim.positions[i].timeStamp) / delta;
			return glm::mix(anim.positions[i].position, anim.positions[i + 1].position, factor);
		}
	}

	return anim.positions.back().position;
}

/**
 * @brief Interpolates bone rotation at a given animation time
 * @param time Current animation time in ticks
 * @param anim Bone animation data containing rotation keyframes
 * @return glm::quat Interpolated rotation quaternion
 * @details Uses spherical linear interpolation (slerp) between adjacent keyframes,
 *          handles quaternion polarity to ensure shortest path rotation
 */
glm::quat Mesh::CalcInterpolatedRotation(float time, const BoneAnimation& anim)
{
	if (anim.rotations.empty())
		return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

	if (anim.rotations.size() == 1)
		return anim.rotations[0].rotation;

	for (size_t i = 0; i < anim.rotations.size() - 1; i++)
	{
		if (time < anim.rotations[i + 1].timeStamp)
		{
			float delta = anim.rotations[i + 1].timeStamp - anim.rotations[i].timeStamp;
			float factor = (time - anim.rotations[i].timeStamp) / delta;

			glm::quat q1 = anim.rotations[i].rotation;
			glm::quat q2 = anim.rotations[i + 1].rotation;

			if (glm::dot(q1, q2) < 0.0f) {
				q2 = -q2;
			}

			float t = static_cast<float>(factor);
			glm::quat result = glm::slerp(q1, q2, t);


			return glm::normalize(result); // always normalize
		}
	}

	return anim.rotations.back().rotation;
}

/**
 * @brief Interpolates bone scale at a given animation time
 * @param time Current animation time in ticks
 * @param anim Bone animation data containing scale keyframes
 * @return glm::vec3 Interpolated scale vector
 * @details Uses linear interpolation between adjacent keyframes
 */
glm::vec3 Mesh::CalcInterpolatedScaling(float time, const BoneAnimation& anim)
{

	if (anim.scales.empty()) {
		return glm::vec3(0.0f);
	}

	if (anim.scales.size() == 1)
		return anim.scales[0].scale;

	for (size_t i = 0; i < anim.scales.size() - 1; i++)
	{
		if (time < anim.scales[i + 1].timeStamp)
		{
			float delta = anim.scales[i + 1].timeStamp - anim.scales[i].timeStamp;
			float factor = (time - anim.scales[i].timeStamp) / delta;
			return glm::mix(anim.scales[i].scale, anim.scales[i + 1].scale, factor);
		}
	}

	return anim.scales.back().scale;
}


/**
 * @brief Converts Assimp vector to GLM vector
 * @param v Assimp 3D vector
 * @return glm::vec3 Converted GLM vector
 */
glm::vec3 Mesh::toVec3(const aiVector3D& v) {
	return glm::vec3(v.x, v.y, v.z);
}



/**
 * @brief Converts Assimp quaternion to GLM quaternion
 * @param q Assimp quaternion
 * @return glm::quat Converted GLM quaternion
 */
glm::quat Mesh::toQuat(const aiQuaternion& q) {
	return glm::quat(q.w, q.x, q.y, q.z);
}


/**
 * @brief Prints all bone names and IDs to console
 * @details Debug utility for displaying the bone name-to-ID mapping
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
 * @param VAO Reference to vertex array object ID
 * @param VBO Reference to vertex buffer object ID
 * @details Creates buffers for drawing debug skeleton lines
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
 * @param start Starting point of the line
 * @param end Ending point of the line
 * @param shader Shader program to use for rendering
 * @param VAO Vertex array object for line rendering
 * @param VBO Vertex buffer object for line rendering
 * @details Used for drawing skeleton debug visualization
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
 * @param viewProj Combined view-projection matrix
 * @param shader Shader program for line rendering
 * @details Draws lines connecting parent bones to child bones, visualizing
 *          the skeleton hierarchy and current animation pose
 */
void Mesh::DebugSkeleton(const glm::mat4& viewProj, GLuint shader)
{
	static GLuint lineVAO = 0;
	static GLuint lineVBO = 0;

	if (lineVAO == 0)
	{
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
 * @param node Current node in the scene hierarchy
 * @param modelTransform Model transformation matrix
 * @param shader Shader program for line rendering
 * @param lineVAO Vertex array object for lines
 * @param lineVBO Vertex buffer object for lines
 * @details Traverses the bone hierarchy, drawing lines from each bone to its parent.
 *          Uses current animated transformations for accurate pose visualization.
 */
void Mesh::DrawSkeletonRecursive(const aiNode* node, const glm::mat4& modelTransform,
	GLuint shader, GLuint lineVAO, GLuint lineVBO)
{
	std::string nodeName(node->mName.C_Str());

	// Check if this node is a bone
	auto boneIt = m_name_index.find(nodeName);

	if (boneIt != m_name_index.end())
	{
		// Get the animated global transform
		auto transformIt = nodeGlobalTransforms.find(nodeName);
		if (transformIt != nodeGlobalTransforms.end())
		{
			glm::mat4 globalTransform = transformIt->second;
			glm::vec3 currentBonePos = glm::vec3(globalTransform[3]);
			currentBonePos = glm::vec3(modelTransform * glm::vec4(currentBonePos, 1.0f));

			// Draw line to parent bone if it exists
			if (node->mParent)
			{
				std::string parentName(node->mParent->mName.C_Str());
				auto parentBoneIt = m_name_index.find(parentName);

				if (parentBoneIt != m_name_index.end())
				{
					auto parentTransformIt = nodeGlobalTransforms.find(parentName);
					if (parentTransformIt != nodeGlobalTransforms.end())
					{
						glm::vec3 parentBonePos = glm::vec3(parentTransformIt->second[3]);
						parentBonePos = glm::vec3(modelTransform * glm::vec4(parentBonePos, 1.0f));

						DrawLine(parentBonePos, currentBonePos, shader, lineVAO, lineVBO);
					}
				}
			}
		}
	}

	// Recursively process all children
	for (unsigned int i = 0; i < node->mNumChildren; i++)
	{
		DrawSkeletonRecursive(node->mChildren[i], modelTransform, shader, lineVAO, lineVBO);
	}
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
	std::cout << "Switched to animation: " << animName << std::endl;
}


/**
 * @brief Sets Y-offset for an animation.
 */
void Mesh::SetAnimationYOffset(const std::string& animName, float offset) {
	m_animationYOffsets[animName] = offset;
}

/**
 * @brief Gets Y-offset for an animation.
 */	
float Mesh::GetAnimationYOffset(const std::string& animName) const {
	auto it = m_animationYOffsets.find(animName);
	return (it != m_animationYOffsets.end()) ? it->second : 0.0f;
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

	float clipLen = clip.endTime - clip.startTime;
	if (clipLen <= 0.0f) return clip.startTime;

	float t = globalTime;

	if (clip.loop) {
		t = fmod(t, clipLen);
		if (t < 0.0f) t += clipLen;
	}
	else {
		if (t < 0.0f) t = 0.0f;
		if (t > clipLen) t = clipLen;
	}

	return clip.startTime + t;
}

