/////////////////////////////////////////////////////////////////////////
// Vertex shader for lighting
//
// Copyright 2013 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////
#version 330

uniform mat4 WorldView, WorldProj, ModelTr, NormalTr;

layout(location = 0) in vec4 vertex;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec2 vertexTexture;
layout(location = 3) in vec3 vertexTangent;
layout(location = 4) in ivec4 boneIds;
layout(location = 5) in vec4 weight;


out vec3 normalVec;
out vec2 texCoord;
out vec4 worldPos;


uniform mat4 gBones[27];//bones final transformation matrix

void main()
{
    

    mat4 bonesTransforms = 
        gBones[boneIds[0]] * weight[0] +
        gBones[boneIds[1]] * weight[1] +
        gBones[boneIds[2]] * weight[2] +
        gBones[boneIds[3]] * weight[3];

    gl_Position = WorldProj * WorldView * ModelTr * bonesTransforms * vertex;
    worldPos = ModelTr*vertex;
    normalVec = vertexNormal*mat3(NormalTr); 
    texCoord = vertexTexture; 


}




