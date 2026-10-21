
#version 330

uniform mat4 WorldView, WorldInverse, WorldProj, ModelTr, NormalTr;

in vec4 vertex;
in vec3 vertexNormal;
in vec2 vertexTexture;
in vec3 vertexTangent;

out vec2 texCoord;
	
void main()
{
    gl_Position = WorldProj*WorldView*ModelTr*vertex;
    
    texCoord = vertexTexture; 
}
