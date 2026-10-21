#version 330 core
in vec3 vertex;

uniform mat4 WorldProj;
uniform mat4 WorldView;

void main()
{
    gl_Position = WorldProj * WorldView * vec4(vertex, 1.0);
}
