#version 430 core

layout(location = 0) in vec3 aPos;

out vec3 bPos;

void main() {
    bPos = aPos;
    gl_Position = vec4(aPos, 1.0f);
}