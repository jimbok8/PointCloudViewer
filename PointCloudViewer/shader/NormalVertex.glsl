#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

out vec3 originPosition;
out vec3 vertexPosition;
out vec3 originalNormal;
out vec3 vertexNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    originPosition = position;
    vertexPosition = vec3(model * vec4(position, 1));
    originalNormal = normal;
    vertexNormal = vec3(model * vec4(normal, 0));

    gl_Position = projection * view * model * vec4(position, 1);
}