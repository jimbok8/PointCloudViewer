#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 center;
layout(location = 3) in float radius;

out vec3 originPosition;
out vec3 vertexPosition;
out vec3 originNormal;
out vec3 vertexNormal;
out vec3 originCenter;
out float originRadius;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    originPosition = position;
    vertexPosition = vec3(model * vec4(position, 1));
    originNormal = normal;
    vertexNormal = vec3(model * vec4(normal, 0));
    originCenter = center;
    originRadius = radius;

    gl_Position = projection * view * model * vec4(position, 1);
}