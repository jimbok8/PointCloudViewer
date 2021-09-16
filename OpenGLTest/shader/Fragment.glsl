#version 430 core

in vec3 bPos;

out vec4 FragColor;

layout(std430, binding = 2) buffer ColorSSBO {
    vec4 color[];
};

void main() {
    int x = int((bPos.x + 1.0f) / 2.0f * 800);
    int y = int((-bPos.y + 1.0f) / 2.0f * 600);

    FragColor = color[y * 800 + x];
}