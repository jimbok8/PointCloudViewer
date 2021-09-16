#version 430 core

layout(location = 0) in vec3 aPos;

out vec3 bPos;

layout(std430, binding = 2) buffer ColorSSBO {
    vec4 color[];
};

void main() {
    int x = int((aPos.x + 1.0f) / 2.0f * 800);
    int y = int((-aPos.y + 1.0f) / 2.0f * 600);

    color[y * 800 + x] = vec4(1.0f, 0.0f, 0.0f, 1.0f);
    
    bPos = aPos;
    gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);
}