#version 430 core

in vec3 bPos;

out vec4 FragColor;

struct Surfel {
    vec4 position, normal, color, transformedNormal;
    int xMin, xMax, yMin, yMax;
    float radius, zMin, zMax, x0, y0, a, b, c, det;
    //vec4 zRangeAndXY;
    //vec4 parameters;
    //ivec4 box;
};

layout(std430, binding = 0) buffer SSBO {
    Surfel zBuffer[];
};

void main() {
    int x = int((bPos.x + 1.0f) / 2.0f * 800);
    int y = int((-bPos.y + 1.0f) / 2.0f * 600);

    FragColor = zBuffer[y * 800 + x].color;
}