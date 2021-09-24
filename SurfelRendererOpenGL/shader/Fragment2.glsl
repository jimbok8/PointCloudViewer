#version 430 core

in vec4 position;
in vec4 normal;
in vec4 color;
in float x0;
in float y0;
in float zMin;
in float zMax;
in float a;
in float b;
in float c;
in float det_;

out vec4 FragColor;

struct ZBufferProperty {
    int bufsize;
    int LUTsize;
    float cutoffRadius;
    float angleTrheshold;
};

struct ZBufferItem {
    vec4 normal, color;
    float zMin, zMax, w;
};

uniform int width;
uniform int height;

layout(std430, binding = 0) buffer SSBO0 {
    ZBufferProperty zBufferProperty;
};

layout(std430, binding = 3) buffer SSBO3 {
    ZBufferItem zBuffer[];
};

void main() {
    int X = int((position.x + 1.0f) / 2.0f * width);
    int Y = int((position.y + 1.0f) / 2.0f * height);
    if (X < 0 || X >= width || Y < 0 || Y >= height)
        return;

    float zbf_cutoffRadius_2;

    float x, y, q;

    zbf_cutoffRadius_2 = zBufferProperty.cutoffRadius * zBufferProperty.cutoffRadius;

    y = float(Y) + 0.5f - y0;
    x = float(X) + 0.5f - x0;
    q = a * x * x + b * x * y + c * y * y;
    int index = Y * width + X;

    if (q < zbf_cutoffRadius_2 && zMin == zBuffer[index].zMin && zMax > zBuffer[index].zMax)
        zBuffer[index].zMax = zMax;
        //atomicMax(&zbufitem->zMax, surfel->zMax);

    FragColor = vec4(0.0f, 0.0f, 0.0f, 1.0f);
}