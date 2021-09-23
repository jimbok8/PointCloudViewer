#version 430 core

#extension GL_NV_shader_atomic_float : require

in vec4 position;
in vec4 color;
in vec4 transformedNormal;
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
    vec4 color, transformedNormal;
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

layout(std430, binding = 4) readonly buffer SSBO4 {
    float filterLUT[];
};

void main() {
    int X = int((position.x + 1.0f) / 2.0f * width);
    int Y = int((position.y + 1.0f) / 2.0f * height);
    if (X < 0 || X >= width || Y < 0 || Y >= height)
        return;

    float zbf_LUTsize, zbf_cutoffRadius_2, _zbf_cutoffRadius_2;

    float x, y, q, w;

    zbf_LUTsize = zBufferProperty.LUTsize;
    zbf_cutoffRadius_2 = zBufferProperty.cutoffRadius * zBufferProperty.cutoffRadius;
    _zbf_cutoffRadius_2 = 1 / zbf_cutoffRadius_2;

    y = float(Y) + 0.5f - y0;
    x = float(X) + 0.5f - x0;
    q = a * x * x + b * x * y + c * y * y;
    int index = Y * width + X;

    if (q < zbf_cutoffRadius_2 && zMin <= zBuffer[index].zMax) {
        w = filterLUT[int(q * _zbf_cutoffRadius_2 * zbf_LUTsize)] * det_;

        atomicAdd(zBuffer[index].w, w);

        atomicAdd(zBuffer[index].color.x, color.x * w);
        atomicAdd(zBuffer[index].color.y, color.y * w);
        atomicAdd(zBuffer[index].color.z, color.z * w);

        atomicAdd(zBuffer[index].transformedNormal.x, transformedNormal.x * w);
        atomicAdd(zBuffer[index].transformedNormal.y, transformedNormal.y * w);
        atomicAdd(zBuffer[index].transformedNormal.z, transformedNormal.z * w);
    }

    FragColor = vec4(0.0f, 0.0f, 0.0f, 1.0f);
}