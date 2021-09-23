#version 430 core

layout(location = 0) in vec4 vertexPosition;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec4 vertexTransformedNormal;
layout(location = 3) in float vertexX0;
layout(location = 4) in float vertexY0;
layout(location = 5) in float vertexZMin;
layout(location = 6) in float vertexZMax;
layout(location = 7) in float vertexA;
layout(location = 8) in float vertexB;
layout(location = 9) in float vertexC;
layout(location = 10) in float vertexDet_;

out vec4 position;
out vec4 color;
out vec4 transformedNormal;
out float x0;
out float y0;
out float zMin;
out float zMax;
out float a;
out float b;
out float c;
out float det_;

void main() {
    position = vertexPosition;
    color = vertexColor;
    transformedNormal = vertexTransformedNormal;
    x0 = vertexX0;
    y0 = vertexY0;
    zMin = vertexZMin;
    zMax = vertexZMax;
    a = vertexA;
    b = vertexB;
    c = vertexC;
    det_ = vertexDet_;

    gl_Position = vertexPosition;
}