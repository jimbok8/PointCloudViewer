#version 430 core

in vec3 bPos;

out vec4 FragColor;

struct Warper {
    float rotation[9];			// model-view rotation
    float translation[3];		// model-view translation
    float normalsRotation[9];	// rotation matrix to transform normals
    float scaling;				// uniform model-view scaling
    float xP, yP;				// x,y-extent of the view frustum at z=1
    float xC, yC;				// x,y-center point of the view frustum at z=1
    float nearplane, farplane;	// camera space z-values of near and far clipping planes
    float v[24];				// vertices of the view frustum in camera space
};

struct Surfel {
    vec4 position, normal, color, transformedNormal;
    int xMin, xMax, yMin, yMax;
    float radius, zMin, zMax, x0, y0, a, b, c, det;
};

layout(std430, binding = 0) buffer SSBO0 {
    Surfel zBuffer[];
};

layout(std430, binding = 1) buffer SSBO1 {
    Warper warper;
};

void main() {
    int x = int((bPos.x + 1.0f) / 2.0f * 800);
    int y = int((-bPos.y + 1.0f) / 2.0f * 600);

    zBuffer[y * 800 + x].color = vec4(warper.v[0], warper.v[1], warper.v[2], 1.0f);
    FragColor = vec4(0.0f, 1.0f, 0.0f, 1.0f);
}