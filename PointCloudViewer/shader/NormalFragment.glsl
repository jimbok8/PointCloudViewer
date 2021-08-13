#version 330 core

in vec3 originPosition;
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec3 originalNormal;

uniform float minX;
uniform float maxX;
uniform vec3 lightDirection;
uniform vec3 cameraPosition;

void main() {
    float x = (originPosition.x - minX) / (maxX - minX), r, g, b;
    if (x <= 0.5f) {
        r = 0.0f;
        g = x * 2.0f;
        b = 1.0f - g;
    }
    else {
        b = 0.0f;
        r = (x - 0.5f) * 2.0f;
        g = 1.0f - r;
    }
    vec3 color = vec3(r, g, b);
    //vec3 color = 0.5 * (normalize(originalNormal) + vec3(1));
    vec3 ambientColor = 1.0 * color;
    vec3 diffuseColor = 2.0 * color;
    vec3 specularColor = vec3(0.3);

    vec3 ambient = ambientColor;

    vec3 N = normalize(vertexNormal);
    vec3 L = normalize(-lightDirection);
    vec3 diffuse = diffuseColor * abs(dot(N, L));

    vec3 V = normalize(cameraPosition - vertexPosition);
    vec3 H = normalize(L + V);
    vec3 specular = specularColor * pow(abs(dot(N, H)), 5);

    gl_FragColor = vec4(ambient + diffuse + specular, 1);
}