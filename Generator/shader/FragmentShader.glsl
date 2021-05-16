#version 330 core

in vec3 vertexPosition;
in vec3 vertexNormal;

uniform float lightPower;
uniform vec3 lightPosition;
uniform vec3 cameraPosition;

void main() {
    float distance = length(lightPosition - vertexPosition);
    vec3 ambientColor = 0.1 * vec3(1, 1, 1);
    vec3 diffuseColor = 0.6 * vec3(1, 1, 1);
    vec3 specularColor = 0.3 * vec3(1, 1, 1);

    vec3 ambient = ambientColor;

    vec3 N = normalize(vertexNormal);
    vec3 L = normalize(lightPosition - vertexPosition);
    vec3 diffuse = diffuseColor * max(dot(N, L), 0) * lightPower / (distance * distance);

    vec3 V = normalize(cameraPosition - vertexPosition);
    vec3 H = normalize(L + V);
    vec3 specular = specularColor * pow(max(dot(N, H), 0), 5) * lightPower / (distance * distance);

    gl_FragColor = vec4(1, 1, 1, 1);
}