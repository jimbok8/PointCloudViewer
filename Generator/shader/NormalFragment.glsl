#version 330 core

in vec3 vertexPosition;
in vec3 vertexNormal;
in vec3 originalNormal;

uniform float lightPower;
uniform vec3 lightPosition;
uniform vec3 cameraPosition;

void main() {
    float distance = length(lightPosition - vertexPosition);
    vec3 color = 0.5 * (normalize(originalNormal) + vec3(1));
    vec3 ambientColor = 0.1 * color;
    vec3 diffuseColor = 0.6 * color;
    vec3 specularColor = vec3(0.3);

    vec3 ambient = ambientColor;

    vec3 N = normalize(vertexNormal);
    vec3 L = normalize(lightPosition - vertexPosition);
    vec3 diffuse = diffuseColor * abs(dot(N, L)) * lightPower / (distance * distance);

    vec3 V = normalize(cameraPosition - vertexPosition);
    vec3 H = normalize(L + V);
    vec3 specular = specularColor * pow(abs(dot(N, H)), 5) * lightPower / (distance * distance);

    gl_FragColor = vec4(ambient + diffuse + specular, 1);
}