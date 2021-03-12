#version 330 core

in vec3 vertexPosition;
in vec3 vertexNormal;

uniform vec3 color;
uniform vec3 lightDirection;
uniform vec3 cameraPosition;

void main() {
    vec3 ambientColor = 0.1 * color;
    vec3 diffuseColor = 0.6 * color;
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