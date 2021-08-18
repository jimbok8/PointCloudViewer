#version 330 core

in vec2 vertexUV;

uniform sampler2D tex;

void main() {
	gl_FragColor = texture(tex, vertexUV);
}