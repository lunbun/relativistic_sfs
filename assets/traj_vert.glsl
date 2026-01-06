#version 330 core

uniform vec3 uPosition;
uniform mat4 uView;
uniform mat4 uProjection;

layout(location = 0) in vec3 aPos;

void main() {
    gl_Position = uProjection * (uView * vec4(uPosition + aPos, 1.0));
}
