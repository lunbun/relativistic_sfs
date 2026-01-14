#version 330 core

uniform vec3 uPosition;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;

out vec3 vPosition;
out vec3 vNormal;
out vec2 vUV;

void main() {
    gl_Position = uProjection * (uView * (uModel * vec4(aPos, 1.0)));
    vPosition = aPos + uPosition;
    vNormal = aNormal;
    vUV = aUV;
}
