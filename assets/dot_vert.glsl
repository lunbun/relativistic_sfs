#version 330 core

uniform float uSize;
uniform vec3 uPosition;
uniform mat4 uView;
uniform mat4 uProjection;

layout(location = 0) in vec2 aPos;

out vec2 fragPos;

void main() {
    fragPos = aPos;

    vec4 projPos = uProjection * (uView * vec4(uPosition, 1.0));
    gl_Position = projPos + vec4(uSize * projPos.w * aPos, 0.0, 0.0);
}
