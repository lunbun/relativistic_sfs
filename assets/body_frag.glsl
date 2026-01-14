#version 330 core

in vec3 vPosition;
in vec3 vNormal;
in vec2 vUV;

out vec4 FragColor;

void main() {
    vec3 lightPos = vec3(0.0, 0.0, 0.0);
    float intensity = 0.1 + clamp(dot(normalize(vNormal), normalize(lightPos - vPosition)), 0.0, 0.9);
    FragColor = vec4(intensity * vec3(0.8), 1.0);
}
