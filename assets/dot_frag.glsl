#version 330 core

in vec2 fragPos;

out vec4 FragColor;

void main() {
    float edge = 0.01;
    float radius = 0.5 - edge;
    float dist = length(fragPos);
    float alpha = 1.0 - smoothstep(radius - edge, radius + edge, dist);
    FragColor = vec4(1.0, 1.0, 1.0, alpha);
}
