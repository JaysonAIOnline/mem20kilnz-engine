#version 450
layout(location = 0) in vec3 vN;
layout(location = 1) in vec3 vC;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform Frame {
    mat4 viewproj;
    vec4 sun_dir;
} frame;

void main() {
    vec3 n = normalize(vN);
    vec3 l = normalize(frame.sun_dir.xyz);
    float ndl = max(dot(n, l), 0.0);
    vec3 amb = vC * 0.13;
    vec3 col = amb + vC * ndl;
    col += vec3(0.04) * pow(ndl, 24.0);
    outColor = vec4(pow(col, vec3(1.0 / 2.2)), 1.0);
}
