#version 450
layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNrm;

layout(set = 0, binding = 0) uniform Frame {
    mat4 viewproj;
    vec4 sun_dir;
} frame;

layout(push_constant) uniform PC {
    mat4 model;
    vec4 color;
} pc;

layout(location = 0) out vec3 vN;
layout(location = 1) out vec3 vC;

void main() {
    gl_Position = frame.viewproj * pc.model * vec4(inPos, 1.0);
    vN = mat3(pc.model) * inNrm;
    vC = pc.color.rgb;
}
