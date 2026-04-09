#version 460 core

// plume.vert - rocket exhaust particles. Each instance has its own
// position (xyz) + age (w). Rendered as GL_POINTS with size from age.

layout(location = 0) in vec4 aPosAge;

uniform mat4 uView;
uniform mat4 uProj;
uniform float uLifetime;
uniform float uPointSize;

out float vAge01;
out float vFade;

void main() {
    vec4 wp = vec4(aPosAge.xyz, 1.0);
    gl_Position = uProj * uView * wp;

    float age01 = clamp(aPosAge.w / uLifetime, 0.0, 1.0);
    vAge01 = age01;
    vFade = 1.0 - age01;

    float distance = length((uView * wp).xyz);
    float sizeFactor = mix(1.3, 0.4, age01);
    gl_PointSize = (uPointSize * sizeFactor) / max(distance * 0.002, 0.5);
}
