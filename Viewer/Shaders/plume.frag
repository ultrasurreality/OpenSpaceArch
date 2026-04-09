#version 460 core

// plume.frag - additively blended circular particles colored by age.
// White-hot core -> yellow -> orange -> red -> fade.

in float vAge01;
in float vFade;

out vec4 fragColor;

vec3 plumeColor(float t) {
    if (t < 0.15) return mix(vec3(1.3, 1.3, 1.2), vec3(1.2, 1.1, 0.5), t / 0.15);
    if (t < 0.35) return mix(vec3(1.2, 1.1, 0.5), vec3(1.1, 0.7, 0.15), (t - 0.15) / 0.2);
    if (t < 0.70) return mix(vec3(1.1, 0.7, 0.15), vec3(0.9, 0.25, 0.05), (t - 0.35) / 0.35);
    return mix(vec3(0.9, 0.25, 0.05), vec3(0.2, 0.02, 0.0), (t - 0.70) / 0.30);
}

void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    float r = length(d);
    if (r > 0.5) discard;
    float disk = 1.0 - smoothstep(0.28, 0.5, r);

    vec3 c = plumeColor(vAge01);
    float alpha = disk * vFade;

    fragColor = vec4(c * alpha, alpha);
}
