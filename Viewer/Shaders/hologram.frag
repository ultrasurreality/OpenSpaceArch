#version 460 core

// Phase 1 minimal hologram fragment shader - Iron Man HUD aesthetic placeholder.
// Noise dissolve, edge glow, scan lines, metallic transition.

in vec3 vWorldPos;
in vec3 vNormal;
in vec3 vViewDir;

uniform float uTime;
uniform float uRevealProgress;
uniform vec3 uHoloColor;
uniform vec3 uMetalColor;

out vec4 fragColor;

// Simple hash-based 3D value noise (no texture needed for MVP)
float hash(vec3 p) {
    p = fract(p * 0.3183099 + vec3(0.1, 0.2, 0.3));
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

float noise3(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);

    float n000 = hash(i + vec3(0.0, 0.0, 0.0));
    float n100 = hash(i + vec3(1.0, 0.0, 0.0));
    float n010 = hash(i + vec3(0.0, 1.0, 0.0));
    float n110 = hash(i + vec3(1.0, 1.0, 0.0));
    float n001 = hash(i + vec3(0.0, 0.0, 1.0));
    float n101 = hash(i + vec3(1.0, 0.0, 1.0));
    float n011 = hash(i + vec3(0.0, 1.0, 1.0));
    float n111 = hash(i + vec3(1.0, 1.0, 1.0));

    return mix(
        mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
        mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y),
        f.z);
}

void main() {
    float n = noise3(vWorldPos * 0.04);
    if (n > uRevealProgress) discard;

    float edgeBand = 0.08;
    float edge = 1.0 - smoothstep(0.0, edgeBand, uRevealProgress - n);

    float fresnel = pow(1.0 - max(dot(vNormal, vViewDir), 0.0), 2.0);

    vec3 holoBase = uHoloColor * (0.15 + fresnel * 0.85);
    vec3 holoEdge = uHoloColor * edge * 4.0;
    vec3 holoColor = holoBase + holoEdge;

    float scan = 0.85 + 0.15 * sin(vWorldPos.z * 8.0 - uTime * 3.0);
    holoColor *= scan;

    float metalBlend = smoothstep(1.0, 1.25, uRevealProgress);
    vec3 color = mix(holoColor, uMetalColor * (0.3 + 0.7 * fresnel), metalBlend);

    float alpha = mix(0.7, 1.0, metalBlend);

    fragColor = vec4(color, alpha);
}
