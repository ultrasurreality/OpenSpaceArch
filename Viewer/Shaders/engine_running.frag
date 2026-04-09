#version 460 core

// engine_running.frag - walls and channels when the engine is "running".
// uMode == 0 : wall heat map (reads T/q from uHeatProfile sampler1D)
// uMode == 1 : coolant flow (scrolling gradient along Z for channel meshes)

in vec3 vWorldPos;
in vec3 vNormal;
in vec3 vViewDir;

uniform int uMode;
uniform float uTime;
uniform float uThrottle;
uniform float uZmin;
uniform float uZmax;
uniform sampler1D uHeatProfile;

out vec4 fragColor;

// black-body-ish color ramp: cold dark blue -> blue -> teal -> yellow -> orange -> red -> white
vec3 heatRamp(float t) {
    t = clamp(t, 0.0, 1.0);
    if (t < 0.2)  return mix(vec3(0.02, 0.04, 0.10), vec3(0.05, 0.25, 0.85), t / 0.2);
    if (t < 0.4)  return mix(vec3(0.05, 0.25, 0.85), vec3(0.20, 0.80, 0.85), (t - 0.2) / 0.2);
    if (t < 0.6)  return mix(vec3(0.20, 0.80, 0.85), vec3(1.00, 0.90, 0.20), (t - 0.4) / 0.2);
    if (t < 0.8)  return mix(vec3(1.00, 0.90, 0.20), vec3(1.00, 0.40, 0.10), (t - 0.6) / 0.2);
    return           mix(vec3(1.00, 0.40, 0.10), vec3(1.00, 0.95, 0.85), (t - 0.8) / 0.2);
}

void main() {
    float zT = clamp((vWorldPos.z - uZmin) / max(uZmax - uZmin, 0.0001), 0.0, 1.0);
    vec2 heat = texture(uHeatProfile, zT).rg; // R = temperature, G = heat flux

    float fresnel = pow(1.0 - max(dot(normalize(vNormal), normalize(vViewDir)), 0.0), 2.0);

    vec3 color;
    float alpha = 1.0;

    if (uMode == 0) {
        // Wall heat map
        float temp = heat.r * uThrottle;
        color = heatRamp(temp);
        float emissive = heat.g * uThrottle;
        color += vec3(1.0, 0.5, 0.1) * emissive * 1.5;
        color += vec3(0.25, 0.75, 1.0) * fresnel * 0.2;
    } else {
        // Coolant flow - scrolling UV along Z
        float flow = fract(zT - uTime * 0.25);
        float tempGrad = zT;
        vec3 coolantCold = vec3(0.1, 0.4, 1.0);
        vec3 coolantHot = vec3(1.0, 0.6, 0.15);
        color = mix(coolantCold, coolantHot, tempGrad);
        float pulse = 0.6 + 0.4 * smoothstep(0.0, 0.2, flow) * (1.0 - smoothstep(0.2, 0.5, flow));
        color *= pulse;
        color += fresnel * 0.3;
        alpha = 0.85;
    }

    fragColor = vec4(color, alpha);
}
