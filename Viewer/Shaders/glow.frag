#version 460 core

// glow.frag - screen-space raymarch of volumetric plasma inside the
// chamber AABB. 3D hash noise animated by time, additive output.

in vec2 vUv;

uniform mat4 uInvViewProj;
uniform vec3 uCameraPos;
uniform float uTime;
uniform float uThrottle;
uniform vec3 uChamberMin;
uniform vec3 uChamberMax;

out vec4 fragColor;

float hash(vec3 p) {
    p = fract(p * 0.3183099 + vec3(0.1, 0.2, 0.3));
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

float noise3(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float n000 = hash(i);
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

float fbm(vec3 p) {
    float v = 0.0;
    float a = 0.5;
    for (int i = 0; i < 4; i++) {
        v += a * noise3(p);
        p *= 2.02;
        a *= 0.5;
    }
    return v;
}

vec2 intersectAABB(vec3 ro, vec3 rd, vec3 bmin, vec3 bmax) {
    vec3 invD = 1.0 / rd;
    vec3 t0 = (bmin - ro) * invD;
    vec3 t1 = (bmax - ro) * invD;
    vec3 tmin = min(t0, t1);
    vec3 tmax = max(t0, t1);
    float tNear = max(max(tmin.x, tmin.y), tmin.z);
    float tFar  = min(min(tmax.x, tmax.y), tmax.z);
    return vec2(tNear, tFar);
}

void main() {
    if (uThrottle <= 0.01) { fragColor = vec4(0.0); return; }

    vec4 nearNDC = vec4(vUv * 2.0 - 1.0, -1.0, 1.0);
    vec4 farNDC  = vec4(vUv * 2.0 - 1.0,  1.0, 1.0);
    vec4 nearWS = uInvViewProj * nearNDC; nearWS /= nearWS.w;
    vec4 farWS  = uInvViewProj * farNDC;  farWS  /= farWS.w;
    vec3 ro = uCameraPos;
    vec3 rd = normalize(farWS.xyz - nearWS.xyz);

    vec2 tRange = intersectAABB(ro, rd, uChamberMin, uChamberMax);
    if (tRange.y < tRange.x || tRange.y < 0.0) {
        fragColor = vec4(0.0);
        return;
    }
    tRange.x = max(tRange.x, 0.0);

    int steps = 20;
    float stepLen = (tRange.y - tRange.x) / float(steps);
    vec3 p = ro + rd * tRange.x;
    vec3 stp = rd * stepLen;

    vec3 accum = vec3(0.0);

    for (int i = 0; i < steps; i++) {
        vec3 np = p * 0.08 + vec3(0.0, 0.0, uTime * 0.8);
        float n = fbm(np);
        n = max(0.0, n - 0.35);
        float t = clamp(float(i) / float(steps), 0.0, 1.0);
        vec3 c = mix(vec3(1.6, 1.2, 0.6), vec3(1.2, 0.3, 0.05), t);
        accum += c * n * stepLen * 0.04;
        p += stp;
    }

    accum *= uThrottle;
    float alpha = clamp(length(accum) * 0.8, 0.0, 1.0);
    fragColor = vec4(accum, alpha);
}
