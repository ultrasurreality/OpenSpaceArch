#version 460 core

// sdf_raymarch.frag - sphere-trace the analytical engine body in real time.
// Reads spike/shroud/wall/shell profiles from a 1D RGBA texture (built from
// AeroSpec via GpuProfileTextures). No PicoGK voxelization required.
//
// Body SDF is the same composition as EngineBodyImplicit minus channel voids:
//   dGas = max(rShroud - rPos, rPos - rSpike)   annular gas-path SDF
//   dSolid = max(-dGas, dGas - shellT)          shell band of width shellT
//
// Output is alpha-blended on top of clear color so PicoGK mesh can draw over
// once it arrives. Color is the hologram blue / metallic copper depending on
// uHoloBlend uniform driven by the runtime.

in vec2 vUv;

uniform mat4 uInvViewProj;
uniform vec3 uCameraPos;
uniform float uTime;
uniform float uHoloBlend;       // 1.0 = pure hologram preview, 0.0 = invisible (mesh dominant)
uniform float uZmin;
uniform float uZmax;
uniform float uRmax;            // outer radial bound for early-out
uniform sampler1D uProfile;     // R=rSpike G=rShroud B=wallT A=shellT
uniform vec3 uHoloColor;
uniform vec3 uMetalColor;

out vec4 fragColor;

vec4 sampleProfile(float z) {
    float zN = clamp((z - uZmin) / max(uZmax - uZmin, 0.0001), 0.0, 1.0);
    return texture(uProfile, zN);
}

float sdEngine(vec3 p) {
    // Out of axial range -> distance to nearest cap (approx)
    if (p.z < uZmin) return uZmin - p.z;
    if (p.z > uZmax) return p.z - uZmax;

    vec4 prof = sampleProfile(p.z);
    float rSpike  = prof.r;
    float rShroud = prof.g;
    float shellT  = max(prof.a, 0.5);

    float rPos = length(p.xy);

    // Annular gas path SDF: positive when OUTSIDE the gas annulus
    float dShroud = rPos - rShroud;     // positive outside shroud
    float dSpike  = rPos - rSpike;      // positive outside spike (rSpike may be 0 above dome)
    float dGas;
    if (rSpike < 0.05) {
        // dome region or below tip - no spike, gas extends to axis
        dGas = dShroud;
    } else {
        dGas = max(-dShroud, dSpike);
    }

    // Solid shell of width shellT around the gas path (the engine wall + cooling jacket)
    return max(-dGas, dGas - shellT);
}

vec3 sdfNormal(vec3 p) {
    float e = 0.4;
    return normalize(vec3(
        sdEngine(p + vec3(e, 0, 0)) - sdEngine(p - vec3(e, 0, 0)),
        sdEngine(p + vec3(0, e, 0)) - sdEngine(p - vec3(0, e, 0)),
        sdEngine(p + vec3(0, 0, e)) - sdEngine(p - vec3(0, 0, e))
    ));
}

bool raymarch(vec3 ro, vec3 rd, out vec3 hitP, out float hitT) {
    float t = 0.0;
    for (int i = 0; i < 96; i++) {
        vec3 p = ro + rd * t;
        // Cheap radial early-out
        float rPos = length(p.xy);
        if (rPos > uRmax + 5.0 && (p.z < uZmin || p.z > uZmax)) {
            t += max(rPos - uRmax, 1.0);
            continue;
        }
        float d = sdEngine(p);
        if (d < 0.05) {
            hitP = p;
            hitT = t;
            return true;
        }
        if (t > 5000.0) break;
        t += max(d * 0.85, 0.25);
    }
    return false;
}

void main() {
    if (uHoloBlend < 0.01) { fragColor = vec4(0.0); return; }

    // Build world-space ray through the pixel
    vec4 nearNDC = vec4(vUv * 2.0 - 1.0, -1.0, 1.0);
    vec4 farNDC  = vec4(vUv * 2.0 - 1.0,  1.0, 1.0);
    vec4 nearWS = uInvViewProj * nearNDC; nearWS /= nearWS.w;
    vec4 farWS  = uInvViewProj * farNDC;  farWS  /= farWS.w;
    vec3 ro = uCameraPos;
    vec3 rd = normalize(farWS.xyz - nearWS.xyz);

    vec3 hitP;
    float hitT;
    if (!raymarch(ro, rd, hitP, hitT)) {
        fragColor = vec4(0.0);
        return;
    }

    vec3 N = sdfNormal(hitP);
    vec3 V = normalize(uCameraPos - hitP);
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 2.5);
    float NdotV = max(dot(N, V), 0.0);

    // Animated scan band along Z to make it feel "live"
    float scan = 0.5 + 0.5 * sin(hitP.z * 0.3 - uTime * 1.5);
    scan = pow(scan, 4.0) * 0.6;

    vec3 base = uHoloColor * (0.18 + NdotV * 0.6);
    vec3 rim  = uHoloColor * fresnel * 1.8;
    vec3 col  = base + rim + uHoloColor * scan;

    float alpha = (0.55 + 0.45 * fresnel) * uHoloBlend;
    fragColor = vec4(col, alpha);
}
