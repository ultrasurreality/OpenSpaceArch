// VariableWallImplicit.cs — Per-voxel wall thickness from Barlow
//
// Replaces global voxOffset(wallThickness) in Step 4.
// Wall = f(z, r) from pressure formula. Thicker at high pressure, thinner where cooling works.
//
// Uses ScalarField(Voxels) for O(1) SDF queries (OpenVDB tree lookup).

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public class VariableWallImplicit : IImplicit
{
    readonly ScalarField _sdfVoids;  // SDF of all combined voids
    readonly AeroSpec _S;

    // Pre-sampled wall thickness
    readonly float[] _wallByZ;
    readonly int _nSamples;
    readonly float _zStart, _zEnd, _zStep;

    public VariableWallImplicit(Voxels voxAllVoids, AeroSpec S)
    {
        _sdfVoids = new ScalarField(voxAllVoids); // O(1) SDF queries via OpenVDB
        _S = S;

        _zStart = S.zTip;
        _zEnd = S.zTotal;
        _nSamples = 2000;
        _zStep = (_zEnd - _zStart) / (_nSamples - 1);
        _wallByZ = new float[_nSamples];

        for (int i = 0; i < _nSamples; i++)
        {
            float z = _zStart + i * _zStep;
            _wallByZ[i] = HeatTransfer.WallThickness(S, z);
        }
    }

    float WallAtZ(float z)
    {
        float t = (z - _zStart) / _zStep;
        int i = (int)t;
        if (i < 0) return _wallByZ[0];
        if (i >= _nSamples - 1) return _wallByZ[_nSamples - 1];
        float frac = t - i;
        return _wallByZ[i] + frac * (_wallByZ[i + 1] - _wallByZ[i]);
    }

    public float fSignedDistance(in Vector3 v)
    {
        // Query SDF of voids via ScalarField
        // bGetValue returns false if point is outside VDB narrow band
        if (!_sdfVoids.bGetValue(v, out float sdfRaw))
            return 5f; // outside tracked region → definitely outside shell

        float dVoid = sdfRaw * Library.fVoxelSizeMM; // convert to mm

        // Wall thickness at this point
        float wall = WallAtZ(v.Z);

        // Extra thickness at structural zones — applied as CONTINUOUS smoothstep ramps,
        // NOT hard if-steps. A discontinuous step in wall(z) puts a step in the offset
        // surface SDF (dVoid - wall) and risks a non-watertight seam at the step plane.
        //
        // Dome reinforcement (G3-thinwall): the injector dome breached at the dome BASE
        // (~zChTop), not just at the faceplate. Ramp the structural multiplier in across
        // the dome (peaking ×1.5 at the faceplate) so the dome wall is thick where the
        // channel void used to pinch the skin.
        //
        // v2 FRAGMENTATION FIX: the channel cross-section is tapered to ZERO over
        // [zChTop - ~6 mm, zChTop] (RoutedChannelFieldImplicit dome taper). Previously the
        // dome reinforcement only began AT zChTop, leaving a thin-wall HANDOFF band just
        // below zChTop where the channels are nearly pinched out but the wall has not yet
        // thickened — there the metal between adjacent closing channels can fragment
        // (body_count 28→48). Start the ramp domePreRoll (6 mm) BELOW zChTop so the wall is
        // already thickening as the channels close, fusing the metal across the handoff and
        // keeping the shell connected. Still a continuous smoothstep (no step); the +0%→+50%
        // multiplier only grows the wall, so it cannot thin anything below the Barlow value.
        const float domePreRoll = 6f; // mm — matches the channel-close band floor (max(fadeLen,6))
        float domeRamp = Smoothstep(_S.zChTop - domePreRoll, _S.zInjector, v.Z); // 0 below band → 1 at faceplate
        wall *= 1f + 0.5f * domeRamp;

        // Tip reinforcement: ramp ×1.3 in over the bottom 3 mm (continuous, no step).
        float tipRamp = Smoothstep(_S.zTip + 3f, _S.zTip, v.Z);    // 0 above the band → 1 at the tip
        wall *= 1f + 0.3f * tipRamp;

        // Hard floor: the emitted wall is NEVER thinner than the LPBF minimum, EVERYWHERE
        // (this clamp runs AFTER every multiplier, so it applies at the injector/dome too).
        wall = MathF.Max(wall, _S.minPrintWall);

        // Outer shell: inside if within 'wall' distance of void surface
        return dVoid - wall;
    }

    static float Smoothstep(float edge0, float edge1, float x)
    {
        float t = Math.Clamp((x - edge0) / (edge1 - edge0), 0f, 1f);
        return t * t * (3f - 2f * t);
    }
}
