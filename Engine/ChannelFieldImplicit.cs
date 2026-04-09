// ChannelFieldImplicit.cs — v5: Live IImplicit evaluation for cooling channels
//
// Each voxel decides: "am I inside a channel?" based on local physics.
// No mesh intermediate. No pre-placed geometry. Physics → SDF → voxels.
//
// O(1) per voxel via cylindrical symmetry: all N channels are identical,
// rotated by 2π/N. Unwrap helix → fold into one period → evaluate.
//
// Pattern: same as RevolutionSDF (pre-sample z-dependent data, interpolate in fSignedDistance)

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public class ChannelFieldImplicit : IImplicit
{
    readonly AeroSpec _S;
    readonly int _nChannels;
    readonly float _angularPeriod;   // 2π / N
    readonly float _twistRate;       // radians per mm of z
    readonly float _zStart, _zEnd;
    readonly bool _isShroud;

    // Pre-sampled physics (RevolutionSDF pattern)
    readonly float[] _halfW;         // tangential half-width (mm)
    readonly float[] _halfH;         // radial half-height (mm)
    readonly float[] _rCenter;       // radial center of channel (mm)
    readonly float[] _superN;        // superellipse exponent
    readonly int _nSamples;
    readonly float _zStep;

    // Turbulator rib params
    readonly float _ribPitch;
    readonly float _ribHeight;
    readonly float _minHalfH;

    public ChannelFieldImplicit(AeroSpec S, bool isShroud)
    {
        _S = S;
        _isShroud = isShroud;
        _nChannels = isShroud ? S.nChannelsShroud : S.nChannelsSpike;
        _angularPeriod = 2f * MathF.PI / _nChannels;

        _zStart = S.zCowl + 2f;
        _zEnd = S.zInjector - 2f;

        float twistTurns = isShroud ? S.channelTwistTurns : 1.0f;
        float twistTotal = twistTurns * 2f * MathF.PI;
        _twistRate = twistTotal / (_zEnd - _zStart);

        _ribPitch = S.ribPitch;
        _ribHeight = S.ribHeight;
        _minHalfH = S.minChannel / 2f;

        // Pre-sample all z-dependent quantities
        _nSamples = 2000;
        _zStep = (_zEnd - _zStart) / (_nSamples - 1);
        _halfW = new float[_nSamples];
        _halfH = new float[_nSamples];
        _rCenter = new float[_nSamples];
        _superN = new float[_nSamples];

        float totalLen = _zEnd - _zStart;

        for (int i = 0; i < _nSamples; i++)
        {
            float z = _zStart + i * _zStep;

            float rSurface, wall;
            float halfW, halfH;

            if (isShroud)
            {
                rSurface = ChamberSizing.ShroudProfile(S, z);
                wall = HeatTransfer.WallThickness(S, z);
                var (w, h) = HeatTransfer.ChannelRect(S, z);
                halfW = w / 2f;
                halfH = h / 2f;
                _rCenter[i] = rSurface + wall + halfH;
            }
            else
            {
                rSurface = ChamberSizing.SpikeProfile(S, z);
                wall = HeatTransfer.WallThickness(S, z);
                var (w, h) = HeatTransfer.ChannelRectSpike(S, z);
                halfW = w / 2f;
                halfH = h / 2f;
                _rCenter[i] = rSurface - wall - halfH;
                if (_rCenter[i] < 2f) _rCenter[i] = 2f;
            }

            // Fade transitions: channels grow from collector at zStart,
            // merge into collector at zEnd. No dead ends.
            float fadeLen = MathF.Max((_zEnd - _zStart) * 0.08f, 5f); // 8% of length or 5mm
            float fadeIn = Smoothstep(_zStart, _zStart + fadeLen, z);
            float fadeOut = Smoothstep(_zEnd, _zEnd - fadeLen, z);
            float fade = fadeIn * fadeOut; // 0 at edges, 1 in middle

            // At edges (fade→0): channels expand to manifold size (merge with collector)
            float manifoldHalf = S.manifoldRadius * 0.8f;
            halfW = manifoldHalf + (halfW - manifoldHalf) * fade;
            halfH = manifoldHalf + (halfH - manifoldHalf) * fade;

            _halfW[i] = halfW;
            _halfH[i] = halfH;

            // Print-angle adaptive superellipse exponent
            float rLocal = _rCenter[i];
            float dzStep = totalLen / 80f; // match v4 nSpinePoints
            float tangStep = MathF.Max(rLocal, 3f) * (_twistRate * dzStep);
            float cosAngle = dzStep / MathF.Sqrt(dzStep * dzStep + tangStep * tangStep);
            _superN[i] = 1.5f + 2.5f * cosAngle;
        }
    }

    // Interpolate pre-sampled value at z (same pattern as RevolutionSDF.R())
    float Lerp(float[] arr, float z)
    {
        float t = (z - _zStart) / _zStep;
        int i = (int)t;
        if (i < 0) return arr[0];
        if (i >= _nSamples - 1) return arr[_nSamples - 1];
        float frac = t - i;
        return arr[i] + frac * (arr[i + 1] - arr[i]);
    }

    public float fSignedDistance(in Vector3 v)
    {
        // Fast reject: outside z range
        if (v.Z < _zStart || v.Z > _zEnd)
        {
            float dz = v.Z < _zStart ? _zStart - v.Z : v.Z - _zEnd;
            return dz + 1f; // positive = outside
        }

        // Step 1: Cylindrical coordinates
        float r = MathF.Sqrt(v.X * v.X + v.Y * v.Y);
        float phi = MathF.Atan2(v.Y, v.X);

        // Step 2: Unwrap helix — remove twist
        float phiUnwrapped = phi - _twistRate * v.Z;

        // Step 3: Fold into single channel period [-period/2, +period/2)
        float phiLocal = phiUnwrapped % _angularPeriod;
        if (phiLocal < 0f) phiLocal += _angularPeriod;
        phiLocal -= _angularPeriod / 2f;

        // Step 4: Interpolate z-dependent physics
        float halfW = Lerp(_halfW, v.Z);
        float halfH = Lerp(_halfH, v.Z);
        float rCenter = Lerp(_rCenter, v.Z);
        float n = Lerp(_superN, v.Z);

        // Per-channel modulation: channels near ports are wider
        // Ports: fuel at phi=0, LOX at phi=π, igniter at phi=π/2
        // Each port disrupts flow → neighbors compensate
        if (_isShroud)
        {
            float channelPhi = phi; // absolute angle of this point
            float minPortDist = MathF.PI; // max possible distance
            // Fuel port (phi=0)
            float d0 = MathF.Abs(AngleWrap(channelPhi));
            minPortDist = MathF.Min(minPortDist, d0);
            // LOX port (phi=π)
            float dPi = MathF.Abs(AngleWrap(channelPhi - MathF.PI));
            minPortDist = MathF.Min(minPortDist, dPi);
            // Igniter port (phi=π/2)
            float dHalf = MathF.Abs(AngleWrap(channelPhi - MathF.PI / 2f));
            minPortDist = MathF.Min(minPortDist, dHalf);

            // Convert angular distance to mm at channel radius
            float portDistMm = minPortDist * rCenter;
            // Modulation: wider near ports (within ~10mm), normal elsewhere
            float portMod = 1f + 0.35f * MathF.Exp(-portDistMm / 6f);
            halfW *= portMod;
            halfH *= portMod;
        }

        // Turbulator ribs: smooth modulation of halfH
        float arcLen = v.Z - _zStart;
        float ribPhase = arcLen % _ribPitch;
        if (ribPhase < 0f) ribPhase += _ribPitch;
        float ribFraction = ribPhase / _ribPitch;
        if (ribFraction < 0.25f)
        {
            // Smoothstep transitions at edges to avoid SDF discontinuities
            float edge = Smoothstep(0f, 0.05f, ribFraction)
                       * Smoothstep(0.25f, 0.20f, ribFraction);
            halfH = MathF.Max(halfH - _ribHeight / 2f * edge, _minHalfH);
        }

        // Step 5: Local 2D coordinates relative to channel center
        float dTangential = rCenter * phiLocal;  // arc-length in tangential direction
        float dRadial = r - rCenter;              // signed radial offset

        // Step 6: Superellipse SDF
        return SuperellipseSDF(dTangential, dRadial, halfW, halfH, n);
    }

    // Superellipse signed distance approximation
    // F = (|x/a|^n + |y/b|^n)^(1/n) - 1
    // Scaled by min(a,b) for approximate Euclidean distance
    static float SuperellipseSDF(float x, float y, float halfW, float halfH, float n)
    {
        if (halfW < 0.01f || halfH < 0.01f) return 1f; // degenerate

        float u = MathF.Abs(x) / halfW;
        float v = MathF.Abs(y) / halfH;

        // Avoid pow(0, n) edge cases
        float pu = u > 1e-6f ? MathF.Pow(u, n) : 0f;
        float pv = v > 1e-6f ? MathF.Pow(v, n) : 0f;
        float F = MathF.Pow(pu + pv, 1f / n) - 1f;

        // Scale to approximate world-space distance
        float scale = MathF.Min(halfW, halfH);
        return F * scale;
    }

    static float Smoothstep(float edge0, float edge1, float x)
    {
        float t = Math.Clamp((x - edge0) / (edge1 - edge0), 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    // Wrap angle to [-π, π]
    static float AngleWrap(float a)
    {
        while (a > MathF.PI) a -= 2f * MathF.PI;
        while (a < -MathF.PI) a += 2f * MathF.PI;
        return a;
    }

    // Bounding box for voxelization
    public BBox3 GetBBox()
    {
        float maxR = 0f;
        for (int i = 0; i < _nSamples; i++)
        {
            float rOuter = _rCenter[i] + (_isShroud ? _halfH[i] + 2f : _halfH[i] + 2f);
            maxR = MathF.Max(maxR, rOuter);
        }
        maxR += 3f; // margin

        return new BBox3(
            new Vector3(-maxR, -maxR, _zStart - 1f),
            new Vector3( maxR,  maxR, _zEnd + 1f));
    }
}
