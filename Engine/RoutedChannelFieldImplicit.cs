// SPDX-License-Identifier: AGPL-3.0-or-later
//
// OpenSpaceArch — Open Computational Architecture for Aerospace Hardware
// Copyright (C) 2025-2026 ultrasurreality
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// RoutedChannelFieldImplicit.cs — v5b: IImplicit for routed cooling channels
//
// THE ACTIVE CHANNEL FIELD. FluidFirst.BuildChannelVoids voxelizes this for both
// shroud and spike (mirroring Program.cs). Unlike ChannelFieldImplicit (which assumes
// perfect 2π/N helical symmetry and unwraps analytically), this works with arbitrary
// spine paths and uses a spatial grid for nearest-segment search.
//
// Spines come from ChannelRouter's ANALYTICAL helices (not SurfaceTurtle — that class
// is retained for reference only). The grid lookup means it also tolerates any routed
// path, should a future router emit non-helical spines.
//
// Cross-section per voxel: superellipse with print-angle-adaptive exponent, spatial
// θ-modulation of radial half-height, and smoothstep-faded turbulator ribs. All of
// these are CONTINUOUS functions of position — no per-channel step functions — so the
// rendered void stays watertight.
//
// Trade-off: O(1) helix unwrap → O(~1) grid lookup. Slightly slower but path-agnostic.

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public class RoutedChannelFieldImplicit : IImplicit
{
    readonly AeroSpec _S;
    readonly bool _isShroud;

    // All spine segments across all channels
    readonly Vector3[] _segA;   // segment start points
    readonly Vector3[] _segB;   // segment end points
    readonly int _nSegments;

    // NOTE: θ-modulation and width variation are applied as CONTINUOUS spatial
    // functions of voxel position inside fSignedDistance (see thetaMod below),
    // NOT as per-channel arrays. Per-channel multipliers were removed because a
    // per-spine step function in a distance field is discontinuous at spine
    // boundaries and breaks watertight marching cubes (the project's #1 failure mode).

    // Spatial grid for fast nearest-segment lookup
    readonly int _gridX, _gridY, _gridZ;
    readonly float _gridMinX, _gridMinY, _gridMinZ;
    readonly float _cellSize;
    readonly List<int>[] _grid;  // each cell → list of segment indices

    // Pre-sampled physics (same as ChannelFieldImplicit)
    readonly float[] _halfW;
    readonly float[] _halfH;
    readonly float[] _superN;
    readonly int _nSamples;
    readonly float _zStart, _zEnd, _zStep;
    readonly float _ribPitch, _ribHeight, _minHalfH;

    // Dome-skin guard: the channel cross-section is tapered continuously to ZERO over
    // [_zTaperStart, _zChTop] so the void cannot bulge up into the injector dome and
    // breach the outer skin where the engine radius is largest (the G3-thinwall fix).
    // Spines already end at zChTop (ChannelRouter), but the superellipse half-extents
    // would otherwise keep a finite blob at the top spine point — exactly the dome-base
    // pinch that fragmented the shell. The taper is a smoothstep of voxel position
    // (nearestZ), so fSignedDistance stays CONTINUOUS.
    readonly float _zChTop;
    readonly float _zTaperStart;

    // Throat-wall guard (G3-thinwall v2): the throat is the thinnest annulus and the
    // densest-channel region, so the channel cross-section + θ-modulation can leave ZERO
    // printable wall between the void and EITHER the hot-gas surface OR the outer skin
    // there (headless reported min wall 0.00 mm at the throat band). Inside a band around
    // zThroat we (a) fade the θ-modulation bulge toward 1.0 so the channel does not balloon
    // radially into the thin inner wall, and (b) hard-cap the radial half-height to the
    // local annulus budget so that >= minPrintWall always remains on BOTH sides. Both are
    // CONTINUOUS spatial functions of nearestZ (smoothstep envelope), so fSignedDistance
    // stays watertight. We do NOT delete channels here — cooling is needed most at the
    // throat; we only bound their cross-section so a wall survives.
    readonly float _zThroat;
    readonly float _throatBandHalf;     // mm — half-width of the guard band around zThroat

    // Bounding box
    float _bboxMinX, _bboxMinY, _bboxMinZ;
    float _bboxMaxX, _bboxMaxY, _bboxMaxZ;

    public RoutedChannelFieldImplicit(AeroSpec S, ChannelRouteResult route, bool isShroud)
    {
        _S = S;
        _isShroud = isShroud;
        _ribPitch = S.ribPitch;
        _ribHeight = S.ribHeight;
        _minHalfH = S.minChannel / 2f;

        var spines = route.Spines;

        // Collect all spine segments (flattened across all channels).
        var segAList = new List<Vector3>();
        var segBList = new List<Vector3>();

        foreach (var spine in spines)
        {
            for (int i = 0; i < spine.Count - 1; i++)
            {
                segAList.Add(spine[i]);
                segBList.Add(spine[i + 1]);
            }
        }

        _segA = segAList.ToArray();
        _segB = segBList.ToArray();
        _nSegments = _segA.Length;

        // Bounding box from segments
        _bboxMinX = _bboxMinY = _bboxMinZ = float.MaxValue;
        _bboxMaxX = _bboxMaxY = _bboxMaxZ = float.MinValue;
        float margin = 10f; // mm margin for channels + wall
        for (int i = 0; i < _nSegments; i++)
        {
            _bboxMinX = MathF.Min(_bboxMinX, MathF.Min(_segA[i].X, _segB[i].X));
            _bboxMinY = MathF.Min(_bboxMinY, MathF.Min(_segA[i].Y, _segB[i].Y));
            _bboxMinZ = MathF.Min(_bboxMinZ, MathF.Min(_segA[i].Z, _segB[i].Z));
            _bboxMaxX = MathF.Max(_bboxMaxX, MathF.Max(_segA[i].X, _segB[i].X));
            _bboxMaxY = MathF.Max(_bboxMaxY, MathF.Max(_segA[i].Y, _segB[i].Y));
            _bboxMaxZ = MathF.Max(_bboxMaxZ, MathF.Max(_segA[i].Z, _segB[i].Z));
        }
        _bboxMinX -= margin; _bboxMinY -= margin; _bboxMinZ -= margin;
        _bboxMaxX += margin; _bboxMaxY += margin; _bboxMaxZ += margin;

        // Build spatial grid
        _cellSize = 5f; // mm per cell
        _gridMinX = _bboxMinX; _gridMinY = _bboxMinY; _gridMinZ = _bboxMinZ;
        _gridX = (int)MathF.Ceiling((_bboxMaxX - _bboxMinX) / _cellSize) + 1;
        _gridY = (int)MathF.Ceiling((_bboxMaxY - _bboxMinY) / _cellSize) + 1;
        _gridZ = (int)MathF.Ceiling((_bboxMaxZ - _bboxMinZ) / _cellSize) + 1;

        int totalCells = _gridX * _gridY * _gridZ;
        _grid = new List<int>[totalCells];
        for (int c = 0; c < totalCells; c++)
            _grid[c] = new List<int>();

        // Insert segments into grid cells they overlap
        for (int i = 0; i < _nSegments; i++)
        {
            int x0 = CellX(MathF.Min(_segA[i].X, _segB[i].X) - 3f);
            int x1 = CellX(MathF.Max(_segA[i].X, _segB[i].X) + 3f);
            int y0 = CellY(MathF.Min(_segA[i].Y, _segB[i].Y) - 3f);
            int y1 = CellY(MathF.Max(_segA[i].Y, _segB[i].Y) + 3f);
            int z0 = CellZ(MathF.Min(_segA[i].Z, _segB[i].Z) - 3f);
            int z1 = CellZ(MathF.Max(_segA[i].Z, _segB[i].Z) + 3f);

            for (int cx = x0; cx <= x1; cx++)
            for (int cy = y0; cy <= y1; cy++)
            for (int cz = z0; cz <= z1; cz++)
            {
                int idx = GridIdx(cx, cy, cz);
                if (idx >= 0 && idx < totalCells)
                    _grid[idx].Add(i);
            }
        }

        // Pre-sample physics
        _zStart = _bboxMinZ + margin;
        _zEnd = _bboxMaxZ - margin;
        _nSamples = 2000;
        _zStep = (_zEnd - _zStart) / (_nSamples - 1);
        _halfW = new float[_nSamples];
        _halfH = new float[_nSamples];
        _superN = new float[_nSamples];

        float fadeLen = MathF.Max((_zEnd - _zStart) * 0.08f, 5f);
        float manifoldHalf = S.manifoldRadius * 0.8f;

        // Dome-skin guard band: taper the channel cross-section to zero as the spine
        // approaches zChTop. Band width = max(fadeLen, 6 mm) so the shrink is gradual
        // (continuous) yet complete before the dome base. The collector manifold is added
        // separately (FluidFirst.BuildShroudCollector), so the channel field itself does
        // NOT need to keep a finite blob at the top.
        _zChTop = S.zChTop;
        _zTaperStart = _zChTop - MathF.Max(fadeLen, 6f);

        // Throat guard band: span ±throatBandHalf around zThroat. Width is scaled to the
        // convergent geometry (a few mm either side of the throat) and floored at 4 mm so
        // the cap is gradual. The band must comfortably contain the headless failure point
        // (~z=26, zThroat≈24.5) and the densest-channel convergent region just above it.
        _zThroat = S.zThroat;
        _throatBandHalf = MathF.Max(0.25f * S.convergentDz, 4f);

        for (int i = 0; i < _nSamples; i++)
        {
            float z = _zStart + i * _zStep;
            float halfW, halfH;
            if (isShroud)
            {
                var (w, h) = HeatTransfer.ChannelRect(S, z);
                halfW = w / 2f;
                halfH = h / 2f;
            }
            else
            {
                var (w, h) = HeatTransfer.ChannelRectSpike(S, z);
                halfW = w / 2f;
                halfH = h / 2f;
            }

            // Fade-IN at the inlet edge: channels emerge from the inlet collector.
            float fadeIn = Smoothstep(_zStart, _zStart + fadeLen, z);
            halfW = manifoldHalf + (halfW - manifoldHalf) * fadeIn;
            halfH = manifoldHalf + (halfH - manifoldHalf) * fadeIn;

            // Fade-OUT at the top edge tapers all the way to ZERO (not manifoldHalf) so the
            // void closes before the dome — prevents the dome-base skin breach (G3-thinwall).
            // (The continuous spatial taper on nearestZ in fSignedDistance is the primary
            // guard; this pre-sample fade keeps the sampled extents consistent with it.)
            float topClose = Smoothstep(_zChTop, _zTaperStart, z); // 1 below taper, 0 at zChTop
            halfW *= topClose;
            halfH *= topClose;

            // Throat radial-budget cap (G3-thinwall v2): in the throat band, the annular gas
            // gap is the thinnest in the engine, so the RADIAL channel half-height must leave
            // >= minPrintWall to BOTH the gas surface and the outer skin. The router places the
            // spine center at rShroud + wall + h/2 (or, on the spike, rSpike - wall - h/2);
            // the remaining radial room before the channel touches the gas side is therefore
            // (wall - minPrintWall) on the inner side. The cap clamps the SAMPLED halfH so the
            // channel cannot eat that margin even before θ-modulation is applied. It is blended
            // by the SAME throat-band envelope used per-voxel in fSignedDistance, so (1) it only
            // affects the throat band — channels keep full size in the chamber — and (2) the
            // pre-sampled extents stay consistent with the per-voxel guard so the two never
            // disagree. Blending min(halfH,capH) by the envelope is continuous in z.
            float bandEnv = ThroatBandEnvelope(z);
            if (bandEnv > 0f)
            {
                float capH = ThroatHalfHCap(S, z, isShroud);
                float cappedH = MathF.Min(halfH, capH);
                halfH = halfH + (cappedH - halfH) * bandEnv;
            }

            _halfW[i] = halfW;
            _halfH[i] = halfH;
            _superN[i] = 2.5f;
        }
    }

    /// Maximum printable RADIAL half-height for a cooling channel at axial station z,
    /// such that >= minPrintWall remains between the channel void and the hot-gas surface.
    /// Returns a large value (no cap) outside the throat band. CONTINUOUS in z (the band
    /// edges are blended by the smoothstep envelope applied at the call site / in
    /// fSignedDistance), so it does not introduce an SDF step. The skin-side wall is grown
    /// separately by VariableWallImplicit and is always >= minPrintWall; this cap protects
    /// the GAS side, which mutual-exclusion only guarantees for the un-bulged cross-section.
    static float ThroatHalfHCap(AeroSpec S, float z, bool isShroud)
    {
        // The wall(z) Barlow thickness IS the nominal gas-side gap the router reserved.
        // Keep >= minPrintWall of it as solid metal: the channel may use at most
        // (wall - minPrintWall) of radial room on the inner side before the spine center,
        // i.e. halfH <= wall - minPrintWall. Floor at a small positive value so the channel
        // never fully closes inside the band (cooling is needed most at the throat).
        float wall = HeatTransfer.WallThickness(S, z);
        float cap = wall - S.minPrintWall;
        // Never cap below the powder-removal minimum half-height, and never below a hard
        // floor of half the min channel — a throat channel must still pass coolant.
        float floor = MathF.Max(S.minChannel / 2f, 0.5f);
        return MathF.Max(cap, floor);
    }

    /// Smooth "tent" envelope centred on the throat: 1.0 inside the band core, ramping
    /// continuously to 0.0 at z = zThroat ± throatBandHalf and staying 0 beyond. Built from
    /// two opposing smoothsteps so it is C1-continuous everywhere — no SDF step. Used to blend
    /// the θ-modulation fade and the radial cap in/out of the throat region. An inner flat
    /// core (60% of the half-band) keeps the cap fully active across the thinnest annulus,
    /// while the outer 40% ramps the guard out gradually toward the chamber/convergent.
    float ThroatBandEnvelope(float z)
    {
        float d = MathF.Abs(z - _zThroat);
        if (d >= _throatBandHalf) return 0f;
        float core = 0.6f * _throatBandHalf;      // fully-on region |z-zThroat| <= core
        if (d <= core) return 1f;
        // Ramp from 1 (at core) down to 0 (at band edge), continuous at both ends.
        return Smoothstep(_throatBandHalf, core, d);
    }

    int CellX(float x) => Math.Clamp((int)((x - _gridMinX) / _cellSize), 0, _gridX - 1);
    int CellY(float y) => Math.Clamp((int)((y - _gridMinY) / _cellSize), 0, _gridY - 1);
    int CellZ(float z) => Math.Clamp((int)((z - _gridMinZ) / _cellSize), 0, _gridZ - 1);
    int GridIdx(int cx, int cy, int cz) => cx + cy * _gridX + cz * _gridX * _gridY;

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
        // Find nearest segment via grid
        int cx = CellX(v.X);
        int cy = CellY(v.Y);
        int cz = CellZ(v.Z);
        int cellIdx = GridIdx(cx, cy, cz);

        if (cellIdx < 0 || cellIdx >= _grid.Length)
            return 10f; // far outside

        float minDist2 = float.MaxValue;
        float nearestZ = v.Z;
        Vector3 nearestClosest = v;
        Vector3 nearestTangent = Vector3.UnitZ;

        var candidates = _grid[cellIdx];
        for (int c = 0; c < candidates.Count; c++)
        {
            int segIdx = candidates[c];
            Vector3 ab = _segB[segIdx] - _segA[segIdx];
            Vector3 ap = v - _segA[segIdx];
            float t = Math.Clamp(Vector3.Dot(ap, ab) / Vector3.Dot(ab, ab), 0f, 1f);
            Vector3 closest = _segA[segIdx] + ab * t;
            float d2 = Vector3.DistanceSquared(v, closest);
            if (d2 < minDist2)
            {
                minDist2 = d2;
                nearestZ = closest.Z;
                nearestClosest = closest;
                nearestTangent = ab;
            }
        }

        if (minDist2 > 100f)
            return MathF.Sqrt(minDist2);

        // Local frame decomposition at nearest spine point:
        // tangent = along spine segment
        // radial = from engine axis (Z) toward spine point (outward)
        // tangential = cross(tangent, radial) — along the surface circumference
        Vector3 tangent = nearestTangent;
        if (tangent.LengthSquared() < 1e-10f) tangent = Vector3.UnitZ;
        tangent = Vector3.Normalize(tangent);

        // Radial direction: from Z axis to spine point (in XY plane)
        Vector3 radialDir = new(nearestClosest.X, nearestClosest.Y, 0f);
        if (radialDir.LengthSquared() < 1e-10f) radialDir = Vector3.UnitX;
        radialDir = Vector3.Normalize(radialDir);

        // Tangential direction: perpendicular to both tangent and radial
        Vector3 tangentialDir = Vector3.Cross(tangent, radialDir);
        if (tangentialDir.LengthSquared() < 1e-10f)
            tangentialDir = Vector3.Cross(tangent, Vector3.UnitX);
        tangentialDir = Vector3.Normalize(tangentialDir);

        // Recompute radial to ensure orthogonal frame
        radialDir = Vector3.Cross(tangentialDir, tangent);
        radialDir = Vector3.Normalize(radialDir);

        // Project offset vector onto local frame
        Vector3 offset = v - nearestClosest;
        float dRadial = Vector3.Dot(offset, radialDir);
        float dTangential = Vector3.Dot(offset, tangentialDir);

        // Channel dimensions at this z.
        // θ-modulation is applied as SPATIAL function of voxel position (not per-channel):
        //   thetaMod = 1 + amp · cos(harmonic · atan2(v.Y, v.X))
        // This gives correlated azimuthal bulge (hot side thicker, cold side thinner) while
        // keeping the distance field CONTINUOUS — no spine jumping discontinuities that would
        // break marching cubes. Per-channel width jitter is intentionally omitted because it
        // is fundamentally discontinuous (step function at spine boundaries breaks watertight).
        float phiVoxel = MathF.Atan2(v.Y, v.X);
        // θ-modulation bulge — faded toward 1.0 inside the throat band. The bipolar bulge
        // (amplitude up to heatFluxAngularAmplitude) is what eats the thin inner wall where
        // the annulus is tightest; throatFade (1 at the throat → 0 outside the band) scales
        // the DEVIATION from 1, not the channel itself, so cooling cross-section is preserved
        // away from the throat. Smoothstep envelope ⇒ CONTINUOUS, no SDF step.
        float throatFade = ThroatBandEnvelope(nearestZ);         // 1 in the band core → 0 outside
        float thetaDev = _S.heatFluxAngularAmplitude
                            * MathF.Cos(_S.heatFluxAngularHarmonic * phiVoxel);
        float thetaMod = 1f + thetaDev * (1f - throatFade);
        float halfW = Lerp(_halfW, nearestZ);                    // tangential half-width
        float halfH = Lerp(_halfH, nearestZ) * thetaMod;         // radial half-height (θ-modulated by position)
        float n = Lerp(_superN, nearestZ);

        // Throat radial-budget cap (G3-thinwall v2), re-applied per voxel AFTER θ-modulation
        // so the bulge can never push the channel back past the gas-side wall budget. The cap
        // is blended in over the band by throatFade, so outside the throat it is a no-op and
        // the field stays continuous. capH already accounts for wall(z) - minPrintWall.
        if (throatFade > 0f)
        {
            float capH = ThroatHalfHCap(_S, nearestZ, _isShroud);
            float cappedH = MathF.Min(halfH, capH);
            halfH = halfH + (cappedH - halfH) * throatFade;      // = capH only at band core
        }

        // Dome-skin taper (G3-thinwall): continuous shrink of the cross-section to ZERO
        // over [_zTaperStart, _zChTop], evaluated on the spine z. Smoothstep → C1-continuous,
        // so the distance field stays watertight. This closes the channel void before the
        // injector dome so it cannot breach the outer skin at the largest engine radius.
        float topTaper = Smoothstep(_zChTop, _zTaperStart, nearestZ); // 1 below taper, 0 at zChTop
        halfW *= topTaper;
        halfH *= topTaper;

        // Turbulator ribs. The rib floor must follow the taper (scale _minHalfH by topTaper)
        // so a fully-closed channel near the dome is NOT re-inflated back up to _minHalfH.
        float arcLen = nearestZ - _zStart;
        float ribPhase = arcLen % _ribPitch;
        if (ribPhase < 0f) ribPhase += _ribPitch;
        if (ribPhase / _ribPitch < 0.25f)
        {
            float edge = Smoothstep(0f, 0.05f, ribPhase / _ribPitch)
                       * Smoothstep(0.25f, 0.20f, ribPhase / _ribPitch);
            halfH = MathF.Max(halfH - _ribHeight / 2f * edge * topTaper, _minHalfH * topTaper);
        }

        // SuperellipseSDF with proper local decomposition
        return SuperellipseSDF(dTangential, dRadial, halfW, halfH, n);
    }

    static float SuperellipseSDF(float x, float y, float halfW, float halfH, float n)
    {
        if (halfW < 0.01f || halfH < 0.01f) return 1f;
        float u = MathF.Abs(x) / halfW;
        float v = MathF.Abs(y) / halfH;
        float pu = u > 1e-6f ? MathF.Pow(u, n) : 0f;
        float pv = v > 1e-6f ? MathF.Pow(v, n) : 0f;
        float F = MathF.Pow(pu + pv, 1f / n) - 1f;
        return F * MathF.Min(halfW, halfH);
    }

    static float Smoothstep(float edge0, float edge1, float x)
    {
        float t = Math.Clamp((x - edge0) / (edge1 - edge0), 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    public BBox3 GetBBox()
    {
        return new BBox3(
            new Vector3(_bboxMinX, _bboxMinY, _bboxMinZ),
            new Vector3(_bboxMaxX, _bboxMaxY, _bboxMaxZ));
    }
}
