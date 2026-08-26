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

// PhysicsFields.cs — 3D physics maps for every voxel of the engine
//
// Turns 1D physics (functions of z) into full 3D fields.
// Each solid voxel gets: temperature, stress, heat flux, coolant direction.
// Uses PicoGK ScalarField/VectorField on the same voxel grid as geometry.
// Pattern: LEAP 71 SimulationExample — CEM exports physics WITH geometry.

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public sealed class PhysicsFields
{
    public ScalarField? Temperature { get; private set; }
    public ScalarField? Stress { get; private set; }
    public ScalarField? HeatFlux { get; private set; }
    public VectorField? CoolantFlow { get; private set; }
    public ScalarField? MeasuredWall { get; private set; }
    public float MeasuredWallMin { get; private set; }
    public float MeasuredWallMinZ { get; private set; }
    public int VoxelCount { get; private set; }
    public int ThinWallViolations { get; private set; }
    public int OverhangViolations { get; private set; }
    public float OverhangMaxAngle { get; private set; }
    // Honest breakdown of overhang violations (see MeasureOverhang for the
    // geometric rationale). "Expected" = surfaces that an aerospike printed
    // spike-tip-down legitimately has and that LPBF handles with minimal/local
    // support (nozzle-exit cowl lip, outer shroud flare, top-flange underside).
    // "Problematic" = shallow downward faces elsewhere (e.g. internal channel
    // roofs, mid-body) that are the ones actually worth acting on.
    public int OverhangExpected { get; private set; }
    public int OverhangProblematic { get; private set; }
    public float OverhangSteepestProblematicAngle { get; private set; }

    public void Compute(AeroSpec S, Voxels voxEngine,
                        RevolutionSDF shroudSDF, RevolutionSDF spikeSDF,
                        IImplicit? channelsShroud)
    {
        Library.Log("Computing 3D physics fields...");

        var temp = new ScalarField();
        var stress = new ScalarField();
        var heatFlux = new ScalarField();
        var coolant = new VectorField();
        int count = 0;

        // Create SDF from engine voxels to get TraverseActive
        var sdf = new ScalarField(voxEngine);

        float Tgas = S.Tc * MathF.Pow(S.Pr_gas, 0.33f);
        float Tcoolant = 300f;
        float twistRate = S.channelTwistTurns * 2f * MathF.PI
            / MathF.Max(S.zTotal, 1f);

        sdf.TraverseActive(new Visitor(
            S, shroudSDF, spikeSDF, channelsShroud,
            temp, stress, heatFlux, coolant,
            Tgas, Tcoolant, twistRate, ref count));

        Temperature = temp;
        Stress = stress;
        HeatFlux = heatFlux;
        CoolantFlow = coolant;
        VoxelCount = count;

        Library.Log($"  {count} voxels filled: Temperature, Stress, HeatFlux, CoolantFlow");
    }

    /// <summary>
    /// MEASURE real wall thickness on final voxels — not formula, geometry.
    /// For each point sampled inside a channel void, read the engine SDF at
    /// that point. |SDF| = distance to the NEAREST solid surface (gas-side or
    /// outer, whichever is closer), which is the local solid wall thickness.
    /// This is a field lookup, not an outward ray-cast. Compare with the
    /// formula prediction and flag violations below the min printable wall.
    /// </summary>
    public void MeasureWallThickness(AeroSpec S, Voxels voxEngine, IImplicit? channelsShroud)
    {
        Library.Log("Measuring real wall thickness on final voxels...");

        var measuredWall = new ScalarField();
        float minWall = float.MaxValue;
        float minWallZ = 0f;
        int violations = 0;
        int measured = 0;

        // SDF of the final engine — value = distance to nearest surface
        var engineSdf = new ScalarField(voxEngine);

        // We need to sample points INSIDE channels (where dChannel < 0)
        // and measure their distance to the outer surface of the engine.
        // Strategy: iterate over a grid, check if point is inside a channel,
        // if yes — read the engine SDF at that point = distance to solid surface.

        if (channelsShroud == null) return;

        voxEngine.CalculateProperties(out _, out BBox3 bbox);
        float step = S.voxelSize;

        for (float z = bbox.vecMin.Z; z <= bbox.vecMax.Z; z += step * 2f)
        {
            for (float y = bbox.vecMin.Y; y <= bbox.vecMax.Y; y += step * 2f)
            {
                for (float x = bbox.vecMin.X; x <= bbox.vecMax.X; x += step * 2f)
                {
                    Vector3 pos = new(x, y, z);
                    float dCh = channelsShroud.fSignedDistance(pos);

                    // Measure at points clearly inside channel (1-3mm deep)
                    // Avoid transition zone (|dCh| < 1) where SDF is noisy
                    if (dCh > -1f || dCh < -3f) continue;

                    // Engine SDF at this point = distance to nearest solid surface.
                    // Since this point is inside a channel void (surrounded by solid),
                    // |SDF| gives the distance to the nearest surface — gas-side or
                    // outer, whichever is closer — i.e. the local wall thickness.
                    if (!engineSdf.bGetValue(pos, out float sdfVal)) continue;

                    // sdfVal is the signed distance. Near channel surface inside
                    // the engine, it should be negative (inside solid). The absolute
                    // value = thickness of solid material between this point and
                    // the nearest surface (either gas side or outer surface).
                    float wallMeasured = MathF.Abs(sdfVal) * S.voxelSize;

                    measuredWall.SetValue(pos, wallMeasured);
                    measured++;

                    if (wallMeasured < minWall)
                    {
                        minWall = wallMeasured;
                        minWallZ = z;
                    }

                    if (wallMeasured < S.minPrintWall)
                        violations++;
                }
            }
        }

        MeasuredWall = measuredWall;
        MeasuredWallMin = minWall;
        MeasuredWallMinZ = minWallZ;
        ThinWallViolations = violations;

        // Diagnostic: find worst point and report details
        Vector3 worstPos = Vector3.Zero;
        float worstSdf = 0f;
        float worstDch = 0f;
        // Re-scan to find the worst point with full info
        for (float zs = bbox.vecMin.Z; zs <= bbox.vecMax.Z; zs += step * 2f)
        for (float ys = bbox.vecMin.Y; ys <= bbox.vecMax.Y; ys += step * 2f)
        for (float xs = bbox.vecMin.X; xs <= bbox.vecMax.X; xs += step * 2f)
        {
            Vector3 p = new(xs, ys, zs);
            float dc = channelsShroud.fSignedDistance(p);
            if (dc > -1f || dc < -3f) continue;
            if (!engineSdf.bGetValue(p, out float sv)) continue;
            float wm = MathF.Abs(sv) * S.voxelSize;
            if (wm <= minWall + 0.001f && MathF.Abs(zs - minWallZ) < 2f)
            { worstPos = p; worstSdf = sv; worstDch = dc; break; }
        }

        float formulaWall = HeatTransfer.WallThickness(S, minWallZ);
        Library.Log($"  Measured {measured} channel-surface points");
        Library.Log($"  Min wall: {minWall:F2} mm at z={minWallZ:F1} (formula predicted {formulaWall:F2} mm)");
        if (worstPos != Vector3.Zero)
        {
            float r = MathF.Sqrt(worstPos.X * worstPos.X + worstPos.Y * worstPos.Y);
            Library.Log($"  Worst point: ({worstPos.X:F1}, {worstPos.Y:F1}, {worstPos.Z:F1}) r={r:F1}mm");
            Library.Log($"    engineSDF={worstSdf:F3}, dChannel={worstDch:F3}");
            Library.Log($"    ShroudProfile={ChamberSizing.ShroudProfile(S, worstPos.Z):F1}mm, SpikeProfile={ChamberSizing.SpikeProfile(S, worstPos.Z):F1}mm");
        }
        if (violations > 0)
            Library.Log($"  WARNING: {violations} points below min printable wall ({S.minPrintWall} mm)!");
        else
            Library.Log($"  All points above min printable wall ({S.minPrintWall} mm)");
    }

    /// <summary>
    /// Measure overhang angles on the final engine surface for LPBF self-support.
    ///
    /// BUILD ORIENTATION (this is the load-bearing assumption — get it wrong and the
    /// whole metric is meaningless). The engine is modelled and PRINTED spike-tip-down:
    /// the spike apex sits at z = zTip = 0 (the minimum z, on the build plate) and the
    /// injector/flange is at z = zInjector ≈ zMax (the top). Equivalently, the build
    /// axis is +Z pointing UP. This is not an arbitrary convention — ChamberSizing
    /// derives the injector-dome closure as a 45° self-supporting cone that closes
    /// going UP toward +Z (domeDz = rSpikeChamber ⇒ 45° from horizontal), which only
    /// self-supports when +Z is up. So +Z-up == spike-tip-down, and the surface-normal
    /// Z component below is already in the correct print frame.
    ///
    /// ANGLE CONVENTION (the bug that inflated the old count). The previous version
    /// computed acos(|n.Z|), called it "angle from vertical", and flagged it when it
    /// EXCEEDED maxOverhang. That is inverted: acos(|n.Z|) is actually the surface's
    /// inclination FROM HORIZONTAL (β). A flat downward ceiling (n=(0,0,-1)) gives
    /// β = acos(1) = 0° — the WORST, fully-unsupported case — yet the old test passed
    /// it (0 < 45). A near-vertical wall (n.Z≈0) gives β = acos(0) = 90° — the BEST,
    /// fully self-supporting case — yet the old test FLAGGED it (90 > 45). An aerospike's
    /// outer shroud and spike flanks are mostly steep near-vertical walls, so the old
    /// metric mislabeled the self-supporting bulk of the surface as ~117k "violations".
    ///
    /// CORRECT RULE (standard AM): for a downward-facing surface, the self-support
    /// limit is on the inclination from horizontal. A surface is self-supporting when
    /// β ≥ (90° − maxOverhang); it is a genuine overhang when β < (90° − maxOverhang)
    /// (with maxOverhang = 45° this threshold is 45°, matching the dome design).
    /// We do NOT alter geometry to chase this number — physics-first stands. We only
    /// measure it honestly and split the count into inherent-but-expected vs problematic.
    /// </summary>
    public void MeasureOverhang(AeroSpec S, Voxels voxEngine)
    {
        Library.Log("Measuring overhang angles on final surface (build: spike-tip-down, +Z up)...");

        var engineSdf = new ScalarField(voxEngine);
        int measured = 0;
        int violations = 0;
        int expected = 0;
        int problematic = 0;
        float maxAngleFromHoriz = 90f;          // track the SHALLOWEST (worst) downward face
        float steepestProblematic = 90f;        // shallowest problematic face = worst actionable case
        float vox = S.voxelSize;

        // Self-support limit on inclination from horizontal: surfaces shallower than
        // this need support. maxOverhang=45 → 45°.  (β < limit ⇒ overhang violation.)
        float limitFromHoriz = 90f - S.maxOverhang;

        voxEngine.CalculateProperties(out _, out BBox3 bbox);

        // Axial bands that legitimately carry expected overhangs on a tip-down aerospike:
        //   - nozzle-exit cowl lip: the shroud trailing edge near zCowl faces partly down.
        //   - top-flange underside: the mount plate at zInjector overhangs the shroud.
        // A small axial tolerance (a few mm) brackets each feature band.
        float band = MathF.Max(4f, 4f * vox);
        float cowlLipZ = S.zCowl;               // nozzle-exit / cowl trailing edge (low z)
        float flangeZ  = S.zInjector;           // flange underside (high z)
        float rShroudMax = MathF.Max(S.rShroudChamber, S.rShroudThroat);

        // Sample surface points: where SDF is near zero (within ~1 voxel of surface)
        for (float z = bbox.vecMin.Z; z <= bbox.vecMax.Z; z += vox * 2f)
        {
            for (float y = bbox.vecMin.Y; y <= bbox.vecMax.Y; y += vox * 2f)
            {
                for (float x = bbox.vecMin.X; x <= bbox.vecMax.X; x += vox * 2f)
                {
                    Vector3 pos = new(x, y, z);
                    if (!engineSdf.bGetValue(pos, out float sdfVal)) continue;

                    // Surface = SDF near zero
                    if (MathF.Abs(sdfVal) > 1.5f) continue;

                    // Gradient of SDF = OUTWARD surface normal (positive=outside,
                    // negative=inside solid ⇒ ∇ points solid→air). Central differences.
                    float h = vox * 0.5f;
                    engineSdf.bGetValue(new(x + h, y, z), out float sx1);
                    engineSdf.bGetValue(new(x - h, y, z), out float sx0);
                    engineSdf.bGetValue(new(x, y + h, z), out float sy1);
                    engineSdf.bGetValue(new(x, y - h, z), out float sy0);
                    engineSdf.bGetValue(new(x, y, z + h), out float sz1);
                    engineSdf.bGetValue(new(x, y, z - h), out float sz0);

                    Vector3 grad = new(sx1 - sx0, sy1 - sy0, sz1 - sz0);
                    if (grad.LengthSquared() < 1e-10f) continue;
                    grad = Vector3.Normalize(grad);

                    // Only downward-facing surfaces can overhang during a +Z-up build.
                    if (grad.Z >= 0f) continue;

                    measured++;

                    // Inclination of the surface from HORIZONTAL (the build plate):
                    //   β = acos(|n.Z|).  β = 0° → flat ceiling (worst), β = 90° → vertical (best).
                    float betaFromHoriz = MathF.Acos(MathF.Abs(grad.Z)) * 180f / MathF.PI;
                    if (betaFromHoriz < maxAngleFromHoriz)
                        maxAngleFromHoriz = betaFromHoriz;   // shallowest = worst overhang seen

                    if (betaFromHoriz >= limitFromHoriz) continue;  // self-supporting → not a violation
                    violations++;

                    // Classify the violation: inherent-but-expected (a known feature band
                    // of a tip-down aerospike) vs truly-problematic (everything else).
                    float r = MathF.Sqrt(pos.X * pos.X + pos.Y * pos.Y);
                    bool nearCowlLip   = MathF.Abs(z - cowlLipZ) <= band && r > 0.5f * rShroudMax;
                    bool nearFlange    = z >= flangeZ - band && r > 0.5f * rShroudMax;
                    bool isExpected    = nearCowlLip || nearFlange;

                    if (isExpected)
                    {
                        expected++;
                    }
                    else
                    {
                        problematic++;
                        if (betaFromHoriz < steepestProblematic)
                            steepestProblematic = betaFromHoriz;
                    }
                }
            }
        }

        OverhangViolations = violations;
        // Keep the public OverhangMaxAngle meaning "worst overhang severity": report it
        // as how far the shallowest downward face is BELOW the limit (0 = none worse than
        // limit). This stays a single comparable scalar for any existing consumer.
        OverhangMaxAngle = (measured > 0)
            ? MathF.Max(0f, limitFromHoriz - maxAngleFromHoriz)
            : 0f;
        OverhangExpected = expected;
        OverhangProblematic = problematic;
        OverhangSteepestProblematicAngle = (problematic > 0) ? steepestProblematic : 90f;

        // ── Honest summary ───────────────────────────────────────────────
        Library.Log($"  Build orientation: spike-tip-down (+Z up); self-support limit "
                  + $"{limitFromHoriz:F0}° from horizontal (maxOverhang {S.maxOverhang:F0}°)");
        Library.Log($"  Checked {measured} downward-facing surface points");
        if (measured > 0)
            Library.Log($"  Shallowest downward face: {maxAngleFromHoriz:F1}° from horizontal "
                      + $"(≥ {limitFromHoriz:F0}° = self-supporting)");
        if (violations == 0)
        {
            Library.Log("  All downward-facing surfaces are self-supporting at this orientation");
        }
        else
        {
            Library.Log($"  {violations} overhang points below the self-support limit, of which:");
            Library.Log($"    • {expected} INHERENT/EXPECTED — nozzle-exit cowl lip + flange "
                      + "underside; an aerospike printed tip-down has these by design "
                      + "(local/minor support, not a geometry defect)");
            Library.Log($"    • {problematic} PROBLEMATIC — shallow downward faces elsewhere "
                      + $"(e.g. internal channel roofs, mid-body); steepest {steepestProblematic:F1}° "
                      + "from horizontal");
            if (problematic == 0)
                Library.Log("  → No problematic overhangs: all violations are inherent to the "
                          + "spike-tip-down orientation. NOT a regression, NOT a geometry defect.");
            else
                Library.Log("  → Problematic overhangs are the only actionable ones. Do NOT distort "
                          + "engine geometry to chase the metric (physics-first); revisit support "
                          + "strategy or local feature angles instead.");
        }
    }

    private class Visitor : ITraverseScalarField
    {
        readonly AeroSpec _S;
        readonly RevolutionSDF _shroud, _spike;
        readonly IImplicit? _ch;
        readonly ScalarField _temp, _stress, _hf;
        readonly VectorField _cool;
        readonly float _Tgas, _Tcool, _twist;
        int _count;

        public Visitor(
            AeroSpec S, RevolutionSDF shroud, RevolutionSDF spike, IImplicit? ch,
            ScalarField temp, ScalarField stress, ScalarField hf, VectorField cool,
            float Tgas, float Tcool, float twist, ref int count)
        {
            _S = S; _shroud = shroud; _spike = spike; _ch = ch;
            _temp = temp; _stress = stress; _hf = hf; _cool = cool;
            _Tgas = Tgas; _Tcool = Tcool; _twist = twist;
            _count = 0;
        }

        public void InformActiveValue(in Vector3 pos, float sdfValue)
        {
            if (sdfValue > 0.5f) return; // outside solid

            // How deep in the wall? Distance from gas surface.
            float dShroud = _shroud.fSignedDistance(pos);
            float dSpike = _spike.fSignedDistance(pos);
            float dGas = MathF.Max(dShroud, -dSpike);

            float z = pos.Z;
            float q = HeatTransfer.HeatFlux(_S, z);
            float wallT = HeatTransfer.WallThickness(_S, z);

            // Depth fraction: 0 = gas surface, 1 = channel side
            float depth = 0f;
            if (dGas > 0f && wallT > 0.01f)
                depth = MathF.Min(dGas / wallT, 1f);

            // Temperature: hot at gas side, cool at channel side
            _temp.SetValue(pos, _Tgas - (_Tgas - _Tcool) * depth);

            // Heat flux: Bartz, attenuated by depth
            _hf.SetValue(pos, q * MathF.Max(0f, 1f - depth) / 1e6f);

            // Stress: hoop + thermal (MPa)
            float r = MathF.Sqrt(pos.X * pos.X + pos.Y * pos.Y);
            float sigH = _S.Pc * (r / 1000f) / MathF.Max(wallT / 1000f, 1e-6f);
            float dT = q * (wallT / 1000f) / _S.k_wall * depth;
            float sigT = _S.E_mod * _S.alpha_CTE * dT / (1f - _S.nu_poisson);
            _stress.SetValue(pos, (sigH + sigT) / 1e6f);

            // Coolant flow: helical direction near channels.
            // The channel winds azimuthally at rate _twist (rad of φ per mm of z,
            // = channelTwistTurns·2π / zTotal). The local helix tangent therefore
            // has tangential speed (r·_twist) per unit axial advance. Splitting the
            // (tangential, axial) components by the local pitch makes the swirl
            // genuinely follow the helix and vary with radius and z, instead of the
            // old z-independent constant sin/cos(_twist·50).
            float dCh = _ch?.fSignedDistance(pos) ?? 10f;
            if (dCh < 2f)
            {
                float phi = MathF.Atan2(pos.Y, pos.X);
                float vT = r * _twist;   // tangential component ∝ radius × azimuthal rate
                float vA = 1f;           // axial advance (per unit z)
                _cool.SetValue(pos, Vector3.Normalize(new(
                    -vT * MathF.Sin(phi),
                     vT * MathF.Cos(phi),
                     vA)));
            }

            _count++;
        }
    }
}
