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
    /// For each point on the channel surface, cast outward and find the
    /// nearest outer surface of the solid engine. The gap = actual wall.
    /// Compare with formula prediction. Flag violations.
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

                    // Engine SDF at this point = distance to nearest solid surface
                    // Since this point is inside a channel void (inside the engine),
                    // the SDF gives distance to the nearest wall.
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
    /// Measure overhang angles on the final engine surface.
    /// Extract surface normals from SDF gradient, check against maxOverhang.
    /// Print axis is Z (vertical). Overhang = angle between surface normal and Z.
    /// If normal points mostly sideways (angle > maxOverhang from vertical) — violation.
    /// </summary>
    public void MeasureOverhang(AeroSpec S, Voxels voxEngine)
    {
        Library.Log("Measuring overhang angles on final surface...");

        var engineSdf = new ScalarField(voxEngine);
        int violations = 0;
        int measured = 0;
        float maxAngle = 0f;
        float vox = S.voxelSize;

        voxEngine.CalculateProperties(out _, out BBox3 bbox);

        // Sample surface points: where SDF is near zero (within 1 voxel of surface)
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

                    // Gradient of SDF = surface normal direction
                    // Central differences
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

                    // Overhang angle: angle between normal and print axis (Z)
                    // If normal.Z > 0 = upward-facing = OK
                    // If normal.Z < 0 = downward-facing = check overhang
                    // Overhang angle from vertical = acos(|normal.Z|)
                    float angleFromVertical = MathF.Acos(MathF.Abs(grad.Z)) * 180f / MathF.PI;

                    // Only check downward-facing surfaces
                    if (grad.Z < 0f)
                    {
                        measured++;
                        if (angleFromVertical > maxAngle)
                            maxAngle = angleFromVertical;
                        if (angleFromVertical > S.maxOverhang)
                            violations++;
                    }
                }
            }
        }

        OverhangViolations = violations;
        OverhangMaxAngle = maxAngle;

        Library.Log($"  Checked {measured} downward-facing surface points");
        Library.Log($"  Max overhang: {maxAngle:F1} deg (limit {S.maxOverhang:F0})");
        if (violations > 0)
            Library.Log($"  WARNING: {violations} points exceed max overhang angle!");
        else
            Library.Log($"  All surfaces within overhang limit");
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

            // Coolant flow: helical direction near channels
            float dCh = _ch?.fSignedDistance(pos) ?? 10f;
            if (dCh < 2f)
            {
                float phi = MathF.Atan2(pos.Y, pos.X);
                float vT = MathF.Sin(_twist * 50f);
                float vA = MathF.Cos(_twist * 50f);
                _cool.SetValue(pos, Vector3.Normalize(new(
                    -vT * MathF.Sin(phi),
                     vT * MathF.Cos(phi),
                     vA)));
            }

            _count++;
        }
    }
}
