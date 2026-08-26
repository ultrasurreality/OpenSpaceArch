// OpenSpaceArch v4 — Field-Driven Aerospike Engine
// Philosophy: "Geometry EMERGES from physics fields"
//
// Physics → ScalarFields → IImplicit → Voxels → OverOffset → Organic
// No explicit channel placement. Cooling pattern emerges from heat flux.

using System.Numerics;
using System.Text.Json;
using PicoGK;
using Leap71.ShapeKernel;
using OpenSpaceArch.Engine;
using Engine = OpenSpaceArch.Engine;

namespace OpenSpaceArch;

class Program
{
    static void Main(string[] args)
    {
        // Headless modes available via CLI args (no UI):
        //   --physics      → run only physics, print numbers, exit
        //   --headless     → full build + STL export, no window
        //   --sweep        → design space sweep
        //   --single       → single variant validation
        // Anything else (or no args) → launch the cinematic viewer with ImGui controls.

        if (args.Length > 0)
        {
            switch (args[0].ToLowerInvariant())
            {
                case "--physics": PhysicsOnly(); return;
                // Default comes from AeroSpec.voxelSize (0.3 mm since 2026-08-12, was 0.4) so the
                // batch-STL resolution lives in exactly one place. Override per run:
                // `--headless 0.2`. The interactive viewer (AppMain) runs a coarser 0.5 mm core
                // grid for responsiveness — intentionally different, see AppMain/ControlPanel.
                case "--headless": BuildEngineHeadless(ParseVoxelArg(args, new AeroSpec().voxelSize)); return;
                case "--sweep": DesignSweep.Run(); return;
                case "--single":
                    Console.WriteLine("\n── Single Variant: Physics + Spatial ──\n");
                    DesignSweep.RunSingle(new AeroSpec());
                    return;
            }
        }

        // Default: cinematic viewer — all controls live inside the window
        OpenSpaceArch.Viewer.AppMain.Run();
    }

    /// <summary>
    /// Optional voxel-size override: `--headless 0.25`. Omitted → AeroSpec.voxelSize default (0.4 mm).
    /// Parsed with InvariantCulture on purpose: the machine locale uses a comma decimal separator,
    /// so "0.25" would otherwise silently parse as 25 and blow up the grid.
    /// </summary>
    static float ParseVoxelArg(string[] args, float fallback)
    {
        if (args.Length > 1 &&
            float.TryParse(args[1], System.Globalization.NumberStyles.Float,
                           System.Globalization.CultureInfo.InvariantCulture, out float mm) &&
            mm > 0.05f && mm <= 2.0f)
            return mm;
        return fallback;
    }

    static void PhysicsOnly()
    {
        Console.WriteLine("\n── Physics Only Mode ──\n");
        AeroSpec spec = new AeroSpec();
        Thermochemistry.Compute(spec);
        ChamberSizing.Compute(spec);
        HeatTransfer.Compute(spec);
        HeatTransfer.DerivePortPositions(spec);
        Console.WriteLine($"\nThrust: {spec.F_thrust/1000:F1} kN");
        Console.WriteLine($"Isp: {spec.Isp_SL:F1} s (SL), {spec.Isp_vac:F1} s (vac)");
        Console.WriteLine($"Mass flow: {spec.mDot:F3} kg/s");
        Console.WriteLine($"Total length: {spec.zTotal:F1} mm");
    }

    static void BuildEngineHeadless(float voxelSize)
    {
        // Init PicoGK Core headless (no Viewer)
        Library.InitHeadless(voxelSize);

        try
        {
            Library.Log("╔══════════════════════════════════════════╗");
            Library.Log("║  OpenSpaceArch v6 — Headless Build        ║");
            Library.Log("║  Physics → Fields → Voxels → STL          ║");
            Library.Log("╚══════════════════════════════════════════╝");

            // ── STEP 1-3: Pure physics (zero geometry) ──
            AeroSpec spec = new AeroSpec();
            spec.voxelSize = voxelSize;   // keep spec in sync with the actual PicoGK grid
            Thermochemistry.Compute(spec);
            ChamberSizing.Compute(spec);
            HeatTransfer.Compute(spec);
            HeatTransfer.DerivePortPositions(spec);
            spec.LogSummary();

            // ── STEP 4-8: Geometry emerges from physics ──
            Voxels voxEngine = EngineAssembly.Build(spec);

            // ── STEP 9: Measure reality vs formulas ──
            Library.Log("");
            Library.Log("── POST-BUILD MEASUREMENT (voxels, not formulas) ──");
            var shroudSDF = new RevolutionSDF(
                z => ChamberSizing.ShroudProfile(spec, z), spec.zCowl, spec.zInjector);
            var spikeSDF = new RevolutionSDF(
                z => ChamberSizing.SpikeProfile(spec, z), spec.zTip, spec.zInjector);
            var chShroud = new RoutedChannelFieldImplicit(spec,
                ChannelRouter.RouteShroudChannels(spec), isShroud: true);

            var fields = new PhysicsFields();
            fields.MeasureWallThickness(spec, voxEngine, chShroud);
            fields.MeasureOverhang(spec, voxEngine);

            // ── VERIFICATION ──
            Verify(voxEngine, spec);

            // ── EXPORT (STL + cutaway + spec JSON) ──
            Export(voxEngine, spec);
        }
        finally
        {
            Library.Shutdown();
        }
    }

    static void Verify(Voxels vox, AeroSpec S)
    {
        Library.Log("");
        Library.Log("── VERIFICATION ─────────────────────────");

        vox.CalculateProperties(out float volumeMM3, out BBox3 bbox);
        float massKg = volumeMM3 * 1e-9f * S.rho;
        float thrustCheck = S.Cf * S.Pc * S.At;

        Library.Log($"  Volume:    {volumeMM3:F0} mm³");
        Library.Log($"  Mass:      {massKg:F3} kg");
        Library.Log($"  BBox:      [{bbox.vecMin.X:F1}, {bbox.vecMin.Y:F1}, {bbox.vecMin.Z:F1}]");
        Library.Log($"             [{bbox.vecMax.X:F1}, {bbox.vecMax.Y:F1}, {bbox.vecMax.Z:F1}]");
        Library.Log($"  Thrust:    {thrustCheck:F0} N ({thrustCheck/1000:F1} kN)");
        Library.Log($"  T/W ratio: {thrustCheck / (massKg * 9.81f):F0}");
        Library.Log($"  Isp (SL):  {S.Isp_SL:F1} s");

        // Sanity checks
        if (massKg < 0.3f)
            Library.Log("  ⚠ WARNING: Mass suspiciously low!");
        if (massKg > 15f)
            Library.Log("  ⚠ WARNING: Mass suspiciously high!");
        if (thrustCheck < S.F_thrust * 0.8f || thrustCheck > S.F_thrust * 1.3f)
            Library.Log($"  ⚠ WARNING: Thrust {thrustCheck:F0}N deviates from target {S.F_thrust:F0}N");
    }

    static void Export(Voxels vox, AeroSpec S)
    {
        string desktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
        string stlPath = Path.Combine(desktop, "AerospikeV4.stl");
        string cutPath = Path.Combine(desktop, "AerospikeV4_Cutaway.stl");

        // Full engine
        Mesh msh = new Mesh(vox);
        msh.SaveToStlFile(stlPath);
        Library.Log($"  Exported: {stlPath}");

        // Cutaway (Y < 0)
        Lattice latCut = new Lattice();
        float rMax = 100f;
        latCut.AddBeam(
            new Vector3(0, -rMax, S.zTip - 5f), rMax,
            new Vector3(0, -rMax, S.zTotal + 5f), rMax);
        Voxels voxCutBlock = new Voxels(latCut);
        Voxels voxCut = vox - voxCutBlock;
        Mesh mshCut = new Mesh(voxCut);
        mshCut.SaveToStlFile(cutPath);
        Library.Log($"  Exported: {cutPath}");

        // JSON sidecar for analyze_stl.py
        string specPath = Path.Combine(desktop, "AerospikeV4_spec.json");
        var specData = new
        {
            z_stations = new Dictionary<string, float>
            {
                ["tip"] = S.zTip,
                ["cowl"] = S.zCowl,
                ["throat"] = S.zThroat,
                ["chBot"] = S.zChBot,
                ["chTop"] = S.zChTop,
                ["injector"] = S.zInjector,
                ["total"] = S.zTotal
            },
            radii = new Dictionary<string, float>
            {
                ["spikeTip"] = S.rSpikeTip,
                ["spikeThroat"] = S.rSpikeThroat,
                ["shroudThroat"] = S.rShroudThroat,
                ["spikeChamber"] = S.rSpikeChamber,
                ["shroudChamber"] = S.rShroudChamber
            },
            channels = new Dictionary<string, float>
            {
                ["nShroud"] = S.nChannelsShroud,
                ["nSpike"] = S.nChannelsSpike,
                ["radiusMin"] = S.chRadiusMin,
                ["radiusMax"] = S.chRadiusMax
            },
            wall = new Dictionary<string, float>
            {
                ["outer"] = 3.0f,
                ["minPrint"] = S.minPrintWall,
                ["throat"] = S.wallThroat
            },
            material = new Dictionary<string, object>
            {
                ["density"] = S.rho,
                ["name"] = "CuCrZr"
            },
            physics = new Dictionary<string, float>
            {
                ["F_thrust"] = S.F_thrust,
                ["Pc_bar"] = S.Pc / 1e5f,
                ["Isp_SL"] = S.Isp_SL,
                ["Isp_vac"] = S.Isp_vac,
                ["mDot"] = S.mDot
            }
        };
        var jsonOpts = new JsonSerializerOptions { WriteIndented = true };
        File.WriteAllText(specPath, JsonSerializer.Serialize(specData, jsonOpts));
        Library.Log($"  Spec JSON: {specPath}");
    }
}
