// FluidFirst.cs — SDF Composition Architecture (v6)
//
// Chamber + channels + variable wall = ONE IImplicit (EngineBodyImplicit).
// No voxel booleans for the core body. No OverOffset. No information loss.
// Manifolds/ports/flange still added as Lattice → Voxels (simple shapes).
//
// Insight: LEAP 71 video analysis (2026-04-01) confirmed that wall and
// channels appear SIMULTANEOUSLY as one evaluated SDF field.

using System.Diagnostics;
using System.Numerics;
using PicoGK;
using Leap71.ShapeKernel;

namespace OpenSpaceArch.Engine;

public static class FluidFirst
{
    /// <summary>
    /// Staged build — emits a StageResult after every major pipeline step.
    /// The cinematic viewer uses this to reveal components one by one.
    /// </summary>
    /// <param name="buildMode">
    /// Atomic: each stage = single voxelization (fast).
    /// ZSliceSlabs: CoreBody split into N horizontal slabs revealed slab-by-slab.
    /// </param>
    /// <param name="zSliceCount">Number of slabs for ZSliceSlabs mode (16-64 typical).</param>
    public static Voxels Build(AeroSpec S, Action<StageResult> onStage,
                               BuildMode buildMode = BuildMode.Atomic,
                               int zSliceCount = 24)
    {
        var sw = Stopwatch.StartNew();
        float LapSec() { var s = (float)sw.Elapsed.TotalSeconds; sw.Restart(); return s; }

        Library.Log("[staged] SDF Composition with per-stage reveal callbacks");

        // STEP 1: Analytical SDFs (shroud + spike voxelized together as preview)
        var shroudSDF = new RevolutionSDF(
            z => ChamberSizing.ShroudProfile(S, z), S.zCowl, S.zInjector);
        var spikeSDF = new RevolutionSDF(
            z => ChamberSizing.SpikeProfile(S, z), S.zTip, S.zInjector);

        {
            float bboxR = S.rShroudChamber + 15f;
            BBox3 bb = new(new(-bboxR, -bboxR, S.zTip - 5f), new(bboxR, bboxR, S.zInjector + 5f));
            Voxels voxShroudOnly = new Voxels(shroudSDF, bb);
            Voxels voxSpikeOnly = new Voxels(spikeSDF, bb);
            Voxels voxPreview = voxShroudOnly + voxSpikeOnly;
            onStage(new StageResult(StageId.AnalyticalSdfs, new Mesh(voxPreview),
                "Analytical SDFs (shroud + spike)", LapSec()));
        }

        // STEP 2: Channel SDFs
        IImplicit? chShroud = null;
        IImplicit? chSpike = null;

        if (S.channelMode == ChannelMode.Routed_v5b)
        {
            float bboxR = S.rShroudChamber + 15f;
            BBox3 routeBBox = new(
                new(-bboxR, -bboxR, S.zTip - 5f),
                new(bboxR, bboxR, S.zInjector + 5f));
            Voxels voxShroudRoute = new Voxels(shroudSDF, routeBBox);

            var routeS = ChannelRouter.RouteShroudChannels(S, voxShroudRoute);
            chShroud = new RoutedChannelFieldImplicit(S, routeS, isShroud: true);

            var routeP = ChannelRouter.RouteSpikeChannels(S, voxShroudRoute);
            chSpike = routeP.Spines.Count > 0
                ? new RoutedChannelFieldImplicit(S, routeP, isShroud: false)
                : new ChannelFieldImplicit(S, isShroud: false);

            voxShroudRoute = null!;
            GC.Collect();
        }
        else
        {
            chShroud = new ChannelFieldImplicit(S, isShroud: true);
            chSpike = new ChannelFieldImplicit(S, isShroud: false);
        }

        // Channel preview: voxelize channel fields alone (narrow bbox around wall band)
        {
            float bboxR = S.rShroudChamber + 15f;
            BBox3 bb = new(new(-bboxR, -bboxR, S.zTip - 5f), new(bboxR, bboxR, S.zInjector + 5f));
            Voxels voxChanPreview = new Voxels(chShroud, bb);
            if (chSpike != null)
                voxChanPreview += new Voxels(chSpike, bb);
            onStage(new StageResult(StageId.ChannelSdfs, new Mesh(voxChanPreview),
                "Cooling channel SDFs", LapSec()));
        }

        // STEP 3: Core body
        var engineBody = new EngineBodyImplicit(S, shroudSDF, spikeSDF, chShroud, chSpike);
        BBox3 bodyBBox = engineBody.GetBBox();

        Voxels voxSolid;
        if (buildMode == BuildMode.ZSliceSlabs)
        {
            // Z-slice mode: chop bbox along Z, voxelize each slab separately, emit each as a stage.
            // Slabs overlap by 1 voxel to avoid surface seams during marching cubes.
            voxSolid = new Voxels();
            float zMin = bodyBBox.vecMin.Z;
            float zMax = bodyBBox.vecMax.Z;
            float slabH = (zMax - zMin) / zSliceCount;
            float overlap = MathF.Max(S.voxelSize * 2f, 0.5f);

            for (int i = 0; i < zSliceCount; i++)
            {
                float z0 = zMin + i * slabH - (i > 0 ? overlap : 0f);
                float z1 = zMin + (i + 1) * slabH + (i < zSliceCount - 1 ? overlap : 0f);
                BBox3 slabBBox = new(
                    new Vector3(bodyBBox.vecMin.X, bodyBBox.vecMin.Y, z0),
                    new Vector3(bodyBBox.vecMax.X, bodyBBox.vecMax.Y, z1));

                Voxels voxSlab = new Voxels(engineBody, slabBBox);
                voxSolid.BoolAdd(voxSlab);

                // Emit slab as its own stage so viewer reveals it incrementally
                onStage(new StageResult(StageId.CoreBodySlab, new Mesh(voxSlab),
                    $"Core slab {i + 1}/{zSliceCount} (z={z0:F0}..{z1:F0})", LapSec()));
            }
        }
        else
        {
            voxSolid = new Voxels(engineBody, bodyBBox);
            onStage(new StageResult(StageId.CoreBody, new Mesh(voxSolid),
                "Core body (SDF composition)", LapSec()));
        }

        // STEP 4: Lattice components (each reported separately)
        Voxels v1 = BuildAxialManifold(S); voxSolid += v1;
        onStage(new StageResult(StageId.AxialManifold, new Mesh(v1), "Axial LOX manifold", LapSec()));

        Voxels v2 = BuildShroudCollector(S); voxSolid += v2;
        onStage(new StageResult(StageId.ShroudCollector, new Mesh(v2), "Shroud collector ring", LapSec()));

        Voxels v3 = BuildShroudInlet(S); voxSolid += v3;
        onStage(new StageResult(StageId.ShroudInlet, new Mesh(v3), "Shroud inlet ring", LapSec()));

        Voxels v4 = BuildFeedPorts(S); voxSolid += v4;
        onStage(new StageResult(StageId.FeedPorts, new Mesh(v4), "Fuel/LOX/igniter ports", LapSec()));

        Voxels v5 = BuildSpikeVanes(S); voxSolid += v5;
        onStage(new StageResult(StageId.SpikeVanes, new Mesh(v5), "Spike structural vanes", LapSec()));

        Voxels v6 = BuildTopFlange(S); voxSolid += v6;
        onStage(new StageResult(StageId.TopFlange, new Mesh(v6), "Top mounting flange", LapSec()));

        // STEP 5: Smoothen
        voxSolid.Smoothen(0.2f);
        onStage(new StageResult(StageId.Smoothen, new Mesh(voxSolid), "Light smoothing pass", LapSec()));

        // STEP 6: Post bores
        Voxels bores = BuildPostSmoothingBores(S);
        voxSolid -= bores;
        onStage(new StageResult(StageId.PostBores, new Mesh(bores), "Post-smoothing bores", LapSec()));

        // Final combined
        onStage(new StageResult(StageId.Final, new Mesh(voxSolid), "Final engine", LapSec()));

        return voxSolid;
    }

    public static Voxels Build(AeroSpec S)
    {
        Library.Log("╔══════════════════════════════════════════╗");
        Library.Log("║  SDF COMPOSITION: ONE field → voxels     ║");
        Library.Log("║  Chamber + channels + wall = 1 equation  ║");
        Library.Log("╚══════════════════════════════════════════╝");

        // ══════════════════════════════════════
        // STEP 1: ANALYTICAL SDFs (no voxels yet)
        // ══════════════════════════════════════
        Library.Log("Step 1: Analytical SDFs...");

        var shroudSDF = new RevolutionSDF(
            z => ChamberSizing.ShroudProfile(S, z), S.zCowl, S.zInjector);
        var spikeSDF = new RevolutionSDF(
            z => ChamberSizing.SpikeProfile(S, z), S.zTip, S.zInjector);
        Library.Log("  Shroud + Spike SDFs defined.");

        // ══════════════════════════════════════
        // STEP 2: CHANNEL SDFs (IImplicit, no voxels)
        // ══════════════════════════════════════
        Library.Log("Step 2: Channel SDFs...");

        IImplicit? chShroud = null;
        IImplicit? chSpike = null;

        if (S.channelMode == ChannelMode.Routed_v5b)
        {
            // Turtle walk needs voxels for bClosestPointOnSurface — temporary only
            Library.Log("  Routing: creating temp surface for turtle walk...");
            float bboxR = S.rShroudChamber + 15f;
            BBox3 routeBBox = new(
                new(-bboxR, -bboxR, S.zTip - 5f),
                new( bboxR,  bboxR, S.zInjector + 5f));
            Voxels voxShroudRoute = new Voxels(shroudSDF, routeBBox);

            var routeS = ChannelRouter.RouteShroudChannels(S, voxShroudRoute);
            chShroud = new RoutedChannelFieldImplicit(S, routeS, isShroud: true);
            Library.Log($"  Shroud: {routeS.Spines.Count} routed channels.");

            var routeP = ChannelRouter.RouteSpikeChannels(S, voxShroudRoute);
            if (routeP.Spines.Count > 0)
                chSpike = new RoutedChannelFieldImplicit(S, routeP, isShroud: false);
            else
                chSpike = new ChannelFieldImplicit(S, isShroud: false);
            Library.Log($"  Spike: {routeP.Spines.Count} routed channels (fallback: implicit).");

            voxShroudRoute = null!;
            GC.Collect();
        }
        else
        {
            chShroud = new ChannelFieldImplicit(S, isShroud: true);
            chSpike = new ChannelFieldImplicit(S, isShroud: false);
            Library.Log("  Channels: ChannelFieldImplicit (v5 helical).");
        }

        // ══════════════════════════════════════
        // STEP 3: ONE IMPLICIT → ONE VOXELIZATION
        // ══════════════════════════════════════
        Library.Log("Step 3: EngineBodyImplicit → voxels (ONE evaluation)...");
        var engineBody = new EngineBodyImplicit(S, shroudSDF, spikeSDF, chShroud, chSpike);
        Voxels voxSolid = new Voxels(engineBody, engineBody.GetBBox());
        Library.Log("  Core body done.");

        // ══════════════════════════════════════
        // STEP 4: ADD LATTICE COMPONENTS
        // (simple shapes, voxel boolean is fine)
        // ══════════════════════════════════════
        Library.Log("Step 4: Lattice additions...");

        voxSolid += BuildAxialManifold(S);
        Library.Log("  + Axial manifold");

        voxSolid += BuildShroudCollector(S);
        Library.Log("  + Shroud collector");

        voxSolid += BuildShroudInlet(S);
        Library.Log("  + Shroud inlet");

        voxSolid += BuildFeedPorts(S);
        Library.Log("  + Feed ports");

        voxSolid += BuildSpikeVanes(S);
        Library.Log("  + Spike vanes");

        voxSolid += BuildTopFlange(S);
        Library.Log("  + Top flange");

        // ══════════════════════════════════════
        // STEP 5: LIGHT POLISH (no OverOffset!)
        // ══════════════════════════════════════
        Library.Log("Step 5: Light smoothen...");
        voxSolid.Smoothen(0.2f);

        // ══════════════════════════════════════
        // STEP 6: POST-SMOOTHING BORES
        // ══════════════════════════════════════
        Library.Log("Step 6: Post-smoothing bores...");
        voxSolid -= BuildPostSmoothingBores(S);
        Library.Log("  Bores drilled.");

        Library.Log("Done.");
        return voxSolid;
    }

    // ─────────────────────────────────────
    // AXIAL LOX MANIFOLD: center of spike
    // ─────────────────────────────────────
    static Voxels BuildAxialManifold(AeroSpec S)
    {
        Lattice lat = new Lattice();
        int nSteps = 30;
        float zStart = S.zTip + 3f;
        float zEnd = S.zChTop;

        for (int i = 1; i <= nSteps; i++)
        {
            float t0 = (float)(i - 1) / nSteps;
            float t1 = (float)i / nSteps;
            float z0 = zStart + t0 * (zEnd - zStart);
            float z1 = zStart + t1 * (zEnd - zStart);

            float rSp0 = ChamberSizing.SpikeProfile(S, z0);
            float rSp1 = ChamberSizing.SpikeProfile(S, z1);
            float r0 = MathF.Max(MathF.Min(1f + t0 * 3f, rSp0 - 3f), 0.5f);
            float r1 = MathF.Max(MathF.Min(1f + t1 * 3f, rSp1 - 3f), 0.5f);

            lat.AddBeam(new Vector3(0, 0, z0), new Vector3(0, 0, z1), r0, r1);
        }

        return new Voxels(lat);
    }

    // ─────────────────────────────────────
    // SHROUD COLLECTOR: toroidal manifold at top
    // Hot CH4 collects here from all shroud channels before entering injector
    // ─────────────────────────────────────
    static Voxels BuildShroudCollector(AeroSpec S)
    {
        float z = S.zChTop;
        float wall = HeatTransfer.WallThickness(S, z);
        var (cw, ch) = HeatTransfer.ChannelRect(S, z);
        float rRing = ChamberSizing.ShroudProfile(S, z) + wall + ch / 2f;

        // Ring of points for Frames
        int nSegs = 60;
        List<Vector3> ringPoints = new();
        for (int i = 0; i <= nSegs; i++)
        {
            float phi = i * 2f * MathF.PI / nSegs;
            ringPoints.Add(new Vector3(rRing * MathF.Cos(phi), rRing * MathF.Sin(phi), z));
        }

        Frames frames = new Frames(ringPoints, Vector3.UnitZ);
        var manifold = new LatticeManifold(frames, S.manifoldRadius,
            fMaxOverhangAngle: S.maxOverhang, bExtendBothSides: false);
        return manifold.voxConstruct();
    }

    // ─────────────────────────────────────
    // SHROUD INLET: toroidal manifold at bottom
    // Cold CH4 enters here, distributes to all shroud channels
    // ─────────────────────────────────────
    static Voxels BuildShroudInlet(AeroSpec S)
    {
        float z = S.zCowl + 3f;
        float wall = HeatTransfer.WallThickness(S, z);
        var (cw, ch) = HeatTransfer.ChannelRect(S, z);
        float rSh = ChamberSizing.ShroudProfile(S, z);
        if (rSh < 2f) rSh = S.rShroudThroat;
        float rRing = rSh + wall + ch / 2f;

        int nSegs = 60;
        List<Vector3> ringPoints = new();
        for (int i = 0; i <= nSegs; i++)
        {
            float phi = i * 2f * MathF.PI / nSegs;
            ringPoints.Add(new Vector3(rRing * MathF.Cos(phi), rRing * MathF.Sin(phi), z));
        }

        Frames frames = new Frames(ringPoints, Vector3.UnitZ);
        var manifold = new LatticeManifold(frames, S.manifoldRadius,
            fMaxOverhangAngle: S.maxOverhang, bExtendBothSides: false);
        return manifold.voxConstruct();
    }

    // ─────────────────────────────────────
    // FEED PORTS: conical transitions breaking axial symmetry
    // Fuel (0°), LOX (180°), Igniter (90°, angled)
    // ─────────────────────────────────────
    static Voxels BuildFeedPorts(AeroSpec S)
    {
        Lattice lat = new Lattice();
        float rPort = S.feedPortRadius;
        float portLen = S.feedPortLength;

        // Fuel port — slight taper
        float zFuel = S.zCowl + 3f;
        float rSh = ChamberSizing.ShroudProfile(S, zFuel);
        if (rSh < 2f) rSh = S.rShroudThroat;
        float wallF = HeatTransfer.WallThickness(S, zFuel);
        float rStart = rSh + wallF;
        lat.AddBeam(
            new Vector3(rStart, 0, zFuel), rPort * 1.15f,
            new Vector3(rStart + portLen, 0, zFuel), rPort);

        // LOX port — opposite side, near injector
        float zLox = S.zInjector - 5f;
        float rShTop = ChamberSizing.ShroudProfile(S, zLox);
        if (rShTop < 2f) rShTop = S.rShroudChamber;
        float wallL = HeatTransfer.WallThickness(S, zLox);
        float rStartL = rShTop + wallL;
        lat.AddBeam(
            new Vector3(-rStartL, 0, zLox), rPort * 1.15f,
            new Vector3(-rStartL - portLen, 0, zLox), rPort);

        // Igniter port — angled entry at 90°
        float zIgn = S.zChBot + (S.zChTop - S.zChBot) * 0.3f;
        float rShIgn = ChamberSizing.ShroudProfile(S, zIgn);
        if (rShIgn < 2f) rShIgn = S.rShroudChamber;
        float wallI = HeatTransfer.WallThickness(S, zIgn);
        float rIgnStart = rShIgn + wallI;
        float ignLen = 6f;
        lat.AddBeam(
            new Vector3(0, rIgnStart, zIgn), 1.2f,
            new Vector3(0, rIgnStart + ignLen * 0.87f, zIgn + ignLen * 0.5f), 1.0f);

        return new Voxels(lat);
    }

    // ─────────────────────────────────────
    // SPIKE VANES: radial cooling channels that become structural bridges
    // Spike MUST connect to shroud — these vanes are the connection
    // Fluid-first: channels through vanes → offset → solid structural bridges
    // ─────────────────────────────────────
    static Voxels BuildSpikeVanes(AeroSpec S)
    {
        Lattice lat = new Lattice();
        int nVanes = 4;
        float vaneR = 0.8f;  // cooling channel radius inside each vane

        // Vanes span from spike to shroud at convergent section
        // Multiple z-stations for each vane (not just one point)
        float zStart = S.zThroat - 2f;
        float zEnd = S.zThroat + 8f;
        int nSteps = 5;

        for (int v = 0; v < nVanes; v++)
        {
            float phi = v * 2f * MathF.PI / nVanes + MathF.PI / 8f; // offset from feed ports

            for (int step = 0; step < nSteps; step++)
            {
                float t0 = (float)step / nSteps;
                float t1 = (float)(step + 1) / nSteps;
                float z0 = zStart + t0 * (zEnd - zStart);
                float z1 = zStart + t1 * (zEnd - zStart);

                float rSp0 = ChamberSizing.SpikeProfile(S, z0) + 0.5f;
                float rSh0 = ChamberSizing.ShroudProfile(S, z0) - 0.5f;
                float rSp1 = ChamberSizing.SpikeProfile(S, z1) + 0.5f;
                float rSh1 = ChamberSizing.ShroudProfile(S, z1) - 0.5f;
                if (rSh0 <= rSp0 || rSh1 <= rSp1) continue;

                // Inner segment (spike side)
                float rMid0 = (rSp0 + rSh0) / 2f;
                float rMid1 = (rSp1 + rSh1) / 2f;
                lat.AddBeam(
                    new Vector3(rSp0 * MathF.Cos(phi), rSp0 * MathF.Sin(phi), z0), vaneR,
                    new Vector3(rMid0 * MathF.Cos(phi), rMid0 * MathF.Sin(phi), z0), vaneR);
                lat.AddBeam(
                    new Vector3(rMid0 * MathF.Cos(phi), rMid0 * MathF.Sin(phi), z0), vaneR,
                    new Vector3(rSh0 * MathF.Cos(phi), rSh0 * MathF.Sin(phi), z0), vaneR);

                // Axial connection between z-stations (along the vane length)
                lat.AddBeam(
                    new Vector3(rMid0 * MathF.Cos(phi), rMid0 * MathF.Sin(phi), z0), vaneR,
                    new Vector3(rMid1 * MathF.Cos(phi), rMid1 * MathF.Sin(phi), z1), vaneR);
            }
        }

        return new Voxels(lat);
    }

    // ─────────────────────────────────────
    // TOP FLANGE: solid mounting plate above injector
    // Engine mounts from top (feed lines + thrust structure)
    // Bolt holes drilled in Step 7 (post-smoothing)
    // ─────────────────────────────────────
    static Voxels BuildTopFlange(AeroSpec S)
    {
        Lattice lat = new Lattice();
        float zBot = S.zInjector;
        float zTop = S.zInjector + 8f;
        float rOuter = S.rShroudChamber + S.mountFlangeExtent;

        // Solid disk: dense concentric rings with overlapping radii → merges into plate
        int nSegs = 60;
        float ringSpacing = 3f;
        float beamR = ringSpacing;  // overlap = solid fill
        for (float z = zBot + beamR; z <= zTop - beamR + 0.1f; z += ringSpacing)
        {
            for (float r = ringSpacing; r <= rOuter; r += ringSpacing)
            {
                for (int i = 0; i < nSegs; i++)
                {
                    float phi0 = i * 2f * MathF.PI / nSegs;
                    float phi1 = (i + 1) * 2f * MathF.PI / nSegs;
                    lat.AddBeam(
                        new Vector3(r * MathF.Cos(phi0), r * MathF.Sin(phi0), z), beamR,
                        new Vector3(r * MathF.Cos(phi1), r * MathF.Sin(phi1), z), beamR);
                }
            }
        }

        return new Voxels(lat);
    }

    // ─────────────────────────────────────
    // POST-SMOOTHING BORES: fine holes drilled after OverOffset+Smoothen
    // Per fluid_first_pattern.md Step 7: small bores would fill during smoothing
    // ─────────────────────────────────────
    static Voxels BuildPostSmoothingBores(AeroSpec S)
    {
        Lattice lat = new Lattice();
        float zTop = S.zInjector;

        // Fuel injection holes (ring around LOX tube)
        float rHoleRing = 5f;
        float rHole = 1.2f;
        for (int i = 0; i < S.nInjectorHoles; i++)
        {
            float phi = i * 2f * MathF.PI / S.nInjectorHoles;
            float x = rHoleRing * MathF.Cos(phi);
            float y = rHoleRing * MathF.Sin(phi);
            lat.AddBeam(
                new Vector3(x, y, zTop - 6f), rHole,
                new Vector3(x, y, zTop + 1f), rHole);
        }

        // Film cooling holes (smaller, near chamber wall)
        float rFilm = S.rShroudChamber * 0.85f;
        int nFilm = 16;
        for (int i = 0; i < nFilm; i++)
        {
            float phi = i * 2f * MathF.PI / nFilm;
            float x = rFilm * MathF.Cos(phi);
            float y = rFilm * MathF.Sin(phi);
            lat.AddBeam(
                new Vector3(x, y, S.zChTop), 0.8f,
                new Vector3(x, y, S.zChTop + 3f), 0.8f);
        }

        // Bolt holes through top flange (8 holes in circle)
        int nBolts = 8;
        float rBoltCircle = S.rShroudChamber + S.mountFlangeExtent * 0.6f;
        float rBolt = 2.0f;
        for (int i = 0; i < nBolts; i++)
        {
            float phi = i * 2f * MathF.PI / nBolts;
            float x = rBoltCircle * MathF.Cos(phi);
            float y = rBoltCircle * MathF.Sin(phi);
            lat.AddBeam(
                new Vector3(x, y, S.zInjector - 2f), rBolt,
                new Vector3(x, y, S.zInjector + 10f), rBolt);
        }

        return new Voxels(lat);
    }

}
