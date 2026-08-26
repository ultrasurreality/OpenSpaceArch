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

// FluidFirst.cs — Voids-first build via voxel composition (v7)
//
// THIS IS THE PRODUCTION BUILD PATH (EngineAssembly.Build → FluidFirst.Build).
//
// Honest description of what actually happens here:
//   - Cooling channels are evaluated as TRUE IImplicit fields
//     (RoutedChannelFieldImplicit: superellipse cross-section, print-angle exponent,
//     port widening, turbulator ribs) and rendered to Voxels per group.
//   - Gas path, spike body, manifolds, ports and flange are Lattice → Voxels.
//   - These are combined with VOXEL BOOLEANS (BoolAdd / BoolSubtract / Trim):
//       voids = channels ∪ gas-path
//       outer = voids grown by VariableWallImplicit (per-z Barlow wall)
//       solid = outer − voids + spike − LOX bore + lattice features
//
// So the build is fluid-first (voids drive the wall) but it is NOT a single
// monolithic SDF. The "one IImplicit for the whole core" idea lives in
// EngineBodyImplicit.cs, which is NOT wired into this path (see that file's header).
// Per-voxel pieces that ARE single SDFs: the channel field and the wall field.
//
// Design rationale: the wall and the channels are not independent objects. Wall
// thickness is a consequence of where the coolant has to go, so it is derived FROM
// the channel voids — here via the channel-void → variable-wall offset chain,
// rather than being specified alongside them.

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

        Library.Log("[staged] CHANNELS-FIRST: channels → gas path → offset → solid");

        // ══════════════════════════════════════
        // STEP 1: CHANNELS FIRST (the primary structure)
        // Physics determines sizing, routing places them in 3D.
        // ══════════════════════════════════════
        var routeS = ChannelRouter.RouteShroudChannels(S);
        Voxels voxChShroud = BuildChannelVoids(S, routeS, isShroud: true);

        var routeP = ChannelRouter.RouteSpikeChannels(S);
        Voxels voxChSpike = routeP.Spines.Count > 0
            ? BuildChannelVoids(S, routeP, isShroud: false)
            : new Voxels();

        Voxels voxChAll = new Voxels();
        voxChAll.BoolAdd(voxChShroud);
        voxChAll.BoolAdd(voxChSpike);
        onStage(new StageResult(StageId.Channels, new Mesh(voxChAll),
            $"Cooling channels ({routeS.Spines.Count}+{routeP.Spines.Count})", LapSec()));

        // ══════════════════════════════════════
        // STEP 2: GAS PATH — split into Nozzle / Chamber / Dome
        // (builds AFTER channels — adapts to them)
        // ══════════════════════════════════════
        Voxels voxNozzle = BuildNozzleVoid(S);
        onStage(new StageResult(StageId.Nozzle, new Mesh(voxNozzle),
            "Nozzle (convergent + throat)", LapSec()));

        Voxels voxChamber = BuildChamberVoid(S);
        onStage(new StageResult(StageId.Chamber, new Mesh(voxChamber),
            "Combustion chamber", LapSec()));

        Voxels voxDome = BuildDomeVoid(S);
        onStage(new StageResult(StageId.Dome, new Mesh(voxDome),
            "Injector dome", LapSec()));

        Voxels voxGas = voxNozzle;
        voxGas.BoolAdd(voxChamber);
        voxGas.BoolAdd(voxDome);

        // ══════════════════════════════════════
        // STEP 3: MUTUAL EXCLUSION (channels stay wallT from gas)
        // ══════════════════════════════════════
        float minWall = S.minPrintWall;
        voxChShroud.BoolSubtract(voxGas.voxOffset(minWall));
        if (routeP.Spines.Count > 0)
        {
            voxChSpike.BoolSubtract(voxGas.voxOffset(minWall));
            voxChShroud.BoolSubtract(voxChSpike.voxOffset(minWall));
            voxChSpike.BoolSubtract(voxChShroud.voxOffset(minWall));
        }

        // ══════════════════════════════════════
        // STEP 4: UNION ALL VOIDS
        // ══════════════════════════════════════
        Voxels voxInner = voxChShroud;  // channels first in union
        voxInner.BoolAdd(voxChSpike);
        voxInner.BoolAdd(voxGas);       // gas path joins channels
        onStage(new StageResult(StageId.AllVoids, new Mesh(voxInner),
            "All fluid voids", LapSec()));

        // STEP 5: Walls = VARIABLE (per-z Barlow) offset from voids
        Voxels voxOuter = BuildVariableWallShell(S, voxInner);
        // Representative wall (throat) — used only to position the nozzle-exit trim cap.
        float wallT = MathF.Max(HeatTransfer.WallThickness(S, S.zThroat), S.minPrintWall) + S.minPrintWall;

        // STEP 6: Fluid-first subtract + open nozzle exit + drill LOX bore
        Voxels voxSolid = voxOuter;
        voxSolid.BoolSubtract(voxInner);

        // Trim below cowl (remove offset cap at nozzle exit)
        voxSolid.Trim(new BBox3(
            new Vector3(-200f, -200f, S.zCowl - wallT),
            new Vector3(200f, 200f, S.zInjector + 20f)));

        // Add spike nozzle body (solid cone, aerospike contour)
        Voxels voxSpike = BuildSpikeBody(S);
        onStage(new StageResult(StageId.Spike, new Mesh(voxSpike),
            "Aerospike nozzle cone", LapSec()));
        voxSolid.BoolAdd(voxSpike);

        // Drill axial LOX manifold
        voxSolid.BoolSubtract(BuildAxialManifold(S));

        onStage(new StageResult(StageId.Shell, new Mesh(voxSolid),
            $"Engine shell (wallT={wallT:F2}mm)", LapSec()));

        // STEP 7: Lattice additions
        Voxels v2 = BuildShroudCollector(S); voxSolid += v2;
        onStage(new StageResult(StageId.Collector, new Mesh(v2), "CH4 collector", LapSec()));

        Voxels v3 = BuildShroudInlet(S); voxSolid += v3;
        onStage(new StageResult(StageId.Inlet, new Mesh(v3), "CH4 inlet", LapSec()));

        Voxels v4 = BuildFeedPorts(S); voxSolid += v4;
        onStage(new StageResult(StageId.FeedPorts, new Mesh(v4), "Feed ports", LapSec()));

        Voxels v5 = BuildSpikeVanes(S); voxSolid += v5;
        onStage(new StageResult(StageId.SpikeVanes, new Mesh(v5), "Spike vanes", LapSec()));

        Voxels v6 = BuildTopFlange(S); voxSolid += v6;
        onStage(new StageResult(StageId.TopFlange, new Mesh(v6), "Top flange", LapSec()));

        // Smoothen + bores (no separate layers — internal post-processing)
        voxSolid.Smoothen(0.3f);
        voxSolid -= BuildPostSmoothingBores(S);

        onStage(new StageResult(StageId.Final, new Mesh(voxSolid), "Final engine", LapSec()));
        return voxSolid;
    }

    public static Voxels Build(AeroSpec S)
    {
        Library.Log("╔══════════════════════════════════════════╗");
        Library.Log("║  CHANNELS-FIRST: channels → gas → solid  ║");
        Library.Log("║  Channels are primary. Chamber adapts.    ║");
        Library.Log("╚══════════════════════════════════════════╝");

        // ══════════════════════════════════════
        // STEP 1: CHANNELS FIRST
        // ══════════════════════════════════════
        Library.Log("Step 1: Channels (primary structure)...");
        var routeS = ChannelRouter.RouteShroudChannels(S);
        Library.Log($"  Shroud: {routeS.Spines.Count} channels.");
        Voxels voxChShroud = BuildChannelVoids(S, routeS, isShroud: true);

        var routeP = ChannelRouter.RouteSpikeChannels(S);
        Library.Log($"  Spike: {routeP.Spines.Count} channels.");
        Voxels voxChSpike = routeP.Spines.Count > 0
            ? BuildChannelVoids(S, routeP, isShroud: false)
            : new Voxels();

        // ══════════════════════════════════════
        // STEP 2: GAS PATH (adapts to channels)
        // ══════════════════════════════════════
        Library.Log("Step 2: Gas path (after channels)...");
        Voxels voxGas = BuildGasPathVoid(S);

        // ══════════════════════════════════════
        // STEP 3: MUTUAL EXCLUSION
        // ══════════════════════════════════════
        Library.Log("Step 3: Mutual exclusion...");
        float minWall = S.minPrintWall;
        voxChShroud.BoolSubtract(voxGas.voxOffset(minWall));
        if (routeP.Spines.Count > 0)
        {
            voxChSpike.BoolSubtract(voxGas.voxOffset(minWall));
            voxChShroud.BoolSubtract(voxChSpike.voxOffset(minWall));
            voxChSpike.BoolSubtract(voxChShroud.voxOffset(minWall));
        }
        Library.Log("  Channels clipped from gas + each other.");

        // ══════════════════════════════════════
        // STEP 4: UNION ALL VOIDS
        // ══════════════════════════════════════
        Library.Log("Step 4: Union all voids...");
        Voxels voxInner = voxChShroud;  // channels first
        voxInner.BoolAdd(voxChSpike);
        voxInner.BoolAdd(voxGas);       // gas path joins
        Library.Log("  Inner volume = channels + gas.");

        // ══════════════════════════════════════
        // STEP 5: WALLS = VARIABLE (per-z Barlow) OFFSET FROM VOIDS
        // ══════════════════════════════════════
        Library.Log("Step 5: Grow variable walls (Barlow per-z)...");
        Voxels voxOuter = BuildVariableWallShell(S, voxInner);
        // Representative wall (throat) — used only to position the nozzle-exit trim cap.
        float wallT = MathF.Max(HeatTransfer.WallThickness(S, S.zThroat), S.minPrintWall) + S.minPrintWall;
        Library.Log($"  Wall thickness: variable f(z) via Barlow (throat ≈ {wallT:F2} mm).");

        // ══════════════════════════════════════
        // STEP 6: FLUID-FIRST SUBTRACT (the moment)
        // ══════════════════════════════════════
        Library.Log("Step 6: Subtract voids from shell...");
        Voxels voxSolid = voxOuter;
        voxSolid.BoolSubtract(voxInner);

        // Trim below cowl (remove offset cap at nozzle exit)
        voxSolid.Trim(new BBox3(
            new Vector3(-200f, -200f, S.zCowl - wallT),
            new Vector3(200f, 200f, S.zInjector + 20f)));

        // Add spike nozzle body (solid cone — aerospike contour)
        voxSolid.BoolAdd(BuildSpikeBody(S));
        Library.Log("  Nozzle exit trimmed + spike body added.");

        // Drill axial LOX manifold as bore (not in void union — was sealing spike)
        voxSolid.BoolSubtract(BuildAxialManifold(S));
        Library.Log("  Axial LOX manifold drilled.");

        Library.Log("  Solid = shell - voids - exit - LOX bore.");

        // ══════════════════════════════════════
        // STEP 7: ADD LATTICE COMPONENTS
        // ══════════════════════════════════════
        Library.Log("Step 7: Lattice additions...");
        voxSolid += BuildShroudCollector(S);
        voxSolid += BuildShroudInlet(S);
        voxSolid += BuildFeedPorts(S);
        voxSolid += BuildSpikeVanes(S);
        voxSolid += BuildTopFlange(S);
        Library.Log("  + Collector, inlet, ports, vanes, flange.");

        // ══════════════════════════════════════
        // STEP 8: SMOOTHEN
        // ══════════════════════════════════════
        Library.Log("Step 8: Smoothen...");
        voxSolid.Smoothen(0.3f);

        // ══════════════════════════════════════
        // STEP 9: POST-SMOOTHING BORES
        // ══════════════════════════════════════
        Library.Log("Step 9: Post-smoothing bores...");
        voxSolid -= BuildPostSmoothingBores(S);

        Library.Log("Done — voids-first build complete.");
        return voxSolid;
    }

    // ─────────────────────────────────────
    // VARIABLE WALL: per-z Barlow wall shell around the void union
    //
    // Replaces the single scalar `voxInner.voxOffset(wallT)`. VariableWallImplicit
    // wraps the void SDF (via ScalarField) and returns dVoid - wall(z), where
    // wall(z) is the Barlow pressure thickness — thicker at high pressure / structural
    // zones, thinner where cooling carries the load. The min-print-wall clamp lives
    // inside the implicit (wall = max(wall, minPrintWall)), so thin spots stay printable.
    //
    // Voxelizing the implicit over the void bbox grown by maxWall gives `voxOuter`
    // = void ∪ wall-shell; Step 6 then subtracts voxInner to leave the shell.
    // ─────────────────────────────────────
    static Voxels BuildVariableWallShell(AeroSpec S, Voxels voxInner)
    {
        var wallField = new VariableWallImplicit(voxInner, S);

        // Bounds: void extent grown by the LARGEST wall the field can emit, so the
        // offset surface is never clipped. VariableWallImplicit applies a ×1.5 structural
        // multiplier near the injector and clamps to minPrintWall, so sample the Barlow
        // wall across the full z range and scale by the same worst-case factor.
        voxInner.CalculateProperties(out _, out BBox3 voidBox);

        float maxBarlow = S.minPrintWall;
        for (float z = S.zTip; z <= S.zInjector + 5f; z += 2f)
            maxBarlow = MathF.Max(maxBarlow, HeatTransfer.WallThickness(S, z));
        // ×1.5 = largest structural multiplier in VariableWallImplicit; + margins.
        float maxWall = maxBarlow * 1.5f + 2f * S.minPrintWall + 3f;

        BBox3 bounds = new BBox3(
            new Vector3(voidBox.vecMin.X - maxWall, voidBox.vecMin.Y - maxWall, voidBox.vecMin.Z - maxWall),
            new Vector3(voidBox.vecMax.X + maxWall, voidBox.vecMax.Y + maxWall, voidBox.vecMax.Z + maxWall));

        return new Voxels(wallField, bounds);
    }

    // ─────────────────────────────────────
    // GAS PATH: split into Nozzle + Chamber + Dome
    // ─────────────────────────────────────

    static Voxels BuildGasPathVoid(AeroSpec S)
    {
        // Full gas path = all three zones
        Voxels v = BuildAnnularZone(S, S.zCowl, S.zChBot);  // nozzle
        v.BoolAdd(BuildAnnularZone(S, S.zChBot, S.zChTop));  // chamber
        v.BoolAdd(BuildAnnularZone(S, S.zChTop, S.zInjector)); // dome
        return v;
    }

    static Voxels BuildNozzleVoid(AeroSpec S) => BuildAnnularZone(S, S.zCowl, S.zChBot);
    static Voxels BuildChamberVoid(AeroSpec S) => BuildAnnularZone(S, S.zChBot, S.zChTop);
    static Voxels BuildDomeVoid(AeroSpec S) => BuildAnnularZone(S, S.zChTop, S.zInjector);

    /// <summary>Build annular Lattice void between spike and shroud for a z-range.</summary>
    static Voxels BuildAnnularZone(AeroSpec S, float zStart, float zEnd)
    {
        Lattice lat = new Lattice();
        int nSegs = 48;
        int nZ = (int)MathF.Max((zEnd - zStart) / 1.5f, 10);

        for (int iz = 0; iz < nZ; iz++)
        {
            float z0 = zStart + iz * (zEnd - zStart) / nZ;
            float z1 = zStart + (iz + 1) * (zEnd - zStart) / nZ;

            float rSpike0 = ChamberSizing.SpikeProfile(S, z0);
            float rShroud0 = ChamberSizing.ShroudProfile(S, z0);
            float rSpike1 = ChamberSizing.SpikeProfile(S, z1);
            float rShroud1 = ChamberSizing.ShroudProfile(S, z1);

            if (rShroud0 < 1f && rShroud1 < 1f) continue;

            float rMid0 = (rSpike0 + rShroud0) / 2f;
            float rBeam0 = MathF.Max((rShroud0 - rSpike0) / 2f, 0.5f);
            float rMid1 = (rSpike1 + rShroud1) / 2f;
            float rBeam1 = MathF.Max((rShroud1 - rSpike1) / 2f, 0.5f);

            for (int ia = 0; ia < nSegs; ia++)
            {
                float phi = ia * 2f * MathF.PI / nSegs;
                float c = MathF.Cos(phi), s = MathF.Sin(phi);
                lat.AddBeam(
                    new Vector3(rMid0 * c, rMid0 * s, z0), rBeam0,
                    new Vector3(rMid1 * c, rMid1 * s, z1), rBeam1);
            }
        }

        return new Voxels(lat);
    }

    // ─────────────────────────────────────
    // SPIKE NOZZLE BODY: solid cone below cowl (aerospike contour)
    // This is NOT part of voids-first — it's a separate solid.
    // Below cowl, gas flows in open air along the spike surface.
    // ─────────────────────────────────────
    static Voxels BuildSpikeBody(AeroSpec S)
    {
        Lattice lat = new Lattice();
        int nZ = 60;
        float zStart = S.zTip;
        float zEnd = S.zThroat + 5f;

        for (int i = 0; i < nZ; i++)
        {
            float z0 = zStart + i * (zEnd - zStart) / nZ;
            float z1 = zStart + (i + 1) * (zEnd - zStart) / nZ;
            float r0 = ChamberSizing.SpikeProfile(S, z0);
            float r1 = ChamberSizing.SpikeProfile(S, z1);
            if (r0 < 0.3f && r1 < 0.3f) continue;
            lat.AddBeam(
                new Vector3(0, 0, z0), MathF.Max(r0, 0.3f),
                new Vector3(0, 0, z1), MathF.Max(r1, 0.3f));
        }

        return new Voxels(lat);
    }

    // ─────────────────────────────────────
    // CHANNEL VOIDS: TRUE channel geometry via RoutedChannelFieldImplicit
    //
    // The implicit evaluates a superellipse cross-section per voxel with:
    //   - print-angle-adaptive superellipse exponent
    //   - spatial θ-modulation of radial half-height (continuous — no per-index steps)
    //   - turbulator ribs (smoothstep-faded, continuous)
    //   - port-aware widening folded into the field
    // This replaces the old circular-Lattice approximation (radius min(w,h)/2),
    // which discarded all of the above. Mirrors Program.cs (~line 89):
    //   new RoutedChannelFieldImplicit(spec, route, isShroud)
    //
    // CONTINUITY: the implicit's fSignedDistance is continuous (θ-mod is a spatial
    // atan2 function, not a per-channel step), so the rendered voids stay watertight.
    // ─────────────────────────────────────
    static Voxels BuildChannelVoids(AeroSpec S, ChannelRouteResult route, bool isShroud)
    {
        if (route.Spines.Count == 0)
            return new Voxels();

        var field = new RoutedChannelFieldImplicit(S, route, isShroud);
        return new Voxels(field, field.GetBBox());
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

        // v7: port positions are DERIVED (computed in HeatTransfer.DerivePortPositions),
        // not hardcoded. Port = consequence of channel geometry, not user input.

        // Fuel port — at derived position
        float zFuel = S.fuelPortZ;
        float phiFuel = S.fuelPortPhi;
        float rSh = ChamberSizing.ShroudProfile(S, zFuel);
        if (rSh < 2f) rSh = S.rShroudThroat;
        float wallF = HeatTransfer.WallThickness(S, zFuel);
        float rStart = rSh + wallF;
        float cfuel = MathF.Cos(phiFuel);
        float sfuel = MathF.Sin(phiFuel);
        lat.AddBeam(
            new Vector3(rStart * cfuel, rStart * sfuel, zFuel), rPort * 1.15f,
            new Vector3((rStart + portLen) * cfuel, (rStart + portLen) * sfuel, zFuel), rPort);

        // LOX port — at derived position
        float zLox = S.loxPortZ;
        float phiLox = S.loxPortPhi;
        float rShTop = ChamberSizing.ShroudProfile(S, zLox);
        if (rShTop < 2f) rShTop = S.rShroudChamber;
        float wallL = HeatTransfer.WallThickness(S, zLox);
        float rStartL = rShTop + wallL;
        float clox = MathF.Cos(phiLox);
        float slox = MathF.Sin(phiLox);
        lat.AddBeam(
            new Vector3(rStartL * clox, rStartL * slox, zLox), rPort * 1.15f,
            new Vector3((rStartL + portLen) * clox, (rStartL + portLen) * slox, zLox), rPort);

        // Igniter port — angled entry at derived position
        float zIgn = S.igniterPortZ;
        float phiIgn = S.igniterPortPhi;
        float rShIgn = ChamberSizing.ShroudProfile(S, zIgn);
        if (rShIgn < 2f) rShIgn = S.rShroudChamber;
        float wallI = HeatTransfer.WallThickness(S, zIgn);
        float rIgnStart = rShIgn + wallI;
        float cign = MathF.Cos(phiIgn);
        float sign = MathF.Sin(phiIgn);
        float ignLen = 6f;
        lat.AddBeam(
            new Vector3(rIgnStart * cign, rIgnStart * sign, zIgn), 1.2f,
            new Vector3((rIgnStart + ignLen * 0.87f) * cign,
                        (rIgnStart + ignLen * 0.87f) * sign,
                        zIgn + ignLen * 0.5f), 1.0f);

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
