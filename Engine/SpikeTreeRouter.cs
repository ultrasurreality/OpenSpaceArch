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

// SpikeTreeRouter.cs — P2 PROTOTYPE (default OFF, NOT wired into FluidFirst)
//
// PURPOSE
//   A plug-nozzle interior is better served by a 3-D BRANCHING-TRUSS / tree-growth
//   coolant volume than by a bundle of longitudinal channels: the heat load sits on
//   the gas-side surface, and a branching manifold reaches it with less pressure drop.
//   OpenSpaceArch currently models the spike interior as longitudinal helical channels
//   (ChannelRouter.RouteSpikeChannels). This file is the COMPILING SKELETON of a future
//   3-D tree router that grows a coolant manifold from a single root at the spike base,
//   branches outward toward the gas-side surface (where the heat load lives), obeys
//   Murray's law at every bifurcation, and stays strictly inside the spike boundary volume.
//
//   It is reachable later behind AeroSpec.useSpikeTree (default false). NOTHING here is
//   called by the production build yet. The working longitudinal-channel spike remains
//   the default and is untouched.
//
// FLUID-FIRST DISCIPLINE (when this is eventually wired in)
//   The tree is a VOID. Generate() returns the coolant skeleton (root → branches → tips);
//   a downstream IImplicit (a SpikeTreeFieldImplicit, not written yet) would render the
//   swept tapered tubes as ONE continuous SDF, exactly as RoutedChannelFieldImplicit does
//   for channels. The metal spike then grows OUTWARD from that void (VariableWallImplicit)
//   and the void is subtracted. We never "place" the truss as solid beams — the solid is
//   the consequence of the coolant tree plus its wall offset.
//
// SDF CONTINUITY (the project's #1 hard-won lesson)
//   The eventual field MUST be a continuous spatial function. Like RoutedChannelFieldImplicit
//   it would do a nearest-segment lookup over the flattened edge list and a smooth (capsule /
//   tapered-capsule) cross-section blended with smoothstep — NEVER a per-branch step function
//   in fSignedDistance. Branch radii vary smoothly along each edge (linear taper) and the
//   union of edges is a smooth-min, so the rendered void stays watertight.
//
// STATUS: skeleton. Generate() builds a deterministic placeholder tree and logs it.
//         The real growth algorithm (space colonization, see the note) is stubbed with a
//         single trunk + first-generation branches so the data structures and the public
//         surface are exercised and the file compiles. No geometry is voxelized here.

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

/// <summary>
/// One node in the coolant tree. Nodes are connected by <see cref="SpikeTreeEdge"/>.
/// Position is in engine mm (same frame as the spines from ChannelRouter: Z is the axis,
/// XY is the cross-section, tip at Z=0). Radius is the local tube radius at this node.
/// </summary>
public sealed class SpikeTreeNode
{
    public int Index;                 // stable index into SpikeTree.Nodes
    public int ParentIndex;           // -1 for the root
    public Vector3 Position;          // mm
    public float Radius;              // mm — tube radius at this node (Murray's-law sized)
    public int Generation;            // 0 = root/trunk, increments per bifurcation
    public bool IsTip;                // true if a leaf (delivers coolant to the hot surface)

    public SpikeTreeNode(int index, int parentIndex, Vector3 position, float radius,
                         int generation, bool isTip)
    {
        Index = index;
        ParentIndex = parentIndex;
        Position = position;
        Radius = radius;
        Generation = generation;
        IsTip = isTip;
    }
}

/// <summary>
/// A directed coolant edge from <see cref="A"/> (closer to root) to <see cref="B"/>
/// (closer to a tip). Radii taper linearly A→B; the downstream field samples this as a
/// tapered capsule so the SDF is continuous along the edge.
/// </summary>
public readonly record struct SpikeTreeEdge(int A, int B, float RadiusA, float RadiusB);

/// <summary>
/// The full coolant tree for one spike: the node list, the edge list (flattened for a
/// nearest-segment SDF lookup), and the root index. This is the analogue of
/// <see cref="ChannelRouteResult"/> for the branching-truss topology.
/// </summary>
public sealed class SpikeTree
{
    public List<SpikeTreeNode> Nodes { get; } = new();
    public List<SpikeTreeEdge> Edges { get; } = new();
    public int RootIndex { get; set; } = -1;

    public int TipCount => Nodes.Count(n => n.IsTip);

    /// <summary>Axis-aligned bbox of all node positions, grown by the largest tube radius
    /// plus a margin — the bounds a future SpikeTreeFieldImplicit would voxelize over.</summary>
    public BBox3 GetBBox(float margin = 8f)
    {
        if (Nodes.Count == 0)
            return new BBox3(Vector3.Zero, Vector3.Zero);

        var min = new Vector3(float.MaxValue);
        var max = new Vector3(float.MinValue);
        float maxR = 0f;
        foreach (var n in Nodes)
        {
            min = Vector3.Min(min, n.Position);
            max = Vector3.Max(max, n.Position);
            maxR = MathF.Max(maxR, n.Radius);
        }
        var grow = new Vector3(maxR + margin);
        return new BBox3(min - grow, max + grow);
    }
}

/// <summary>
/// Grows the coolant branching-truss inside the spike boundary volume. P2 prototype:
/// the public surface and data structures are final-shaped, but the growth body is a
/// deterministic placeholder (trunk + one generation of branches). The real algorithm is
/// space-colonization constrained to the spike interior — see the design note.
/// </summary>
public static class SpikeTreeRouter
{
    // Golden angle for low-discrepancy azimuthal placement of branch tips (same constant
    // and intent as ChannelRouter — biological, aperiodic distribution).
    const float GOLDEN_ANGLE = 2.3999632297f;

    // Distinct seed lane so the tree's randomness never collides with the channel router's.
    internal const int TREE_SEED_OFFSET = 2000;

    // Murray's law exponent: a parent of radius r_p feeding children r_c obeys
    //   r_p^MURRAY = Σ r_c^MURRAY   (3 = classic Murray for laminar viscous flow).
    const float MURRAY_EXPONENT = 3f;

    /// <summary>
    /// Build the coolant tree for the spike. Deterministic in (S.channelSeed): same seed →
    /// identical tree, like the channel router. Does NOT voxelize and does NOT touch the
    /// production build — callers wire it in later behind AeroSpec.useSpikeTree.
    /// </summary>
    public static SpikeTree Generate(AeroSpec S)
    {
        Library.Log("  [SpikeTreeRouter] PROTOTYPE — building placeholder coolant tree (not wired)...");

        var tree = new SpikeTree();
        int seed = S.channelSeed + TREE_SEED_OFFSET;

        // ── Boundary volume the tree must live inside ─────────────────────────
        // Root sits just above the spike tip on the axis; the trunk climbs the axis to the
        // chamber-top station. Tips reach toward the gas-side surface (SpikeProfile) minus a
        // wall budget, so a printable wall always remains between coolant and hot gas.
        float zRoot = S.zTip + 3f;
        float zTop  = S.zChTop;
        if (zTop <= zRoot)
        {
            Library.Log("  [SpikeTreeRouter] spike too short for a tree — returning empty.");
            return tree;
        }

        // Tip count scales with the longitudinal-channel count so the cooling capacity is
        // comparable to the working path (a fair starting point for later A/B comparison).
        int nTips = Math.Max(S.nChannelsSpike, 4);

        // ── Root + trunk (generation 0) along the axis ────────────────────────
        // Trunk radius is sized so that, after Murray-splitting into nTips tips of the
        // minimum printable channel radius, the law balances at the root.
        float tipRadius   = MathF.Max(S.minChannel / 2f, 0.5f);
        float trunkRadius = MurrayParentRadius(tipRadius, nTips);

        int rootIdx = AddNode(tree, parentIndex: -1,
                              position: new Vector3(0f, 0f, zRoot),
                              radius: trunkRadius, generation: 0, isTip: false);
        tree.RootIndex = rootIdx;

        // A few trunk nodes up the axis give the downstream SDF a smooth spine to taper along.
        int trunkSteps = 4;
        int prevIdx = rootIdx;
        float branchZ = zRoot + 0.55f * (zTop - zRoot); // where the trunk starts to branch out
        for (int k = 1; k <= trunkSteps; k++)
        {
            float z = zRoot + (branchZ - zRoot) * k / trunkSteps;
            int idx = AddNode(tree, parentIndex: prevIdx,
                              position: new Vector3(0f, 0f, z),
                              radius: trunkRadius, generation: 0, isTip: false);
            AddEdge(tree, prevIdx, idx);
            prevIdx = idx;
        }
        int branchBaseIdx = prevIdx;

        // ── First generation of branches reaching toward the gas-side surface ─
        // PLACEHOLDER: one straight branch per tip, azimuth by golden-angle phyllotaxis +
        // deterministic jitter. The real router would recurse (each branch bifurcates again
        // until tips are dense enough on the surface), with space-colonization choosing
        // directions from attractor points sampled inside the spike shell. Murray's law sets
        // each child radius from its share of the parent flow.
        float childRadius = MurrayChildRadius(trunkRadius, nTips);
        for (int i = 0; i < nTips; i++)
        {
            float phi = (i * GOLDEN_ANGLE
                         + ChannelRouter.DetRand(seed, i, ChannelRouter.AXIS_PHI) * 0.3f);

            // Tip station spread along the upper spike so tips are not co-planar.
            float frac = (i + 0.5f) / nTips;
            float zTip = branchZ + (zTop - branchZ) * frac;

            // Tip radius target: gas-side surface minus a wall budget, clamped to stay
            // strictly inside the spike (>= a small positive radius near the slender top).
            float rSurface = ChamberSizing.SpikeProfile(S, zTip);
            float wall = HeatTransfer.WallThickness(S, zTip);
            float rTip = MathF.Max(rSurface - wall - tipRadius, 0.5f);

            var tipPos = new Vector3(rTip * MathF.Cos(phi), rTip * MathF.Sin(phi), zTip);

            int tipIdx = AddNode(tree, parentIndex: branchBaseIdx,
                                 position: tipPos, radius: tipRadius,
                                 generation: 1, isTip: true);
            AddEdge(tree, branchBaseIdx, tipIdx, childRadius, tipRadius);
        }

        Library.Log($"    [SpikeTreeRouter] placeholder tree: {tree.Nodes.Count} nodes, "
                  + $"{tree.Edges.Count} edges, {tree.TipCount} tips, "
                  + $"trunk R={trunkRadius:F2}mm → tip R={tipRadius:F2}mm (Murray check ok).");
        return tree;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Murray's-law helpers
    // ──────────────────────────────────────────────────────────────────────────

    /// <summary>Radius a parent must have to feed <paramref name="nChildren"/> equal children
    /// of <paramref name="childRadius"/> under Murray's law: r_p = (n · r_c^k)^(1/k).</summary>
    static float MurrayParentRadius(float childRadius, int nChildren)
    {
        float sum = nChildren * MathF.Pow(childRadius, MURRAY_EXPONENT);
        return MathF.Pow(sum, 1f / MURRAY_EXPONENT);
    }

    /// <summary>Radius of each of <paramref name="nChildren"/> equal children that together
    /// carry the flow of a parent of <paramref name="parentRadius"/>:
    /// r_c = r_p / n^(1/k).</summary>
    static float MurrayChildRadius(float parentRadius, int nChildren)
    {
        return parentRadius / MathF.Pow(nChildren, 1f / MURRAY_EXPONENT);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Tree construction helpers
    // ──────────────────────────────────────────────────────────────────────────

    static int AddNode(SpikeTree tree, int parentIndex, Vector3 position, float radius,
                       int generation, bool isTip)
    {
        int idx = tree.Nodes.Count;
        tree.Nodes.Add(new SpikeTreeNode(idx, parentIndex, position, radius, generation, isTip));
        return idx;
    }

    static void AddEdge(SpikeTree tree, int a, int b)
        => AddEdge(tree, a, b, tree.Nodes[a].Radius, tree.Nodes[b].Radius);

    static void AddEdge(SpikeTree tree, int a, int b, float radiusA, float radiusB)
        => tree.Edges.Add(new SpikeTreeEdge(a, b, radiusA, radiusB));
}
