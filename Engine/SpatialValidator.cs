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

// SpatialValidator.cs — Voxel-free geometric validation
//
// Checks spatial conflicts between the engine's load-bearing elements
// (shroud/spike channels vs chamber, collector, flange bolts) with pure
// math. Every element is a function of position and size in z.
//
// It does NOT treat feed ports, vanes and the manifold as hard conflicts:
// they intersect channels BY DESIGN, and mutual exclusion (voxOffset
// subtract) carves the channels around them. For those, only the size is
// sanity-checked (a port must not kill >25% of the channels).

using System.Numerics;

namespace OpenSpaceArch.Engine;

public static class SpatialValidator
{
    public record Conflict(string ElementA, string ElementB, float Z, float Distance, float MinRequired);

    public static List<Conflict> Validate(AeroSpec S)
    {
        var conflicts = new List<Conflict>();
        float minWall = S.minPrintWall; // 0.5mm — real LPBF minimum
        float zStep = 1f; // check every mm

        // ── Shroud channels vs chamber (gas path) ──
        // v7: channels stop at zChTop (chamber top), not at zInjector — above
        // them sits the collector, not a channel. Only that zone is checked.
        for (float z = S.zCowl + 2f; z <= S.zChTop - 2f; z += zStep)
        {
            float rShroud = ChamberSizing.ShroudProfile(S, z);
            if (rShroud < 2f) continue;
            float wall = HeatTransfer.WallThickness(S, z);
            var (cw, ch) = HeatTransfer.ChannelRect(S, z);

            // Inner channel edge = rShroud + wall; the chamber wall up to the
            // channel IS the clearance. It must be > minWall.
            float gap = wall; // wall IS the gap
            if (gap < minWall)
                conflicts.Add(new("shroud_channel", "chamber", z, gap, minWall));

            // Do the channels fit around the circumference?
            // Same rCenter formula as HeatTransfer.ChannelRect
            float rCenter = rShroud + wall + 2f;
            float circ = 2f * MathF.PI * rCenter;
            float needed = S.nChannelsShroud * (cw + S.minRibWall);
            if (needed > circ)
                conflicts.Add(new("shroud_channels", "circumference", z, circ, needed));
        }

        // ── Spike channels vs chamber ──
        // v7: spike channels also stop at zChTop; above that it is solid spike body.
        for (float z = S.zCowl + 2f; z <= S.zChTop - 2f; z += zStep)
        {
            float rSpike = ChamberSizing.SpikeProfile(S, z);
            if (rSpike < 3f) continue;
            float wall = HeatTransfer.WallThickness(S, z);
            var (cw, ch) = HeatTransfer.ChannelRectSpike(S, z);

            // Spike wall up to the channel = clearance; must be > minWall.
            if (wall < minWall)
                conflicts.Add(new("spike_channel", "chamber", z, wall, minWall));

            // Do the spike channels fit?
            float rCenter = rSpike - wall - ch / 2f;
            if (rCenter < 2f) continue;
            float circ = 2f * MathF.PI * rCenter;
            float needed = S.nChannelsSpike * (cw + S.minRibWall);
            if (needed > circ)
                conflicts.Add(new("spike_channels", "circumference", z, circ, needed));
        }

        // Spike channels vs axial manifold — handled by mutual exclusion (voxOffset subtract)
        // Not a hard conflict — manifold carves through spike channels by design

        // ── Collector vs shroud channels at zChTop ──
        {
            float z = S.zChTop;
            float rShroud = ChamberSizing.ShroudProfile(S, z);
            float wall = HeatTransfer.WallThickness(S, z);
            var (cw, ch) = HeatTransfer.ChannelRect(S, z);
            float chCenter = rShroud + wall + ch / 2f;
            float collectorR = S.manifoldRadius;

            // The collector sits at the same radius — overlapping the channels is
            // expected. What we check is that it does not stick out past the
            // shroud outer wall
            float outerEdge = chCenter + collectorR;
            float maxOuter = rShroud + wall + ch + 3f; // reasonable limit
            if (outerEdge > maxOuter + 5f)
                conflicts.Add(new("collector", "outer_bound", z, maxOuter, outerEdge));
        }

        // Feed ports and vanes intersect channels BY DESIGN.
        // Mutual exclusion (voxOffset subtract) carves the channels around them.
        // That is not an error — we only check the port is not oversized.
        {
            float zFuel = S.zCowl + 3f;
            float rShroud = ChamberSizing.ShroudProfile(S, zFuel);
            if (rShroud < 2f) rShroud = S.rShroudThroat;
            var (cw, ch) = HeatTransfer.ChannelRect(S, zFuel);
            float chCenter = rShroud + HeatTransfer.WallThickness(S, zFuel) + ch / 2f;
            float angularSpanPort = 2f * MathF.Atan2(S.feedPortRadius, chCenter);
            float angularSpanChannel = (cw + S.minRibWall) / chCenter;
            int affectedChannels = (int)MathF.Ceiling(angularSpanPort / angularSpanChannel);
            // A port killing >25% of the channels IS a problem
            if (affectedChannels > S.nChannelsShroud / 4)
                conflicts.Add(new("fuel_port", "too_many_channels", zFuel,
                    affectedChannels, S.nChannelsShroud / 4f));
        }

        // ── Injector holes vs channels ──
        // Film cooling holes are INSIDE the chamber (rShroudChamber*0.85 < rShroud)
        // They don't conflict with shroud channels which are OUTSIDE the chamber wall
        // Only check: bolt holes through flange vs top of channels
        {
            float z = S.zInjector;
            float rShroud = ChamberSizing.ShroudProfile(S, z);
            float wall = HeatTransfer.WallThickness(S, z);
            var (cw, ch) = HeatTransfer.ChannelRect(S, z);
            float chOuterEdge = rShroud + wall + ch;

            float rBoltCircle = S.rShroudChamber + S.mountFlangeExtent * 0.6f;
            float rBolt = 2.0f;
            // Flange bolts — clear of the channels? Bolt inner edge vs channel outer edge
            float gapBolt = rBoltCircle - rBolt - chOuterEdge;
            if (gapBolt < minWall)
                conflicts.Add(new("bolt_hole", "shroud_channel_top", z, gapBolt, minWall));
        }

        return conflicts;
    }

    public static void PrintReport(AeroSpec S)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var conflicts = Validate(S);
        sw.Stop();

        Console.WriteLine($"\nSpatial validation: {sw.ElapsedTicks * 1000000 / System.Diagnostics.Stopwatch.Frequency} µs");
        Console.WriteLine($"Conflicts found: {conflicts.Count}");

        if (conflicts.Count == 0)
        {
            Console.WriteLine("  All clear — no spatial conflicts detected.");
            return;
        }

        // Group by conflict type
        var groups = conflicts.GroupBy(c => $"{c.ElementA} vs {c.ElementB}");
        foreach (var g in groups)
        {
            var worst = g.OrderBy(c => c.Distance).First();
            Console.WriteLine($"  {g.Key}: {g.Count()} conflicts, worst at z={worst.Z:F1}mm (gap={worst.Distance:F2}, need={worst.MinRequired:F2})");
        }
    }
}
