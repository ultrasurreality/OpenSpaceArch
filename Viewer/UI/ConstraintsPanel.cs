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

// ConstraintsPanel.cs — Phase 1 live constraint visualization.
//
// Separate ImGui panel that renders each Viability check as a colored slack
// bar, reactive to slider changes in real time. This is the "CSP outer-layer
// visibility" piece of the two-layer architecture — it does not change
// generation, only shows where the current spec sits relative to every
// physics/manufacturing constraint boundary.
//
// Instantiated once in AppMain and Draw()n every frame, but the Viability it
// renders is NOT recomputed every frame. AppMain.UpdatePreviewProfiles early-
// returns when the parameter hash is unchanged (F_thrust, Pc, O/F, voxel,
// channelMode, CR, L*, SF), so EngineValidator.Check + _liveViability only
// refresh when a slider actually moves a tracked parameter. Between changes
// this panel redraws the SAME cached Viability — it is reactive to edits, not
// per-frame live. (Parameters outside that hash, e.g. twist, won't refresh it.)
//
// See ARCHITECTURE.md, "VALIDATION".

using System.Numerics;
using ImGuiNET;
using OpenSpaceArch.Viewer.Simulation;

namespace OpenSpaceArch.Viewer.UI;

public sealed class ConstraintsPanel
{
    // Cached sorted list to avoid re-allocating every frame
    private readonly List<CheckResult> _sorted = new(capacity: 32);

    public void Draw(Viability live)
    {
        ImGui.SetNextWindowPos(new Vector2(1268, 320), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(332, 560), ImGuiCond.FirstUseEver);
        ImGui.Begin("Constraints (live)");

        if (live.Checks == null || live.Checks.Count == 0)
        {
            ImGui.TextDisabled("waiting for physics...");
            ImGui.End();
            return;
        }

        int total = live.Checks.Count;
        int passing = 0;
        for (int i = 0; i < total; i++)
            if (live.Checks[i].Passed) passing++;
        int violated = total - passing;

        // Header status
        if (violated == 0)
        {
            ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(0.3f, 1f, 0.4f, 1f));
            ImGui.Text($"{passing}/{total} passing — all green");
        }
        else
        {
            ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(1f, 0.55f, 0.3f, 1f));
            ImGui.Text($"{passing}/{total} passing — {violated} violated");
        }
        ImGui.PopStyleColor();

        ImGui.TextDisabled("worst first. bar = slack, green=safe red=over.");
        ImGui.Separator();

        // Sort worst-first (lowest slack on top)
        _sorted.Clear();
        _sorted.AddRange(live.Checks);
        _sorted.Sort(static (a, b) => a.SlackNormalized.CompareTo(b.SlackNormalized));

        ImGui.BeginChild("constraints_scroll", new Vector2(0, 0), ImGuiChildFlags.None);

        foreach (var c in _sorted)
            DrawOneRow(c);

        ImGui.EndChild();
        ImGui.End();
    }

    private static void DrawOneRow(CheckResult c)
    {
        // Row 1: status icon + name
        if (c.Passed)
        {
            ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(0.35f, 0.95f, 0.45f, 1f));
            ImGui.TextUnformatted("[OK]");
            ImGui.PopStyleColor();
        }
        else
        {
            ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(1f, 0.4f, 0.35f, 1f));
            ImGui.TextUnformatted("[!!]");
            ImGui.PopStyleColor();
        }

        ImGui.SameLine();
        ImGui.TextUnformatted(c.Name);

        // Row 2: slack bar, centered at midpoint
        var drawList = ImGui.GetWindowDrawList();
        var cursor = ImGui.GetCursorScreenPos();

        const float barHeight = 8f;
        float totalWidth = ImGui.GetContentRegionAvail().X - 4f;
        float midX = cursor.X + totalWidth * 0.5f;
        float half = totalWidth * 0.5f;

        // Background track
        uint trackColor = ImGui.GetColorU32(new Vector4(0.18f, 0.18f, 0.22f, 1f));
        drawList.AddRectFilled(
            new Vector2(cursor.X, cursor.Y),
            new Vector2(cursor.X + totalWidth, cursor.Y + barHeight),
            trackColor);

        // Centerline marker
        uint centerColor = ImGui.GetColorU32(new Vector4(0.5f, 0.5f, 0.55f, 1f));
        drawList.AddRectFilled(
            new Vector2(midX - 0.5f, cursor.Y),
            new Vector2(midX + 0.5f, cursor.Y + barHeight),
            centerColor);

        // Slack fill (left=red if violated, right=green if safe)
        float clampedNorm = Math.Clamp(c.SlackNormalized, -1f, 1f);
        if (clampedNorm >= 0f)
        {
            // Green fill, right from center
            uint fillColor = ImGui.GetColorU32(new Vector4(0.25f, 0.85f, 0.35f, 1f));
            float fillWidth = clampedNorm * half;
            drawList.AddRectFilled(
                new Vector2(midX, cursor.Y),
                new Vector2(midX + fillWidth, cursor.Y + barHeight),
                fillColor);
        }
        else
        {
            // Red fill, left from center
            uint fillColor = ImGui.GetColorU32(new Vector4(0.95f, 0.3f, 0.3f, 1f));
            float fillWidth = (-clampedNorm) * half;
            drawList.AddRectFilled(
                new Vector2(midX - fillWidth, cursor.Y),
                new Vector2(midX, cursor.Y + barHeight),
                fillColor);
        }

        // Advance cursor past the bar
        ImGui.Dummy(new Vector2(totalWidth, barHeight));

        // Row 3: detail text
        ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(0.7f, 0.72f, 0.78f, 1f));
        ImGui.TextWrapped(c.Detail);
        ImGui.PopStyleColor();

        ImGui.Spacing();
    }
}
