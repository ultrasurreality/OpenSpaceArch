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

// ControlPanel.cs — ImGui overlay for OpenSpaceArch cinematic viewer.

using System.Numerics;
using ImGuiNET;
using OpenSpaceArch.Engine;
using OpenSpaceArch.Viewer.Pipeline;
using OpenSpaceArch.Viewer.Rendering;
using OpenSpaceArch.Viewer.Simulation;

namespace OpenSpaceArch.Viewer.UI;

public sealed class ControlPanel
{
    // ── Boundary conditions (mission spec + printer) ──
    public float Thrust = 5000f;          // N — mission
    // VoxelSize is the INTERACTIVE PREVIEW/BUILD resolution driven from this panel.
    // It is deliberately distinct from the two other voxel defaults in the codebase:
    //   • AeroSpec.voxelSize = 0.4 mm  — headless/batch STL generation (G4's file, fits ~5 GB RAM)
    //   • AppMain.Run() InitHeadless(0.5 mm) — the PicoGK core grid the viewer runs on
    // These three are intentionally NOT forced equal (preview vs build trade resolution
    // for responsiveness). Lowered 1.0 → 0.5 mm to (a) match the 0.5 mm core grid AppMain
    // initialises, and (b) keep clear of EngineValidator's C12 boundary (voxel < throatGap/2).
    // For the default 5 kN/110 bar engine throatGap ≈ 3.1 mm, so throatGap/2 ≈ 1.53 mm:
    // 1.0 mm voxel still PASSES C12 (≈3 voxels across the annulus) but that is the bare
    // 2-voxel minimum and risks the thin-annulus non-watertight failure mode; 0.5 mm gives
    // ≈6 voxels across the throat gap — safe headroom and far from the C12 limit. The 5 kN
    // throat never *fails* C12 anywhere on the Pc slider (gap/2 ≥ 1.11 mm even at 200 bar).
    public float VoxelSize = 0.5f;        // mm — interactive preview/build resolution
    public ChannelMode ChannelMode = ChannelMode.Routed_v5b;
    public float MinWall = 0.5f;          // mm — LPBF min wall
    public float MaxOverhang = 45f;       // degrees — LPBF max overhang

    // ── Design variables (sweep searches these, Manual lets you set directly) ──
    public float PcBar = 110f;            // bar
    public float OF = 3.2f;
    public float CR = 4.0f;
    public float Lstar = 0.4f;            // m
    public float SF = 1.5f;
    public float TwistTurns = 2.0f;

    public BuildMode BuildMode = BuildMode.ZSliceSlabs;
    public int ZSliceCount = 24;

    public bool RegenerateRequested;
    public bool ResetCameraRequested;
    public bool IgniteRequested;
    public bool ShutdownRequested;

    // v7: three-tab UI. SweepPanel embedded in Search tab.
    public SweepPanel? SweepPanel;

    public void Draw(PipelineController pipeline, Renderer renderer, int sceneStageCount,
                     AeroSpec lastBuiltSpec, StartupSequence startup, Viability viability)
    {
        DrawMainPanel(pipeline, sceneStageCount);
        DrawLayersPanel(renderer);
        DrawLogPanel(pipeline);
        DrawEngineStatePanel(renderer, startup, lastBuiltSpec, viability);
        DrawStylePanel(renderer);
    }

    public AeroSpec BuildSpec()
    {
        return new AeroSpec
        {
            F_thrust = Thrust,
            Pc = PcBar * 1e5f,
            OF_ratio = OF,
            CR = CR,
            Lstar = Lstar,
            SF = SF,
            channelTwistTurns = TwistTurns,
            voxelSize = VoxelSize,
            channelMode = ChannelMode,
            minPrintWall = MinWall,
            maxOverhang = MaxOverhang,
        };
    }

    /// <summary>
    /// Writes the 8 slider-editable fields of <paramref name="spec"/> back
    /// into this panel and triggers a regenerate. Used by Phase 2's
    /// <c>SweepPanel.OnApplyWinner</c> to push a winning design into the
    /// viewer's normal build flow.
    /// </summary>
    public void ApplyFromSpec(AeroSpec spec)
    {
        Thrust = spec.F_thrust;
        PcBar = spec.Pc / 1e5f;
        OF = spec.OF_ratio;
        CR = spec.CR;
        Lstar = spec.Lstar;
        SF = spec.SF;
        TwistTurns = spec.channelTwistTurns;
        VoxelSize = spec.voxelSize;
        // ChannelMode / BuildMode / ZSliceCount are user-set preferences,
        // leave untouched.
        RegenerateRequested = true;
    }

    // ─────────────────────────────────────────────────────────────
    // v7: Main panel with three tabs — Mission / Search / Manual
    // ─────────────────────────────────────────────────────────────

    private void DrawMainPanel(PipelineController pipeline, int sceneStageCount)
    {
        ImGui.SetNextWindowPos(new Vector2(12, 12), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(340, 680), ImGuiCond.FirstUseEver);
        ImGui.Begin("Engine Designer");

        if (ImGui.BeginTabBar("DesignerTabs"))
        {
            if (ImGui.BeginTabItem("Mission"))
            {
                DrawMissionTab();
                ImGui.EndTabItem();
            }
            if (ImGui.BeginTabItem("Search"))
            {
                DrawSearchTab();
                ImGui.EndTabItem();
            }
            if (ImGui.BeginTabItem("Manual"))
            {
                DrawManualTab();
                ImGui.EndTabItem();
            }
            ImGui.EndTabBar();
        }

        ImGui.Separator();
        DrawBuildSection(pipeline, sceneStageCount);

        ImGui.End();
    }

    private void DrawMissionTab()
    {
        ImGui.TextDisabled("Boundary Conditions");
        ImGui.Separator();

        ImGui.TextDisabled("Mission spec:");
        ImGui.SliderFloat("Thrust (N)", ref Thrust, 1000f, 20000f, "%.0f");
        ImGui.TextDisabled("Propellant: LOX/CH4");

        ImGui.Spacing();
        ImGui.TextDisabled("Printer constraints:");
        ImGui.SliderFloat("Min wall (mm)", ref MinWall, 0.3f, 1.5f, "%.2f");
        ImGui.SliderFloat("Max overhang", ref MaxOverhang, 30f, 60f, "%.0f");
        ImGui.SliderFloat("Voxel (mm)", ref VoxelSize, 0.4f, 2.5f, "%.2f");

        ImGui.Spacing();
        // Combo item order MUST match enum ChannelMode { MeshBased_v4=0, Implicit_v5=1, Routed_v5b=2 }
        // (AeroSpec.cs). The cast (ChannelMode)chMode relies on this 1:1 index mapping. Verified in sync.
        int chMode = (int)ChannelMode;
        if (ImGui.Combo("Channels", ref chMode, "MeshBased v4\0Implicit v5\0Routed v5b\0"))
            ChannelMode = (ChannelMode)chMode;

        ImGui.Spacing();
        ImGui.TextDisabled("Everything else is found by Search.");
    }

    private void DrawSearchTab()
    {
        if (SweepPanel == null)
        {
            ImGui.TextDisabled("SweepPanel not connected.");
            return;
        }

        SweepPanel.SyncPinnedInputs(Thrust, VoxelSize);
        SweepPanel.DrawContent();
    }

    private void DrawManualTab()
    {
        ImGui.TextDisabled("Direct parameter control");
        ImGui.Separator();

        ImGui.SliderFloat("Thrust (N)", ref Thrust, 1000f, 20000f, "%.0f");
        ImGui.SliderFloat("Pc (bar)", ref PcBar, 30f, 200f, "%.0f");
        ImGui.SliderFloat("O/F ratio", ref OF, 2.0f, 4.0f, "%.2f");
        ImGui.SliderFloat("CR", ref CR, 2.0f, 8.0f, "%.2f");
        ImGui.SliderFloat("L* (m)", ref Lstar, 0.2f, 1.5f, "%.2f");
        ImGui.SliderFloat("SF", ref SF, 1.2f, 2.5f, "%.2f");
        ImGui.SliderFloat("Twist", ref TwistTurns, 0.5f, 5.0f, "%.2f");
        ImGui.SliderFloat("Voxel (mm)", ref VoxelSize, 0.4f, 2.5f, "%.2f");

        // Same enum-order contract as the Mission tab Combo:
        // "MeshBased v4\0Implicit v5\0Routed v5b\0" == ChannelMode (0,1,2). Keep in sync.
        int chMode = (int)ChannelMode;
        if (ImGui.Combo("Channels", ref chMode, "MeshBased v4\0Implicit v5\0Routed v5b\0"))
            ChannelMode = (ChannelMode)chMode;
    }

    private void DrawBuildSection(PipelineController pipeline, int sceneStageCount)
    {
        // Build mode
        int mode = (int)BuildMode;
        ImGui.RadioButton("Atomic", ref mode, 0);
        ImGui.SameLine();
        ImGui.RadioButton("Z-slice", ref mode, 1);
        BuildMode = (BuildMode)mode;

        if (BuildMode == BuildMode.ZSliceSlabs)
            ImGui.SliderInt("Slabs", ref ZSliceCount, 8, 64);

        // Status
        string status = pipeline.IsRunning ? "BUILDING..." : (pipeline.StagesReceived > 0 ? "done" : "idle");
        if (pipeline.IsRunning)
            ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(0.3f, 1f, 0.3f, 1f));
        ImGui.TextUnformatted($"Status: {status}  ({pipeline.CurrentBuildElapsedSec:F1}s)  stages: {sceneStageCount}");
        if (pipeline.IsRunning)
            ImGui.PopStyleColor();

        if (pipeline.LastError != null)
        {
            ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(1f, 0.4f, 0.4f, 1f));
            ImGui.TextWrapped($"Error: {pipeline.LastError.Message}");
            ImGui.PopStyleColor();
        }

        // Buttons
        string label = pipeline.IsRunning ? "Cancel + Rebuild" : "Build Engine";
        if (ImGui.Button(label, new Vector2(160, 32)))
            RegenerateRequested = true;
        ImGui.SameLine();
        if (ImGui.Button("Frame", new Vector2(70, 32)))
            ResetCameraRequested = true;
    }

    // Pipeline panel removed — build status integrated into DrawBuildSection (tabs).

    private void DrawLogPanel(PipelineController pipeline)
    {
        ImGui.SetNextWindowPos(new Vector2(344, 12), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(540, 300), ImGuiCond.FirstUseEver);
        ImGui.Begin("Pipeline Log");

        var snapshot = pipeline.SnapshotLog();

        if (snapshot.Length == 0)
        {
            ImGui.TextDisabled("(no stages yet — waiting for build)");
        }
        else
        {
            if (ImGui.BeginTable("log", 4,
                ImGuiTableFlags.Borders | ImGuiTableFlags.RowBg | ImGuiTableFlags.ScrollY |
                ImGuiTableFlags.SizingStretchProp))
            {
                ImGui.TableSetupScrollFreeze(0, 1);
                ImGui.TableSetupColumn("t (s)", ImGuiTableColumnFlags.WidthFixed, 55f);
                ImGui.TableSetupColumn("Δt", ImGuiTableColumnFlags.WidthFixed, 50f);
                ImGui.TableSetupColumn("Stage", ImGuiTableColumnFlags.WidthFixed, 130f);
                ImGui.TableSetupColumn("Description", ImGuiTableColumnFlags.WidthStretch);
                ImGui.TableHeadersRow();

                foreach (var t in snapshot)
                {
                    ImGui.TableNextRow();
                    ImGui.TableSetColumnIndex(0);
                    ImGui.TextUnformatted($"{t.CumulativeSec:F2}");
                    ImGui.TableSetColumnIndex(1);
                    ImGui.TextUnformatted($"+{t.LapSec:F2}");
                    ImGui.TableSetColumnIndex(2);
                    ImGui.TextUnformatted(t.Stage.ToString());
                    ImGui.TableSetColumnIndex(3);
                    ImGui.TextWrapped(t.Description);
                }

                if (pipeline.IsRunning)
                    ImGui.SetScrollHereY(1f);

                ImGui.EndTable();
            }
        }

        ImGui.End();
    }

    private void DrawEngineStatePanel(Renderer renderer, StartupSequence startup, AeroSpec spec, Viability viability)
    {
        ImGui.SetNextWindowPos(new Vector2(344, 320), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(540, 460), ImGuiCond.FirstUseEver);
        ImGui.Begin("Engine State");

        // State indicator
        string stateText = renderer.State == EngineState.Materializing ? "MATERIALIZING" : "RUNNING";
        Vector4 stateColor = renderer.State == EngineState.Materializing
            ? new Vector4(0.3f, 0.8f, 1.0f, 1f)
            : new Vector4(1.0f, 0.4f, 0.1f, 1f);
        ImGui.PushStyleColor(ImGuiCol.Text, stateColor);
        ImGui.TextUnformatted($"State: {stateText}");
        ImGui.PopStyleColor();

        ImGui.TextUnformatted($"Throttle:   {startup.Throttle * 100f:F0}%");
        ImGui.ProgressBar(startup.Throttle, new Vector2(-1, 0));

        ImGui.Separator();

        // Live physics tickers — real throughflow values scaled by throttle where applicable
        float T = startup.Throttle;

        ImGui.TextUnformatted($"Chamber P:     {spec.Pc / 1e5f * (0.3f + 0.7f * T):F1}  bar");
        ImGui.TextUnformatted($"Chamber T:     {spec.Tc:F0} K  (gamma={spec.gamma:F3}, MW={spec.molWeight:F1})");
        ImGui.TextUnformatted($"Mass flow:     {spec.mDot * T:F3} kg/s");
        ImGui.TextUnformatted($"Thrust:        {spec.F_thrust * T / 1000f:F2} kN");
        ImGui.TextUnformatted($"Isp (SL/Vac):  {spec.Isp_SL:F1} / {spec.Isp_vac:F1} s");
        ImGui.TextUnformatted($"Exit velocity: {spec.Isp_SL * 9.81f:F0} m/s");
        ImGui.TextUnformatted($"Throat dia:    {spec.Dt * 1000f:F2} mm (A*={spec.At * 1e6f:F1} mm^2)");
        ImGui.TextUnformatted($"c*:            {spec.cStar:F0} m/s,  Cf={spec.Cf:F3}");
        ImGui.TextUnformatted($"Total length:  {spec.zTotal:F1} mm");

        ImGui.Separator();

        // Viability checklist
        ImGui.TextUnformatted("Viability:");
        ImGui.SameLine();
        Vector4 vColor = viability.IsViable
            ? new Vector4(0.3f, 1.0f, 0.3f, 1f)
            : new Vector4(1.0f, 0.4f, 0.3f, 1f);
        ImGui.PushStyleColor(ImGuiCol.Text, vColor);
        ImGui.TextUnformatted(viability.Headline);
        ImGui.PopStyleColor();

        if (viability.Checks != null && viability.Checks.Count > 0)
        {
            if (ImGui.BeginTable("checks", 3,
                ImGuiTableFlags.RowBg | ImGuiTableFlags.SizingStretchProp))
            {
                ImGui.TableSetupColumn("", ImGuiTableColumnFlags.WidthFixed, 20f);
                ImGui.TableSetupColumn("Check", ImGuiTableColumnFlags.WidthFixed, 180f);
                ImGui.TableSetupColumn("Value", ImGuiTableColumnFlags.WidthStretch);
                foreach (var c in viability.Checks)
                {
                    ImGui.TableNextRow();
                    ImGui.TableSetColumnIndex(0);
                    ImGui.PushStyleColor(ImGuiCol.Text, c.Passed
                        ? new Vector4(0.3f, 1f, 0.3f, 1f)
                        : new Vector4(1f, 0.4f, 0.3f, 1f));
                    ImGui.TextUnformatted(c.Passed ? "ok" : "!!");
                    ImGui.PopStyleColor();
                    ImGui.TableSetColumnIndex(1);
                    ImGui.TextUnformatted(c.Name);
                    ImGui.TableSetColumnIndex(2);
                    ImGui.TextWrapped(c.Detail);
                }
                ImGui.EndTable();
            }
        }

        ImGui.Separator();

        bool canIgnite = renderer.State == EngineState.Materializing && viability.IsViable;
        if (renderer.State == EngineState.Materializing)
        {
            if (!canIgnite) ImGui.BeginDisabled();
            ImGui.PushStyleColor(ImGuiCol.Button, new Vector4(0.8f, 0.3f, 0.1f, 1f));
            ImGui.PushStyleColor(ImGuiCol.ButtonHovered, new Vector4(1f, 0.5f, 0.2f, 1f));
            string label = canIgnite ? "IGNITE" : "CANNOT IGNITE";
            if (ImGui.Button(label, new Vector2(200, 44)) && canIgnite)
                IgniteRequested = true;
            ImGui.PopStyleColor(2);
            if (!canIgnite) ImGui.EndDisabled();

            if (!canIgnite)
            {
                ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(1f, 0.4f, 0.3f, 1f));
                ImGui.TextWrapped("Engine geometry or physics failed validation. Fix parameters and Regenerate.");
                ImGui.PopStyleColor();
            }
        }
        else
        {
            if (ImGui.Button("Shutdown", new Vector2(200, 44)))
                ShutdownRequested = true;
        }

        ImGui.End();
    }

    private static readonly (StageId id, string name)[] _layerNames =
    {
        (StageId.Channels,    "Cooling channels"),
        (StageId.Nozzle,      "Nozzle (convergent + throat)"),
        (StageId.Chamber,     "Combustion chamber"),
        (StageId.Dome,        "Injector dome"),
        (StageId.Spike,       "Aerospike cone"),
        (StageId.AllVoids,    "All voids combined"),
        (StageId.Shell,       "Engine walls"),
        (StageId.Collector,   "CH4 collector (hot out)"),
        (StageId.Inlet,       "CH4 inlet (cold in)"),
        (StageId.FeedPorts,   "Fuel / LOX / igniter ports"),
        (StageId.SpikeVanes,  "Spike structural ribs"),
        (StageId.TopFlange,   "Mounting flange"),
        (StageId.Final,       "Complete engine"),
    };

    private void DrawLayersPanel(Renderer renderer)
    {
        ImGui.SetNextWindowPos(new Vector2(12, 700), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(240, 300), ImGuiCond.FirstUseEver);
        ImGui.Begin("Layers");

        var loaded = renderer.GetLoadedStages();
        if (loaded.Count == 0)
        {
            ImGui.TextDisabled("(building...)");
        }
        else
        {
            foreach (var (id, name) in _layerNames)
            {
                if (!loaded.Contains(id)) continue;
                bool vis = renderer.IsStageVisible(id);
                if (ImGui.Checkbox(name, ref vis))
                    renderer.SetStageVisible(id, vis);
            }
        }

        ImGui.End();
    }

    private void DrawStylePanel(Renderer renderer)
    {
        ImGui.SetNextWindowPos(new Vector2(1268, 12), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(320, 280), ImGuiCond.FirstUseEver);
        ImGui.Begin("Style");

        Vector3 holo = renderer.HoloColor;
        if (ImGui.ColorEdit3("Hologram color", ref holo))
            renderer.HoloColor = holo;

        Vector3 metal = renderer.MetalColor;
        if (ImGui.ColorEdit3("Metal color", ref metal))
            renderer.MetalColor = metal;

        Vector3 bg = renderer.ClearColor;
        if (ImGui.ColorEdit3("Background", ref bg))
            renderer.ClearColor = bg;

        ImGui.Separator();
        ImGui.TextDisabled("Cross-section");
        float clipZ = renderer.ClipZ;
        if (ImGui.SliderFloat("Slice Z", ref clipZ, -10f, 160f, "%.0f mm"))
            renderer.ClipZ = clipZ;
        if (ImGui.Button("Reset slice", new Vector2(100, 0)))
            renderer.ClipZ = -999f;

        ImGui.Separator();
        ImGui.TextDisabled("Shader hot reload");
        bool ok = renderer.LastShaderReloadMessage == "OK" || renderer.LastShaderReloadMessage == "initial load";
        ImGui.PushStyleColor(ImGuiCol.Text, ok
            ? new Vector4(0.3f, 1f, 0.3f, 1f)
            : new Vector4(1f, 0.4f, 0.4f, 1f));
        ImGui.TextUnformatted($"{renderer.LastShaderReloadTime:HH:mm:ss}  {renderer.LastShaderReloadMessage}");
        ImGui.PopStyleColor();
        ImGui.TextDisabled("Save Viewer/Shaders/*.frag");
        ImGui.TextDisabled("-> auto recompile");

        ImGui.End();
    }
}
