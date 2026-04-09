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
    // Editable AeroSpec mirror
    public float Thrust = 5000f;          // N
    public float PcBar = 110f;            // bar
    public float OF = 3.2f;
    public float VoxelSize = 1.0f;        // mm
    public ChannelMode ChannelMode = ChannelMode.Routed_v5b;

    public BuildMode BuildMode = BuildMode.ZSliceSlabs;
    public int ZSliceCount = 24;

    public bool RegenerateRequested;
    public bool ResetCameraRequested;
    public bool IgniteRequested;
    public bool ShutdownRequested;

    public void Draw(PipelineController pipeline, Renderer renderer, int sceneStageCount,
                     AeroSpec lastBuiltSpec, StartupSequence startup, Viability viability)
    {
        DrawParameterPanel();
        DrawBuildModePanel();
        DrawPipelinePanel(pipeline, sceneStageCount);
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
            voxelSize = VoxelSize,
            channelMode = ChannelMode,
        };
    }

    private void DrawParameterPanel()
    {
        ImGui.SetNextWindowPos(new Vector2(12, 12), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(320, 220), ImGuiCond.FirstUseEver);
        ImGui.Begin("Parameters");

        ImGui.SliderFloat("Thrust (N)", ref Thrust, 1000f, 20000f, "%.0f");
        ImGui.SliderFloat("Pc (bar)", ref PcBar, 30f, 200f, "%.0f");
        ImGui.SliderFloat("O/F ratio", ref OF, 2.0f, 4.0f, "%.2f");
        ImGui.SliderFloat("Voxel size (mm)", ref VoxelSize, 0.4f, 2.5f, "%.2f");

        int chMode = (int)ChannelMode;
        if (ImGui.Combo("Channel mode", ref chMode, "MeshBased v4\0Implicit v5\0Routed v5b\0"))
            ChannelMode = (ChannelMode)chMode;

        ImGui.End();
    }

    private void DrawBuildModePanel()
    {
        ImGui.SetNextWindowPos(new Vector2(12, 240), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(320, 140), ImGuiCond.FirstUseEver);
        ImGui.Begin("Build Mode");

        int mode = (int)BuildMode;
        ImGui.RadioButton("Atomic (one core voxelization)", ref mode, 0);
        ImGui.RadioButton("Z-slice slabs (live progress)", ref mode, 1);
        BuildMode = (BuildMode)mode;

        if (BuildMode == BuildMode.ZSliceSlabs)
        {
            ImGui.SliderInt("Slab count", ref ZSliceCount, 8, 64);
        }

        ImGui.TextDisabled("Z-slice = visible accumulation");
        ImGui.TextDisabled("Atomic  = faster, single block");

        ImGui.End();
    }

    private void DrawPipelinePanel(PipelineController pipeline, int sceneStageCount)
    {
        ImGui.SetNextWindowPos(new Vector2(12, 388), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(320, 240), ImGuiCond.FirstUseEver);
        ImGui.Begin("Pipeline");

        string status = pipeline.IsRunning ? "BUILDING..." : (pipeline.StagesReceived > 0 ? "done" : "idle");
        if (pipeline.IsRunning)
            ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(0.3f, 1f, 0.3f, 1f));
        ImGui.TextUnformatted($"Status: {status}");
        if (pipeline.IsRunning)
            ImGui.PopStyleColor();

        float elapsed = pipeline.CurrentBuildElapsedSec;
        ImGui.TextUnformatted($"Elapsed:          {elapsed:F2}s");
        ImGui.TextUnformatted($"Stages received:  {pipeline.StagesReceived}");
        ImGui.TextUnformatted($"Stages on screen: {sceneStageCount}");

        if (pipeline.LastError != null)
        {
            ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(1f, 0.4f, 0.4f, 1f));
            ImGui.TextWrapped($"Error: {pipeline.LastError.Message}");
            ImGui.PopStyleColor();
        }

        ImGui.Separator();

        string label = pipeline.IsRunning ? "Cancel + Regenerate" : "Regenerate";
        if (ImGui.Button(label, new Vector2(160, 32)))
            RegenerateRequested = true;

        ImGui.SameLine();
        if (ImGui.Button("Frame camera", new Vector2(120, 32)))
            ResetCameraRequested = true;

        ImGui.End();
    }

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
