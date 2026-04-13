// SweepPanel.cs — Phase 2 design-space visualization.
//
// A dedicated ImGui panel that drives the OuterSweep bounded random
// search and renders the resulting design cloud as a scatter plot
// (Isp vs mass, colored by weighted score). The user tunes weights in
// real time and sees the best engine re-rank instantly. Clicking "Apply
// Best" copies the winning AeroSpec back to ControlPanel and triggers
// the normal regenerate flow so the 3D scene rebuilds with the sweep
// winner.
//
// Architecture context: this is the "user-visible half" of the two-layer
// CSP architecture (the other half is the existing ConstraintsPanel
// shipped in Phase 1). The sweep itself runs in Engine.CSP.OuterSweep
// — this file is UI only.
//
// See ARCHITECTURE.md, Phase 2.

using System.Diagnostics;
using System.Numerics;
using System.Text.Json;
using ImGuiNET;
using OpenSpaceArch.Engine;
using OpenSpaceArch.Engine.CSP;

namespace OpenSpaceArch.Viewer.UI;

public sealed class SweepPanel
{
    private readonly OuterSweep.Config _cfg = new();
    private readonly ScoringWeights _weights = ScoringWeights.Default();
    private List<SweepResult> _results = new();

    // Sample count presets exposed in a combo
    private static readonly int[] _nPresets = { 100, 500, 1000, 2500, 5000 };
    private int _nPresetIndex = 1; // 500 default

    private int _seedInput = 42;
    private long _lastSweepMs;
    private bool _uiFoldConfig = true;
    private bool _uiFoldWeights = true;

    /// <summary>
    /// Wired in AppMain to <c>_controlPanel.ApplyFromSpec</c>. When the user
    /// clicks "Apply Best" the panel invokes this with the winning spec,
    /// which updates the sliders and flags RegenerateRequested.
    /// </summary>
    public Action<AeroSpec>? OnApplyWinner;

    /// <summary>
    /// Sets the pinned thrust/voxel for the sweep based on the current
    /// ControlPanel state. Called every frame from AppMain so the sweep
    /// always reflects the currently-selected mission thrust.
    /// </summary>
    public void SyncPinnedInputs(float fixedThrust, float fixedVoxel)
    {
        _cfg.FixedThrust = fixedThrust;
        _cfg.FixedVoxel = fixedVoxel;
    }

    public void Draw()
    {
        ImGui.SetNextWindowPos(new Vector2(1268, 600), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(332, 296), ImGuiCond.FirstUseEver);
        ImGui.Begin("Design Space Sweep");
        DrawContent();
        ImGui.End();
    }

    /// <summary>Draws all sweep content without Begin/End — for embedding in another panel.</summary>
    public void DrawContent()
    {
        DrawHeader();
        DrawConfigBlock();
        DrawWeightsBlock();
        DrawStatsLine();
        DrawScatterPlot();
        DrawTopList();
    }

    // ─────────────────────────────────────────────────────────────
    // Header: Run / Apply buttons
    // ─────────────────────────────────────────────────────────────

    private void DrawHeader()
    {
        // Pin thrust readout — so user knows what the sweep is varying around
        ImGui.TextDisabled($"Fixed: F={_cfg.FixedThrust / 1000f:F1}kN · voxel={_cfg.FixedVoxel:F2}mm");

        if (ImGui.Button("Quick (500)", new Vector2(80, 24)))
            RunSweep();

        ImGui.SameLine();
        if (ImGui.Button("pymoo NSGA-III", new Vector2(110, 24)))
            RunPymoo();

        ImGui.SameLine();

        bool hasResults = _results.Count > 0;
        if (!hasResults) ImGui.BeginDisabled();
        if (ImGui.Button("Apply Best", new Vector2(85, 24)))
            ApplyBest();
        if (!hasResults) ImGui.EndDisabled();

        ImGui.SameLine();
        if (hasResults)
            ImGui.TextDisabled($"{_lastSweepMs}ms {_results.Count} pts");
    }

    // ─────────────────────────────────────────────────────────────
    // Config block: N, seed, ranges summary
    // ─────────────────────────────────────────────────────────────

    private void DrawConfigBlock()
    {
        if (!ImGui.CollapsingHeader("Config", ref _uiFoldConfig, ImGuiTreeNodeFlags.DefaultOpen))
            return;

        if (ImGui.Combo("Samples", ref _nPresetIndex,
                        "100\0500\01000\02500\05000\0"))
            _cfg.NumSamples = _nPresets[_nPresetIndex];

        if (ImGui.InputInt("Seed", ref _seedInput))
            _cfg.MasterSeed = _seedInput;

        ImGui.TextDisabled("Ranges:");
        ImGui.TextDisabled(_cfg.Vars.Summary());
    }

    // ─────────────────────────────────────────────────────────────
    // Weights block: 5 sliders + sum indicator
    // ─────────────────────────────────────────────────────────────

    private void DrawWeightsBlock()
    {
        if (!ImGui.CollapsingHeader("Weights", ref _uiFoldWeights, ImGuiTreeNodeFlags.DefaultOpen))
            return;

        ImGui.SliderFloat("Isp", ref _weights.wIsp, 0f, 1f, "%.2f");
        ImGui.SliderFloat("T/W", ref _weights.wTW, 0f, 1f, "%.2f");
        ImGui.SliderFloat("Thermal", ref _weights.wThermal, 0f, 1f, "%.2f");
        ImGui.SliderFloat("Ch.fit", ref _weights.wChannelFit, 0f, 1f, "%.2f");
        ImGui.SliderFloat("Compact", ref _weights.wCompactness, 0f, 1f, "%.2f");

        ImGui.TextDisabled($"Σ positive = {_weights.PositiveSum:F2}   (max score before penalties)");
    }

    // ─────────────────────────────────────────────────────────────
    // Stats line: counts + best score
    // ─────────────────────────────────────────────────────────────

    private void DrawStatsLine()
    {
        if (_results.Count == 0)
        {
            ImGui.TextDisabled("no sweep yet — press Run Sweep.");
            return;
        }

        int total = _results.Count;
        int valid = 0;
        float bestScore = 0f;
        foreach (var r in _results)
        {
            if (r.Valid) valid++;
            float s = r.ComputeScore(_weights);
            if (s > bestScore) bestScore = s;
        }

        ImGui.Text($"N={total} · valid {valid} ({100f * valid / total:F0}%) · best {bestScore:F3}");
    }

    // ─────────────────────────────────────────────────────────────
    // Scatter plot: Isp (x) vs Mass (y), colored by score
    // ─────────────────────────────────────────────────────────────

    private void DrawScatterPlot()
    {
        // Fixed axis ranges so the plot is stable across sweeps
        const float ispMin = 250f;
        const float ispMax = 340f;
        const float massMin = 0.3f;
        const float massMax = 4.0f;
        const float plotHeight = 120f;

        var drawList = ImGui.GetWindowDrawList();
        var origin = ImGui.GetCursorScreenPos();
        float plotWidth = ImGui.GetContentRegionAvail().X;

        // Plot background frame
        uint bg = ImGui.GetColorU32(new Vector4(0.08f, 0.09f, 0.12f, 1f));
        uint frameColor = ImGui.GetColorU32(new Vector4(0.3f, 0.32f, 0.38f, 1f));
        drawList.AddRectFilled(
            origin,
            new Vector2(origin.X + plotWidth, origin.Y + plotHeight),
            bg);
        drawList.AddRect(
            origin,
            new Vector2(origin.X + plotWidth, origin.Y + plotHeight),
            frameColor);

        // Axis labels
        uint labelColor = ImGui.GetColorU32(new Vector4(0.55f, 0.58f, 0.65f, 1f));
        drawList.AddText(new Vector2(origin.X + 4, origin.Y + 2), labelColor, "mass↑ / Isp→");
        drawList.AddText(new Vector2(origin.X + 2, origin.Y + plotHeight - 14), labelColor, $"{ispMin:F0}");
        drawList.AddText(new Vector2(origin.X + plotWidth - 24, origin.Y + plotHeight - 14), labelColor, $"{ispMax:F0}");

        if (_results.Count == 0)
        {
            drawList.AddText(
                new Vector2(origin.X + plotWidth * 0.5f - 32, origin.Y + plotHeight * 0.5f - 6),
                labelColor,
                "(no data)");
            ImGui.Dummy(new Vector2(plotWidth, plotHeight));
            return;
        }

        // Find best sample by current weights so we can highlight it
        int bestIdx = -1;
        float bestScore = -1f;
        for (int i = 0; i < _results.Count; i++)
        {
            if (!_results[i].Valid) continue;
            float s = _results[i].ComputeScore(_weights);
            if (s > bestScore) { bestScore = s; bestIdx = i; }
        }

        // Draw each sample
        for (int i = 0; i < _results.Count; i++)
        {
            var r = _results[i];
            if (r.Isp_SL <= 0f || r.MassEstimate_kg <= 0f) continue;

            float xFrac = Math.Clamp((r.Isp_SL - ispMin) / (ispMax - ispMin), 0f, 1f);
            float yFrac = Math.Clamp((r.MassEstimate_kg - massMin) / (massMax - massMin), 0f, 1f);
            float px = origin.X + xFrac * plotWidth;
            // Y inverted: small mass = higher on screen
            float py = origin.Y + plotHeight - yFrac * plotHeight;

            uint dotColor;
            if (!r.Valid)
            {
                dotColor = ImGui.GetColorU32(new Vector4(0.45f, 0.46f, 0.5f, 0.35f));
                drawList.AddCircleFilled(new Vector2(px, py), 2f, dotColor);
            }
            else
            {
                float score = r.ComputeScore(_weights);
                dotColor = ScoreToColor(score);
                drawList.AddCircleFilled(new Vector2(px, py), 2.5f, dotColor);
            }
        }

        // Best sample highlight — draw LAST so it's on top
        if (bestIdx >= 0)
        {
            var r = _results[bestIdx];
            float xFrac = Math.Clamp((r.Isp_SL - ispMin) / (ispMax - ispMin), 0f, 1f);
            float yFrac = Math.Clamp((r.MassEstimate_kg - massMin) / (massMax - massMin), 0f, 1f);
            float px = origin.X + xFrac * plotWidth;
            float py = origin.Y + plotHeight - yFrac * plotHeight;

            uint ring = ImGui.GetColorU32(new Vector4(1f, 0.95f, 0.3f, 1f));
            drawList.AddCircle(new Vector2(px, py), 5.5f, ring, 16, 2f);
        }

        // Advance cursor past the plot so subsequent widgets appear below
        ImGui.Dummy(new Vector2(plotWidth, plotHeight));
    }

    /// <summary>
    /// Score → HSV gradient: 0 = red, 0.5 = yellow, 1 = green.
    /// </summary>
    private static uint ScoreToColor(float score)
    {
        score = Math.Clamp(score, 0f, 1f);
        // Hue 0° (red) → 120° (green), so HSV gradient maps linearly
        float h = score * (120f / 360f); // [0, 0.333]
        float s = 0.85f;
        float v = 0.95f;

        ImGui.ColorConvertHSVtoRGB(h, s, v, out float r, out float g, out float b);
        return ImGui.GetColorU32(new Vector4(r, g, b, 1f));
    }

    // ─────────────────────────────────────────────────────────────
    // Top-5 list by score
    // ─────────────────────────────────────────────────────────────

    private void DrawTopList()
    {
        if (_results.Count == 0) return;

        // Cheap in-place copy + sort by current score (descending).
        // N=500 × log2(500) × a few compares = ~5 μs. No problem.
        var scored = new List<(SweepResult Result, float Score)>(_results.Count);
        foreach (var r in _results)
            if (r.Valid)
                scored.Add((r, r.ComputeScore(_weights)));

        scored.Sort((a, b) => b.Score.CompareTo(a.Score));

        ImGui.Separator();
        ImGui.TextDisabled($"Top {Math.Min(5, scored.Count)} by score:");

        int take = Math.Min(5, scored.Count);
        for (int i = 0; i < take; i++)
        {
            var (r, score) = scored[i];
            var color = ScoreToColor(score);
            ImGui.PushStyleColor(ImGuiCol.Text, color);
            ImGui.TextUnformatted($"#{i + 1}");
            ImGui.PopStyleColor();
            ImGui.SameLine();
            ImGui.TextUnformatted(
                $" s={score:F2} " +
                $"Isp={r.Isp_SL:F0} " +
                $"T/W={r.TWRatio:F0} " +
                $"m={r.MassEstimate_kg:F2}kg");
        }

        if (scored.Count == 0)
            ImGui.TextDisabled("(no valid samples — try widening ranges)");
    }

    // ─────────────────────────────────────────────────────────────
    // Actions: run / apply
    // ─────────────────────────────────────────────────────────────

    private void RunSweep()
    {
        var sw = Stopwatch.StartNew();
        _results = OuterSweep.Run(_cfg);
        sw.Stop();
        _lastSweepMs = sw.ElapsedMilliseconds;
    }

    private void RunPymoo()
    {
        try
        {
            string scriptDir = Path.Combine(AppContext.BaseDirectory);
            // Walk up to find project root
            string root = AppContext.BaseDirectory;
            for (int i = 0; i < 6; i++)
            {
                if (File.Exists(Path.Combine(root, "OpenSpaceArch.csproj"))) break;
                var p = Directory.GetParent(root);
                if (p == null) break;
                root = p.FullName;
            }
            string script = Path.Combine(root, "Engine", "CSP", "PymooSweep.py");
            string jsonPath = Path.Combine(root, "pareto_results.json");

            var psi = new ProcessStartInfo("python", $"\"{script}\" 200 500 \"{jsonPath}\"")
            {
                UseShellExecute = false,
                RedirectStandardError = true,
                CreateNoWindow = true
            };

            var sw = Stopwatch.StartNew();
            var proc = Process.Start(psi);
            proc?.WaitForExit(60000);
            sw.Stop();
            _lastSweepMs = sw.ElapsedMilliseconds;

            if (File.Exists(jsonPath))
                LoadParetoJson(jsonPath);
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[pymoo] Error: {ex.Message}");
        }
    }

    private void LoadParetoJson(string path)
    {
        try
        {
            var json = File.ReadAllText(path);
            using var doc = JsonDocument.Parse(json);
            var solutions = doc.RootElement.GetProperty("solutions");

            _results.Clear();
            foreach (var sol in solutions.EnumerateArray())
            {
                var spec = new AeroSpec
                {
                    F_thrust = _cfg.FixedThrust,
                    Pc = (float)sol.GetProperty("Pc").GetDouble(),
                    OF_ratio = (float)sol.GetProperty("OF").GetDouble(),
                    CR = (float)sol.GetProperty("CR").GetDouble(),
                    Lstar = (float)sol.GetProperty("Lstar").GetDouble(),
                    SF = (float)sol.GetProperty("SF").GetDouble(),
                    channelTwistTurns = (float)sol.GetProperty("Twist").GetDouble(),
                    voxelSize = _cfg.FixedVoxel,
                };

                _results.Add(new SweepResult(
                    Seed: 0,
                    Spec: spec,
                    Valid: true,
                    FailReasons: "OK",
                    Isp_SL: (float)sol.GetProperty("Isp").GetDouble(),
                    TWRatio: _cfg.FixedThrust / ((float)sol.GetProperty("mass").GetDouble() * 9.81f),
                    MassEstimate_kg: (float)sol.GetProperty("mass").GetDouble(),
                    SigmaThermal_MPa: (float)sol.GetProperty("stress_MPa").GetDouble(),
                    SpatialConflicts: 0,
                    zTotal_mm: 0f,
                    qThroat_MW: 0f,
                    ChRadiusMin_mm: 0f,
                    NChannels: 0));
            }

            Console.WriteLine($"[pymoo] Loaded {_results.Count} Pareto solutions from {path}");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[pymoo] JSON load error: {ex.Message}");
        }
    }

    private void ApplyBest()
    {
        if (_results.Count == 0 || OnApplyWinner == null) return;

        SweepResult best = default;
        float bestScore = -1f;
        bool found = false;
        foreach (var r in _results)
        {
            if (!r.Valid) continue;
            float s = r.ComputeScore(_weights);
            if (s > bestScore) { bestScore = s; best = r; found = true; }
        }

        if (found)
            OnApplyWinner(best.Spec);
    }
}
