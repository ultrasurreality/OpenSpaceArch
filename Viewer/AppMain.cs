// AppMain.cs — entry point for the cinematic viewer with simulation.

using System.Numerics;
using ImGuiNET;
using PicoGK;
using Silk.NET.Input;
using Silk.NET.Maths;
using Silk.NET.OpenGL.Extensions.ImGui;
using OpenSpaceArch.Engine;
using OpenSpaceArch.Viewer.Pipeline;
using OpenSpaceArch.Viewer.Rendering;
using OpenSpaceArch.Viewer.Simulation;
using OpenSpaceArch.Viewer.UI;
using OpenSpaceArch.Viewer.Window;

namespace OpenSpaceArch.Viewer;

public static class AppMain
{
    private static AppWindow _app = null!;
    private static Renderer _renderer = null!;
    private static PipelineController _pipeline = null!;
    private static ControlPanel _controlPanel = null!;
    private static ConstraintsPanel _constraintsPanel = null!;
    private static ShaderWatcher? _shaderWatcher;
    private static ImGuiController? _imgui;
    private static IInputContext? _input;
    private static Vector2 _lastMouse;
    private static bool _leftDown, _rightDown;
    private static float _stageClock;
    private static int _sceneStageCount;
    private static bool _initialFrameDone;
    private static AeroSpec _lastBuiltSpec = new();
    private static Viability _viability;          // post-build (final stage)
    private static Viability _liveViability;      // updated every slider change (phase 1)
    private static GasFlowProfile _flow = new();

    // Simulation
    private static HeatProfile? _heatProfile;
    private static ParticleSystem? _plume;
    private static ChamberGlow? _glow;
    private static StartupSequence _startup = new();
    private static ShaderProgram? _plumeProgram;
    private static GpuProfileTextures? _profiles;
    private static SdfRaymarchPass? _sdfPreview;
    private static AeroSpec _previewSpec = new();
    private static int _previewParamHash;

    private static string ResolveShaderDir()
    {
        string dir = AppContext.BaseDirectory;
        for (int i = 0; i < 6; i++)
        {
            string candidate = Path.Combine(dir, "Viewer", "Shaders");
            if (Directory.Exists(candidate) && File.Exists(Path.Combine(dir, "OpenSpaceArch.csproj")))
                return candidate;
            var parent = Directory.GetParent(dir);
            if (parent == null) break;
            dir = parent.FullName;
        }
        return Path.Combine(AppContext.BaseDirectory, "Shaders");
    }

    public static void Run()
    {
        Library.InitHeadless(voxelSizeMM: 1.0f);

        _app = AppWindow.Create(1600, 900, "OpenSpaceArch — Cinematic Viewer + Simulation");
        _app.OnLoad += HandleLoad;
        _app.OnRender += HandleRender;
        _app.OnUpdate += HandleUpdate;
        _app.OnResize += HandleResize;
        _app.OnClose += HandleClose;

        _app.Run();

        _shaderWatcher?.Dispose();
        _sdfPreview?.Dispose();
        _profiles?.Dispose();
        _plumeProgram?.Dispose();
        _plume?.Dispose();
        _glow?.Dispose();
        _heatProfile?.Dispose();
        _renderer?.Dispose();
        _input?.Dispose();
        Library.Shutdown();
    }

    private static void HandleLoad(Silk.NET.OpenGL.GL gl)
    {
        string shaderDir = ResolveShaderDir();
        Console.WriteLine($"[Viewer] Shader directory: {shaderDir}");

        string vertSrc = File.ReadAllText(Path.Combine(shaderDir, "mesh.vert"));
        string hologramFragSrc = File.ReadAllText(Path.Combine(shaderDir, "hologram.frag"));
        string runningFragSrc = File.ReadAllText(Path.Combine(shaderDir, "engine_running.frag"));
        string plumeVertSrc = File.ReadAllText(Path.Combine(shaderDir, "plume.vert"));
        string plumeFragSrc = File.ReadAllText(Path.Combine(shaderDir, "plume.frag"));
        string glowVertSrc = File.ReadAllText(Path.Combine(shaderDir, "glow.vert"));
        string glowFragSrc = File.ReadAllText(Path.Combine(shaderDir, "glow.frag"));
        string sdfVertSrc = File.ReadAllText(Path.Combine(shaderDir, "sdf_raymarch.vert"));
        string sdfFragSrc = File.ReadAllText(Path.Combine(shaderDir, "sdf_raymarch.frag"));

        _renderer = new Renderer(gl, vertSrc, hologramFragSrc, vertSrc, runningFragSrc);

        _plumeProgram = new ShaderProgram(gl, plumeVertSrc, plumeFragSrc);
        _renderer.SetPlumeProgram(_plumeProgram);

        _plume = new ParticleSystem(gl);
        _glow = new ChamberGlow(gl, glowVertSrc, glowFragSrc);
        _heatProfile = new HeatProfile(gl);
        _profiles = new GpuProfileTextures(gl);
        _sdfPreview = new SdfRaymarchPass(gl, sdfVertSrc, sdfFragSrc);

        _renderer.Plume = _plume;
        _renderer.Glow = _glow;
        _renderer.HeatProfile = _heatProfile;
        _renderer.Profiles = _profiles;
        _renderer.SdfPreview = _sdfPreview;
        _renderer.Startup = _startup;

        // Compute initial physics + upload profile texture so the SDF preview
        // is visible immediately when the window opens, before any PicoGK build runs.
        UpdatePreviewProfiles(_controlPanel?.BuildSpec() ?? new AeroSpec(), force: true);

        try
        {
            _shaderWatcher = new ShaderWatcher(shaderDir);
            Console.WriteLine("[Viewer] Shader hot reload enabled");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[Viewer] Shader watcher disabled: {ex.Message}");
        }

        _pipeline = new PipelineController();
        _controlPanel = new ControlPanel();
        _constraintsPanel = new ConstraintsPanel();

        _renderer.Camera.Target = new Vector3(0, 0, 100f);
        _renderer.Camera.Distance = 500f;

        _input = _app.RawWindow.CreateInput();
        _imgui = new ImGuiController(gl, _app.RawWindow, _input);

        foreach (var kb in _input.Keyboards)
            kb.KeyDown += (k, key, _) => { if (key == Key.Escape) _app.Close(); };
        foreach (var mouse in _input.Mice)
        {
            mouse.MouseDown += (m, btn) => {
                if (ImGui.GetIO().WantCaptureMouse) return;
                if (btn == MouseButton.Left) _leftDown = true;
                if (btn == MouseButton.Right) _rightDown = true;
                _lastMouse = new Vector2(m.Position.X, m.Position.Y);
            };
            mouse.MouseUp += (_, btn) => {
                if (btn == MouseButton.Left) _leftDown = false;
                if (btn == MouseButton.Right) _rightDown = false;
            };
            mouse.MouseMove += (m, pos) => {
                var p = new Vector2(pos.X, pos.Y);
                var delta = p - _lastMouse;
                if (!ImGui.GetIO().WantCaptureMouse)
                {
                    if (_leftDown)
                        _renderer.Camera.Orbit(-delta.X * 0.008f, delta.Y * 0.008f);
                    else if (_rightDown)
                        _renderer.Camera.Pan(new Vector2(-delta.X, delta.Y));
                }
                _lastMouse = p;
            };
            mouse.Scroll += (_, s) => {
                if (!ImGui.GetIO().WantCaptureMouse)
                    _renderer.Camera.Zoom(s.Y);
            };
        }

        _pipeline.Regenerate(_controlPanel.BuildSpec(), _controlPanel.BuildMode, _controlPanel.ZSliceCount);

        // Re-upload preview profiles now that the panel exists
        UpdatePreviewProfiles(_controlPanel.BuildSpec(), force: true);

        Console.WriteLine("[Viewer] Ready. ImGui controls active. Esc = quit.");
    }

    /// <summary>
    /// Recomputes physics + sizing for the spec and uploads new profile textures
    /// for the GPU SDF raymarcher. Skips when the parameter hash matches the
    /// previous upload to avoid wasting work on every frame.
    /// </summary>
    private static void UpdatePreviewProfiles(AeroSpec spec, bool force = false)
    {
        if (_profiles == null) return;

        int hash = HashCode.Combine(spec.F_thrust, spec.Pc, spec.OF_ratio, spec.voxelSize, (int)spec.channelMode);
        if (!force && hash == _previewParamHash) return;
        _previewParamHash = hash;

        try
        {
            Thermochemistry.Compute(spec);
            ChamberSizing.Compute(spec);
            HeatTransfer.Compute(spec);
            _profiles.Upload(spec);
            _previewSpec = spec;

            // Phase 1: live constraint validation. Reactive to every slider
            // change because UpdatePreviewProfiles is gated by _previewParamHash
            // — runs only when a parameter actually changed. The ConstraintsPanel
            // reads _liveViability on each HandleRender tick.
            _liveViability = EngineValidator.Check(spec);
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[SDF preview] physics failed: {ex.Message}");
        }
    }

    private static void HandleUpdate(double deltaTime)
    {
        float dt = (float)deltaTime;
        _imgui?.Update(dt);

        // Shader hot reload
        if (_shaderWatcher != null && _shaderWatcher.ConsumePending())
        {
            try
            {
                string shaderDir = _shaderWatcher.ShaderDir;
                string vert = File.ReadAllText(Path.Combine(shaderDir, "mesh.vert"));
                string holoFrag = File.ReadAllText(Path.Combine(shaderDir, "hologram.frag"));
                string runFrag = File.ReadAllText(Path.Combine(shaderDir, "engine_running.frag"));
                _renderer.ReloadShaders(vert, holoFrag, vert, runFrag);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[ShaderReload] Read error: {ex.Message}");
            }
        }

        // Drain pipeline
        _pipeline.DrainQueue(stage =>
        {
            Console.WriteLine($"[Viewer] Uploading stage: {stage.Stage} ({stage.Mesh.nVertexCount()} verts)");
            var gpuMesh = new GpuMesh(_app.Gl, stage.Mesh);
            float delay = _stageClock;
            float duration = 1.5f;
            _renderer.AddMesh(gpuMesh, stage.Stage, revealDelay: delay, revealDuration: duration);
            _stageClock += 0.6f;
            _sceneStageCount++;

            if (!_initialFrameDone)
            {
                _renderer.Camera.Frame(gpuMesh.Bounds);
                _initialFrameDone = true;
            }

            if (stage.Stage == StageId.Final)
            {
                // When final stage arrives, cache spec for simulation + compute flow profile + upload heat profile
                _lastBuiltSpec = _controlPanel.BuildSpec();
                Thermochemistry.Compute(_lastBuiltSpec);
                ChamberSizing.Compute(_lastBuiltSpec);
                HeatTransfer.Compute(_lastBuiltSpec);

                _flow.Compute(_lastBuiltSpec);
                _heatProfile?.Upload(_lastBuiltSpec, _flow);
                if (_plume != null)
                {
                    _plume.Spec = _lastBuiltSpec;
                    _plume.Flow = _flow;
                }
                _renderer.Spec = _lastBuiltSpec;

                // Run viability validator
                _viability = EngineValidator.Check(_lastBuiltSpec);
                Console.WriteLine($"[Viability] {_viability.Headline}");
                foreach (var c in _viability.Checks)
                    Console.WriteLine($"  {(c.Passed ? "[ok]" : "[FAIL]")} {c.Name,-28} {c.Detail}");

                // Report flow summary
                Console.WriteLine($"[GasFlow] Throat Mach=1 at z={_flow.ZThroat:F1}, exit M={_flow.Mach[_flow.N - 1]:F2}, u_exit={_flow.U_ms[_flow.N - 1]:F0} m/s");
            }
        });

        // Live preview: re-upload profile textures whenever sliders change
        UpdatePreviewProfiles(_controlPanel.BuildSpec());

        // Startup sequence drives throttle
        _startup.Update(dt);
        if (_plume != null) _plume.Throttle = _renderer.State == EngineState.Running ? _startup.Throttle : 0f;
        _plume?.Update(dt);

        // UI requests
        if (_controlPanel.RegenerateRequested)
        {
            _controlPanel.RegenerateRequested = false;
            _renderer.ClearScene();
            _stageClock = 0f;
            _sceneStageCount = 0;
            _initialFrameDone = false;
            _renderer.State = EngineState.Materializing;
            _startup.Reset();
            _pipeline.Regenerate(_controlPanel.BuildSpec(), _controlPanel.BuildMode, _controlPanel.ZSliceCount);
        }
        if (_controlPanel.ResetCameraRequested)
        {
            _controlPanel.ResetCameraRequested = false;
            _renderer.Camera.Frame(_renderer.SceneBounds());
        }
        if (_controlPanel.IgniteRequested)
        {
            _controlPanel.IgniteRequested = false;
            if (_viability.IsViable)
            {
                _renderer.State = EngineState.Running;
                _startup.Ignite();
                Console.WriteLine("[Viewer] IGNITE");
            }
            else
            {
                Console.WriteLine($"[Viewer] IGNITE REFUSED: {_viability.Headline}");
            }
        }
        if (_controlPanel.ShutdownRequested)
        {
            _controlPanel.ShutdownRequested = false;
            _renderer.State = EngineState.Materializing;
            _startup.Reset();
        }
    }

    private static void HandleRender(double deltaTime, Silk.NET.OpenGL.GL gl)
    {
        _renderer.RenderFrame(deltaTime, _app.Size.X, _app.Size.Y);

        _controlPanel.Draw(_pipeline, _renderer, _sceneStageCount, _lastBuiltSpec, _startup, _viability);
        _constraintsPanel.Draw(_liveViability);
        _imgui?.Render();
    }

    private static void HandleResize(Vector2D<int> size) { }
    private static void HandleClose() => Console.WriteLine("[Viewer] closing...");
}
