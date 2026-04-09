// Renderer.cs — two-state frame render loop.
//
// Materializing state: hologram dissolve reveal per stage (Phase 2).
// Running state:       wall heat map + coolant flow + plume particles + chamber glow.
//
// Items are tagged with their StageId so the running-state pass can pick the
// right shader (wall vs channel).

using System.Numerics;
using OpenSpaceArch.Engine;
using OpenSpaceArch.Viewer.Simulation;
using Silk.NET.OpenGL;

namespace OpenSpaceArch.Viewer.Rendering;

public sealed class Renderer : IDisposable
{
    private readonly GL _gl;
    private ShaderProgram _hologram;
    private ShaderProgram _engineRunning;
    private readonly List<RenderItem> _items = new();
    private float _time;

    public Camera Camera { get; } = new();

    public Vector3 ClearColor = new(0.02f, 0.04f, 0.08f);
    public Vector3 HoloColor = new(0.25f, 0.75f, 1.0f);   // Iron Man blue
    public Vector3 MetalColor = new(0.85f, 0.55f, 0.25f); // copper

    public EngineState State { get; set; } = EngineState.Materializing;

    // Simulation subsystems — owned externally but invoked from here each frame
    public HeatProfile? HeatProfile;
    public ParticleSystem? Plume;
    public ChamberGlow? Glow;
    public StartupSequence? Startup;
    public AeroSpec? Spec;
    public GpuProfileTextures? Profiles;
    public SdfRaymarchPass? SdfPreview;

    public string LastShaderReloadMessage { get; private set; } = "initial load";
    public DateTime LastShaderReloadTime { get; private set; } = DateTime.Now;

    public Renderer(GL gl, string hologramVert, string hologramFrag, string runningVert, string runningFrag)
    {
        _gl = gl;
        _hologram = new ShaderProgram(gl, hologramVert, hologramFrag);
        _engineRunning = new ShaderProgram(gl, runningVert, runningFrag);

        _gl.Enable(EnableCap.DepthTest);
        _gl.Enable(EnableCap.Blend);
        _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
        _gl.Enable(EnableCap.CullFace);
        _gl.CullFace(TriangleFace.Back);
    }

    public bool ReloadShaders(string hologramVert, string hologramFrag, string runningVert, string runningFrag)
    {
        try
        {
            var newHolo = new ShaderProgram(_gl, hologramVert, hologramFrag);
            var newRunning = new ShaderProgram(_gl, runningVert, runningFrag);
            _hologram.Dispose();
            _engineRunning.Dispose();
            _hologram = newHolo;
            _engineRunning = newRunning;
            LastShaderReloadMessage = "OK";
            LastShaderReloadTime = DateTime.Now;
            Console.WriteLine($"[ShaderReload] OK @ {LastShaderReloadTime:HH:mm:ss}");
            return true;
        }
        catch (Exception ex)
        {
            LastShaderReloadMessage = ex.Message;
            LastShaderReloadTime = DateTime.Now;
            Console.WriteLine($"[ShaderReload] FAILED: {ex.Message}");
            return false;
        }
    }

    public int AddMesh(GpuMesh mesh, StageId stageId, float revealDelay = 0f, float revealDuration = 3f)
    {
        var item = new RenderItem(mesh, stageId, Matrix4x4.Identity, revealDelay, revealDuration);
        _items.Add(item);
        return _items.Count - 1;
    }

    public void ClearScene()
    {
        foreach (var it in _items) it.Mesh.Dispose();
        _items.Clear();
        _time = 0;
    }

    public void RenderFrame(double deltaTime, int fbWidth, int fbHeight)
    {
        _time += (float)deltaTime;

        _gl.Viewport(0, 0, (uint)fbWidth, (uint)fbHeight);
        _gl.ClearColor(ClearColor.X, ClearColor.Y, ClearColor.Z, 1f);
        _gl.Clear((uint)(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit));

        Matrix4x4 view = Camera.ViewMatrix;
        float aspect = fbWidth / (float)Math.Max(1, fbHeight);
        Matrix4x4 proj = Camera.ProjectionMatrix(aspect);
        Vector3 camPos = Camera.Position;

        // Normal alpha blend for mesh rendering
        _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
        _gl.Enable(EnableCap.CullFace);
        _gl.Enable(EnableCap.DepthTest);
        _gl.DepthMask(true);

        // SDF raymarch preview always renders first (instant preview).
        // Its alpha fades out as PicoGK mesh stages accumulate, so the real
        // mesh naturally takes over once available.
        if (SdfPreview != null && Profiles != null)
        {
            float blend = ComputePreviewBlend();
            SdfPreview.HoloBlend = blend;
            SdfPreview.Draw(Profiles, view, proj, camPos, _time, HoloColor, MetalColor);
        }

        if (_items.Count > 0)
        {
            if (State == EngineState.Materializing)
                DrawHologramPass(view, proj, camPos);
            else
                DrawRunningPass(view, proj, camPos);
        }

        // After the main mesh pass: chamber glow (additive) + plume particles (additive)
        if (State == EngineState.Running)
        {
            float throttle = Startup?.Throttle ?? 1f;
            if (Glow != null && Spec != null)
                Glow.Draw(Spec, view, proj, camPos, _time, throttle);

            if (Plume != null)
            {
                DrawPlume(view, proj);
            }
        }
    }

    private void DrawHologramPass(Matrix4x4 view, Matrix4x4 proj, Vector3 camPos)
    {
        _hologram.Use();
        _hologram.SetMatrix4("uView", view);
        _hologram.SetMatrix4("uProj", proj);
        _hologram.SetVec3("uCameraPos", camPos);
        _hologram.SetFloat("uTime", _time);
        _hologram.SetVec3("uHoloColor", HoloColor);
        _hologram.SetVec3("uMetalColor", MetalColor);

        foreach (var item in _items)
        {
            float progress = Math.Clamp((_time - item.RevealDelay) / item.RevealDuration, 0f, 1.3f);
            _hologram.SetMatrix4("uModel", item.Transform);
            _hologram.SetFloat("uRevealProgress", progress);
            item.Mesh.Draw();
        }
    }

    private void DrawRunningPass(Matrix4x4 view, Matrix4x4 proj, Vector3 camPos)
    {
        _engineRunning.Use();
        _engineRunning.SetMatrix4("uView", view);
        _engineRunning.SetMatrix4("uProj", proj);
        _engineRunning.SetVec3("uCameraPos", camPos);
        _engineRunning.SetFloat("uTime", _time);
        _engineRunning.SetFloat("uThrottle", Startup?.Throttle ?? 1f);

        float zmin = Spec?.zTip ?? 0f;
        float zmax = Spec != null ? Spec.zInjector + 5f : 200f;
        _engineRunning.SetFloat("uZmin", zmin);
        _engineRunning.SetFloat("uZmax", zmax);

        if (HeatProfile != null)
        {
            _gl.ActiveTexture(TextureUnit.Texture0);
            _gl.BindTexture(TextureTarget.Texture1D, HeatProfile.Texture);
            _engineRunning.SetInt("uHeatProfile", 0);
        }

        foreach (var item in _items)
        {
            int mode = item.StageId == StageId.ChannelSdfs ? 1 : 0;
            _engineRunning.SetMatrix4("uModel", item.Transform);
            _engineRunning.SetInt("uMode", mode);
            item.Mesh.Draw();
        }
    }

    private ShaderProgram? _plumeProgram;
    public void SetPlumeProgram(ShaderProgram program) => _plumeProgram = program;

    private void DrawPlume(Matrix4x4 view, Matrix4x4 proj)
    {
        if (Plume == null || _plumeProgram == null) return;

        _gl.Disable(EnableCap.CullFace);
        _gl.DepthMask(false);
        _gl.Enable(EnableCap.Blend);
        _gl.BlendFunc(BlendingFactor.One, BlendingFactor.One);

        _plumeProgram.Use();
        _plumeProgram.SetMatrix4("uView", view);
        _plumeProgram.SetMatrix4("uProj", proj);
        _plumeProgram.SetFloat("uLifetime", Plume.Lifetime);
        _plumeProgram.SetFloat("uPointSize", 48f);

        Plume.Draw();

        _gl.DepthMask(true);
        _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
        _gl.Enable(EnableCap.CullFace);
    }

    /// <summary>
    /// Crossfade preview to mesh: 1.0 = preview alone, 0.0 = mesh alone.
    /// We fade out as more pipeline stages have arrived. Final stage = full mesh visible.
    /// </summary>
    private float ComputePreviewBlend()
    {
        if (State == EngineState.Running) return 0f;        // running mode — mesh dominant
        if (_items.Count == 0) return 1f;                    // nothing built yet — pure preview
        // Linear fade as stage count grows. After ~6 stages we're at 0.4 (still visible behind mesh edges).
        float fade = MathF.Max(0f, 1f - (_items.Count / 8f));
        return fade;
    }

    public BoundingSphere SceneBounds()
    {
        if (_items.Count == 0) return new BoundingSphere(Vector3.Zero, 1f);
        Vector3 min = new(float.MaxValue);
        Vector3 max = new(float.MinValue);
        foreach (var it in _items)
        {
            min = Vector3.Min(min, it.Mesh.Bounds.Center - new Vector3(it.Mesh.Bounds.Radius));
            max = Vector3.Max(max, it.Mesh.Bounds.Center + new Vector3(it.Mesh.Bounds.Radius));
        }
        Vector3 c = (min + max) * 0.5f;
        float r = Vector3.Distance(min, max) * 0.5f;
        return new BoundingSphere(c, r);
    }

    public void Dispose()
    {
        ClearScene();
        _hologram.Dispose();
        _engineRunning.Dispose();
        _plumeProgram?.Dispose();
    }

    private sealed class RenderItem
    {
        public GpuMesh Mesh;
        public StageId StageId;
        public Matrix4x4 Transform;
        public float RevealDelay;
        public float RevealDuration;

        public RenderItem(GpuMesh mesh, StageId stageId, Matrix4x4 transform, float revealDelay, float revealDuration)
        {
            Mesh = mesh;
            StageId = stageId;
            Transform = transform;
            RevealDelay = revealDelay;
            RevealDuration = revealDuration;
        }
    }
}
