// ParticleSystem.cs - geometry-aware particles for the running aerospike engine.
//
// Two flavours share one VBO for efficient rendering:
//   Gas: spawn at injector face, follow gas path through annular nozzle,
//        accelerate through throat using real u(z) from GasFlowProfile,
//        expand around the spike tip. Real m/s velocities.
//   Coolant: spawn at shroud inlet ring (z=zCowl), helix upward along the
//            channel wall band (rShroud + wall + chR/2), exit at zChTop.
//
// Both respect the actual geometry every frame via ChamberSizing.ShroudProfile
// and SpikeProfile. No ad-hoc motion.

using System.Numerics;
using OpenSpaceArch.Engine;
using Silk.NET.OpenGL;

namespace OpenSpaceArch.Viewer.Simulation;

public sealed class ParticleSystem : IDisposable
{
    private readonly GL _gl;
    private readonly int _capacity;
    private readonly Particle[] _particles;
    private readonly Vector4[] _uploadBuffer;
    private readonly uint _vao;
    private readonly uint _vbo;
    private int _aliveCount;
    private float _spawnAccumGas;
    private float _spawnAccumCoolant;
    private Random _rng = new(12345);

    public AeroSpec? Spec;
    public GasFlowProfile? Flow;
    public float Throttle = 1f;    // 0..1

    // World-to-visual scale: AeroSpec is in mm. Scale m/s velocities to mm/s for display speed.
    // cinemaScale adjusts cinematic pacing: >1.0 makes plume noticeably faster for the viewer.
    public float CinemaScale = 0.35f;

    public int AliveCount => _aliveCount;
    public float Lifetime { get; set; } = 0.45f;

    public ParticleSystem(GL gl, int capacity = 24000)
    {
        _gl = gl;
        _capacity = capacity;
        _particles = new Particle[capacity];
        _uploadBuffer = new Vector4[capacity];

        _vao = gl.GenVertexArray();
        gl.BindVertexArray(_vao);
        _vbo = gl.GenBuffer();
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, _vbo);
        unsafe
        {
            gl.BufferData(BufferTargetARB.ArrayBuffer,
                (nuint)(capacity * sizeof(float) * 4), null,
                BufferUsageARB.DynamicDraw);
            gl.EnableVertexAttribArray(0);
            gl.VertexAttribPointer(0, 4, VertexAttribPointerType.Float, false,
                (uint)(sizeof(float) * 4), (void*)0);
        }
        gl.BindVertexArray(0);

        gl.Enable(EnableCap.ProgramPointSize);
    }

    public unsafe void Update(float dt)
    {
        if (Spec is null || Flow is null) return;
        var S = Spec;

        // ── Spawn new gas particles at the injector face ───────────────────
        float gasRate = 18000f * Throttle;
        _spawnAccumGas += dt * gasRate;
        int toSpawnGas = (int)_spawnAccumGas;
        _spawnAccumGas -= toSpawnGas;

        float zInj = S.zInjector - 0.5f;
        float rSpikeInj = MathF.Max(ChamberSizing.SpikeProfile(S, zInj), 0f);
        float rShroudInj = MathF.Max(ChamberSizing.ShroudProfile(S, zInj), rSpikeInj + 1f);

        for (int i = 0; i < toSpawnGas; i++)
        {
            int slot = FindDeadSlot();
            if (slot < 0) break;

            float phi = (float)(_rng.NextDouble() * Math.PI * 2.0);
            float r = Lerp(rSpikeInj + 0.3f, rShroudInj - 0.3f, (float)_rng.NextDouble());
            float jitter = ((float)_rng.NextDouble() - 0.5f) * 0.8f;

            _particles[slot].Pos = new Vector3(
                r * MathF.Cos(phi),
                r * MathF.Sin(phi),
                zInj + jitter);
            _particles[slot].Age = 0f;
            _particles[slot].Lifetime = Lifetime * (0.85f + 0.3f * (float)_rng.NextDouble());
            _particles[slot].Alive = true;
            _particles[slot].Kind = ParticleKind.Gas;
            _particles[slot].Phi0 = phi;
        }

        // ── Spawn new coolant particles at the shroud inlet ring ───────────
        float coolantRate = 8000f * Throttle;
        _spawnAccumCoolant += dt * coolantRate;
        int toSpawnCoolant = (int)_spawnAccumCoolant;
        _spawnAccumCoolant -= toSpawnCoolant;

        float zIn = S.zCowl + 3f;
        float wallIn = HeatTransfer.WallThickness(S, zIn);
        float chRIn = HeatTransfer.ChannelRadius(S, zIn);
        float rShIn = MathF.Max(ChamberSizing.ShroudProfile(S, zIn), 1f);
        float rBandIn = rShIn + wallIn + chRIn;

        for (int i = 0; i < toSpawnCoolant; i++)
        {
            int slot = FindDeadSlot();
            if (slot < 0) break;

            float phi = (float)(_rng.NextDouble() * Math.PI * 2.0);

            _particles[slot].Pos = new Vector3(
                rBandIn * MathF.Cos(phi),
                rBandIn * MathF.Sin(phi),
                zIn);
            _particles[slot].Age = 0f;
            _particles[slot].Lifetime = 1.5f;
            _particles[slot].Alive = true;
            _particles[slot].Kind = ParticleKind.Coolant;
            _particles[slot].Phi0 = phi;
        }

        // ── Update all alive particles ──────────────────────────────────────
        int alive = 0;
        float gamma = S.gamma;

        for (int i = 0; i < _capacity; i++)
        {
            ref var p = ref _particles[i];
            if (!p.Alive) continue;
            p.Age += dt;
            if (p.Age >= p.Lifetime)
            {
                p.Alive = false;
                continue;
            }

            if (p.Kind == ParticleKind.Gas)
            {
                UpdateGas(ref p, dt, S);
            }
            else
            {
                UpdateCoolant(ref p, dt, S);
            }

            _uploadBuffer[alive++] = new Vector4(p.Pos, p.Age);
        }
        _aliveCount = alive;

        if (_aliveCount > 0)
        {
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _vbo);
            fixed (Vector4* buf = _uploadBuffer)
            {
                _gl.BufferSubData(BufferTargetARB.ArrayBuffer, 0,
                    (nuint)(_aliveCount * sizeof(float) * 4), buf);
            }
        }
    }

    private void UpdateGas(ref Particle p, float dt, AeroSpec S)
    {
        // Sample real gas velocity in m/s, convert to mm/s display (× 1000 × CinemaScale)
        float u_ms = Flow!.VelocityAt(p.Pos.Z);
        float speed = u_ms * 1000f * CinemaScale;

        // Direction: predominantly -Z (toward spike tip). Inside the nozzle,
        // particles stay within the annular gas path; past the tip they expand.
        float newZ = p.Pos.Z - speed * dt;
        p.Pos.Z = newZ;

        if (newZ > S.zTip)
        {
            // Inside nozzle: enforce annular constraint
            float rSpike = MathF.Max(ChamberSizing.SpikeProfile(S, newZ), 0f);
            float rShroud = MathF.Max(ChamberSizing.ShroudProfile(S, newZ), rSpike + 0.5f);

            // Keep particle radius between rSpike+eps and rShroud-eps
            float rNow = MathF.Sqrt(p.Pos.X * p.Pos.X + p.Pos.Y * p.Pos.Y);
            float rTarget = Math.Clamp(rNow, rSpike + 0.3f, rShroud - 0.3f);
            if (rNow > 0.001f && MathF.Abs(rTarget - rNow) > 1e-4f)
            {
                float nx = p.Pos.X / rNow;
                float ny = p.Pos.Y / rNow;
                p.Pos.X = rTarget * nx;
                p.Pos.Y = rTarget * ny;
            }
        }
        else
        {
            // Past spike tip: expand radially (conservation of momentum + pressure matching)
            float rNow = MathF.Sqrt(p.Pos.X * p.Pos.X + p.Pos.Y * p.Pos.Y);
            if (rNow > 0.001f)
            {
                float nx = p.Pos.X / rNow;
                float ny = p.Pos.Y / rNow;
                // Expand outward — aerospike under-expansion gives slight outward cone
                float radialSpeed = speed * 0.12f;
                p.Pos.X += nx * radialSpeed * dt;
                p.Pos.Y += ny * radialSpeed * dt;
            }
        }
    }

    private void UpdateCoolant(ref Particle p, float dt, AeroSpec S)
    {
        // Coolant flows UP the wall band from zCowl to zChTop with helical twist
        // Use measured v_cool_max as the speed; helix pitch from channelTwistTurns.
        float speed = S.v_cool_max * 1000f * 0.18f;  // m/s -> mm/s display, slowed for visibility
        p.Pos.Z += speed * dt;

        // Radial: stick to wall band at current z
        float wall = HeatTransfer.WallThickness(S, p.Pos.Z);
        float chR = HeatTransfer.ChannelRadius(S, p.Pos.Z);
        float rSh = ChamberSizing.ShroudProfile(S, p.Pos.Z);
        if (rSh < 1f) rSh = S.rShroudThroat;
        float rBand = rSh + wall + chR;

        // Helical phase advance along z (full twist count over channel length)
        float zFrac = (p.Pos.Z - S.zCowl) / MathF.Max(1f, S.zChTop - S.zCowl);
        float twistRad = S.channelTwistTurns * 2f * MathF.PI;
        float phi = p.Phi0 + twistRad * Math.Clamp(zFrac, 0f, 1f);

        p.Pos.X = rBand * MathF.Cos(phi);
        p.Pos.Y = rBand * MathF.Sin(phi);

        // Kill when past the collector ring
        if (p.Pos.Z > S.zChTop + 2f)
            p.Alive = false;
    }

    public void Draw()
    {
        if (_aliveCount == 0) return;
        _gl.BindVertexArray(_vao);
        _gl.DrawArrays(PrimitiveType.Points, 0, (uint)_aliveCount);
        _gl.BindVertexArray(0);
    }

    private int FindDeadSlot()
    {
        for (int i = 0; i < _capacity; i++)
            if (!_particles[i].Alive) return i;
        return -1;
    }

    private static float Lerp(float a, float b, float t) => a + (b - a) * t;

    public void Dispose()
    {
        _gl.DeleteBuffer(_vbo);
        _gl.DeleteVertexArray(_vao);
    }

    private enum ParticleKind : byte { Gas, Coolant }

    private struct Particle
    {
        public Vector3 Pos;
        public float Age;
        public float Lifetime;
        public float Phi0;
        public ParticleKind Kind;
        public bool Alive;
    }
}
