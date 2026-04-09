// HeatProfile.cs — builds a 1D float texture of wall temperature T_wall(z) [K]
// and heat flux q(z) [W/m²] from the already-computed AeroSpec + HeatTransfer state.
//
// The engine_running.frag shader samples this texture using the vertex's world Z
// (remapped to [0..1] across the engine length) and colors the wall accordingly.
//
// Texture layout: RG32F, width = 256 samples
//   R channel = normalized temperature [0..1] (Tc wall ≈ 1.0, ambient ≈ 0.0)
//   G channel = normalized heat flux [0..1] (throat peak ≈ 1.0)

using OpenSpaceArch.Engine;
using Silk.NET.OpenGL;

namespace OpenSpaceArch.Viewer.Simulation;

public sealed class HeatProfile : IDisposable
{
    private readonly GL _gl;
    public uint Texture { get; }
    public int Width { get; }
    public float Zmin { get; private set; }
    public float Zmax { get; private set; }
    public float Twall_K { get; private set; }
    public float QmaxWm2 { get; private set; }

    public HeatProfile(GL gl, int width = 256)
    {
        _gl = gl;
        Width = width;
        Texture = gl.GenTexture();
        gl.BindTexture(TextureTarget.Texture1D, Texture);
        gl.TexParameter(TextureTarget.Texture1D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear);
        gl.TexParameter(TextureTarget.Texture1D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
        gl.TexParameter(TextureTarget.Texture1D, TextureParameterName.TextureWrapS, (int)TextureWrapMode.ClampToEdge);
    }

    /// <summary>
    /// Builds a physically-grounded (T(z), q(z)) texture from the real Bartz
    /// heat flux of the current spec. T(z) follows the isentropic flow profile
    /// from GasFlowProfile. q(z) comes from HeatTransfer.HeatFlux(S, z) directly.
    /// </summary>
    public unsafe void Upload(AeroSpec S, GasFlowProfile flow)
    {
        Zmin = S.zTip;
        Zmax = S.zInjector + 5f;
        Twall_K = 800f;

        float[] data = new float[Width * 2];
        float qPeak = 0f;
        float[] qRaw = new float[Width];
        float[] tRaw = new float[Width];
        float Tmax = S.Tc;
        float Tamb = 300f;

        for (int i = 0; i < Width; i++)
        {
            float t = i / (float)(Width - 1);
            float z = Zmin + t * (Zmax - Zmin);

            // Real gas static temperature at this Z from isentropic flow.
            // Outside the gas path region, fall back to ambient.
            float tempK;
            if (z < S.zTip || z > S.zInjector)
                tempK = Tamb;
            else
                tempK = flow.TempAt(z);

            // Wall temperature heuristic: gas-side wall sits ~T_aw - film,
            // approx 30-40% between ambient and gas static at this station.
            // For display we use gas static temp normalized into the heat ramp.
            float tempNorm = (tempK - Tamb) / MathF.Max(1f, Tmax - Tamb);
            tRaw[i] = tempNorm;

            // Heat flux from actual Bartz implementation (W/m^2).
            float q = HeatTransfer.HeatFlux(S, z);
            if (q < 0f) q = 0f;
            qRaw[i] = q;
            if (q > qPeak) qPeak = q;
        }

        for (int i = 0; i < Width; i++)
        {
            data[i * 2]     = Math.Clamp(tRaw[i], 0f, 1f);
            data[i * 2 + 1] = qPeak > 0f ? qRaw[i] / qPeak : 0f;
        }

        QmaxWm2 = qPeak;

        _gl.BindTexture(TextureTarget.Texture1D, Texture);
        fixed (float* p = data)
        {
            _gl.TexImage1D(
                TextureTarget.Texture1D, 0, InternalFormat.RG32f,
                (uint)Width, 0, PixelFormat.RG, PixelType.Float, p);
        }
    }

    public void Dispose() => _gl.DeleteTexture(Texture);
}
