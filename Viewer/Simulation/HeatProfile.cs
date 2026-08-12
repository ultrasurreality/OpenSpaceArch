// HeatProfile.cs — builds a 1D float texture of gas static temperature T_gas(z) [K]
// (for display) and heat flux q(z) [W/m²] from the already-computed AeroSpec +
// HeatTransfer state.
//
// The engine_running.frag shader samples this texture using the vertex's world Z
// (remapped to [0..1] across the engine length) and colors the wall accordingly.
// Note: the R channel is the gas STATIC temperature, not the gas-side wall
// temperature — it is used purely as a display ramp, not as a thermal result.
//
// Texture layout: RG32F, width = 256 samples
//   R channel = normalized gas static temperature [0..1] (Tc ≈ 1.0, ambient ≈ 0.0)
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
    /// Builds a (T_gas(z), q(z)) texture from the real Bartz heat flux of the
    /// current spec. T_gas(z) is the gas static temperature from the isentropic
    /// flow profile (GasFlowProfile), used as a display ramp — it is NOT a wall
    /// temperature. q(z) comes from HeatTransfer.HeatFlux(S, z) directly.
    /// </summary>
    public unsafe void Upload(AeroSpec S, GasFlowProfile flow)
    {
        Zmin = S.zTip;
        Zmax = S.zInjector + 5f;

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

            // Display ramp: normalize the gas STATIC temperature at this station
            // into [0..1]. This is not a wall temperature — the gas-side wall
            // would sit well below T_gas due to film cooling — it is only used
            // to color the wall for the running-engine visualization.
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
