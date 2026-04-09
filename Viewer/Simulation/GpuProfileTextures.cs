// GpuProfileTextures.cs - sample r(z) profiles into 1D textures so the SDF
// raymarcher can evaluate the engine body in real time on the GPU without
// waiting for PicoGK voxelization.
//
// Two RGBA32F 1D textures (1024 wide):
//   profile.r = rSpike(z) in mm
//   profile.g = rShroud(z) in mm
//   profile.b = wallThickness(z) in mm  (HeatTransfer.WallThickness)
//   profile.a = shellThickness(z) in mm (wall + 2*chR)
//
// Re-uploaded whenever the AeroSpec parameters change. Cost: ~1 ms.

using OpenSpaceArch.Engine;
using Silk.NET.OpenGL;

namespace OpenSpaceArch.Viewer.Simulation;

public sealed class GpuProfileTextures : IDisposable
{
    private readonly GL _gl;
    public uint Texture { get; }
    public int Width { get; }
    public float Zmin { get; private set; }
    public float Zmax { get; private set; }
    public float Rmax { get; private set; }
    public bool HasData { get; private set; }

    public GpuProfileTextures(GL gl, int width = 1024)
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
    /// Recompute profiles from current AeroSpec and upload to GPU.
    /// Spec must already have ChamberSizing.Compute applied.
    /// </summary>
    public unsafe void Upload(AeroSpec S)
    {
        Zmin = S.zTip - 1f;
        Zmax = S.zInjector + 5f;

        float[] data = new float[Width * 4];
        float rmax = 0f;

        for (int i = 0; i < Width; i++)
        {
            float t = i / (float)(Width - 1);
            float z = Zmin + t * (Zmax - Zmin);

            float rSpike = MathF.Max(0f, ChamberSizing.SpikeProfile(S, z));
            float rShroud = MathF.Max(0f, ChamberSizing.ShroudProfile(S, z));
            float wall = MathF.Max(S.minPrintWall, HeatTransfer.WallThickness(S, z));
            float chR = HeatTransfer.ChannelRadius(S, z);
            float shell = wall + 2f * chR + S.minPrintWall;

            data[i * 4 + 0] = rSpike;
            data[i * 4 + 1] = rShroud;
            data[i * 4 + 2] = wall;
            data[i * 4 + 3] = shell;

            float outer = rShroud + shell;
            if (outer > rmax) rmax = outer;
        }

        Rmax = rmax;

        _gl.BindTexture(TextureTarget.Texture1D, Texture);
        fixed (float* p = data)
        {
            _gl.TexImage1D(
                TextureTarget.Texture1D, 0, InternalFormat.Rgba32f,
                (uint)Width, 0, PixelFormat.Rgba, PixelType.Float, p);
        }

        HasData = true;
    }

    public void Dispose() => _gl.DeleteTexture(Texture);
}
