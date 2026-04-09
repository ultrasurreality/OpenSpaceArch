// SdfRaymarchPass.cs - full-screen pass that sphere-traces the engine body
// directly from the GpuProfileTextures. Renders before the mesh pass so the
// PicoGK voxel meshes naturally draw on top once they arrive.
//
// HoloBlend uniform fades the raymarcher out as PicoGK stages accumulate:
//   1.0 = no mesh yet, raymarcher is the only image
//   0.0 = full mesh present, raymarcher invisible
// Smooth crossfade is the responsibility of the caller.

using System.Numerics;
using Silk.NET.OpenGL;
using OpenSpaceArch.Viewer.Rendering;

namespace OpenSpaceArch.Viewer.Simulation;

public sealed class SdfRaymarchPass : IDisposable
{
    private readonly GL _gl;
    private readonly ShaderProgram _program;
    private readonly uint _vao;
    private readonly uint _vbo;

    public float HoloBlend = 1f;

    public SdfRaymarchPass(GL gl, string vertSrc, string fragSrc)
    {
        _gl = gl;
        _program = new ShaderProgram(gl, vertSrc, fragSrc);

        float[] verts = { -1f, -1f, 3f, -1f, -1f, 3f };
        _vao = gl.GenVertexArray();
        gl.BindVertexArray(_vao);
        _vbo = gl.GenBuffer();
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, _vbo);
        unsafe
        {
            fixed (float* p = verts)
                gl.BufferData(BufferTargetARB.ArrayBuffer,
                    (nuint)(verts.Length * sizeof(float)), p, BufferUsageARB.StaticDraw);
            gl.EnableVertexAttribArray(0);
            gl.VertexAttribPointer(0, 2, VertexAttribPointerType.Float, false,
                2 * sizeof(float), (void*)0);
        }
        gl.BindVertexArray(0);
    }

    public void Draw(GpuProfileTextures profiles, Matrix4x4 view, Matrix4x4 proj,
                     Vector3 cameraPos, float time, Vector3 holoColor, Vector3 metalColor)
    {
        if (!profiles.HasData || HoloBlend <= 0.01f) return;

        _gl.DepthMask(false);
        _gl.Disable(EnableCap.DepthTest);
        _gl.Disable(EnableCap.CullFace);
        _gl.Enable(EnableCap.Blend);
        _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

        _program.Use();

        Matrix4x4 vp = view * proj;
        if (!Matrix4x4.Invert(vp, out var invVP))
            invVP = Matrix4x4.Identity;
        _program.SetMatrix4("uInvViewProj", invVP);
        _program.SetVec3("uCameraPos", cameraPos);
        _program.SetFloat("uTime", time);
        _program.SetFloat("uHoloBlend", HoloBlend);
        _program.SetFloat("uZmin", profiles.Zmin);
        _program.SetFloat("uZmax", profiles.Zmax);
        _program.SetFloat("uRmax", profiles.Rmax);
        _program.SetVec3("uHoloColor", holoColor);
        _program.SetVec3("uMetalColor", metalColor);

        _gl.ActiveTexture(TextureUnit.Texture0);
        _gl.BindTexture(TextureTarget.Texture1D, profiles.Texture);
        _program.SetInt("uProfile", 0);

        _gl.BindVertexArray(_vao);
        _gl.DrawArrays(PrimitiveType.Triangles, 0, 3);
        _gl.BindVertexArray(0);

        _gl.DepthMask(true);
        _gl.Enable(EnableCap.DepthTest);
        _gl.Enable(EnableCap.CullFace);
        _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
    }

    public void Dispose()
    {
        _gl.DeleteBuffer(_vbo);
        _gl.DeleteVertexArray(_vao);
        _program.Dispose();
    }
}
