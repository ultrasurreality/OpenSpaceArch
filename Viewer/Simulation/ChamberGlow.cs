// ChamberGlow.cs — full-screen quad that raymarches volumetric plasma
// inside the chamber cavity AABB. Additively blended on top of the scene.
//
// Requires: glow.vert + glow.frag, camera invViewProj matrix, chamber bounds.

using System.Numerics;
using OpenSpaceArch.Engine;
using Silk.NET.OpenGL;
using OpenSpaceArch.Viewer.Rendering;

namespace OpenSpaceArch.Viewer.Simulation;

public sealed class ChamberGlow : IDisposable
{
    private readonly GL _gl;
    private readonly ShaderProgram _program;
    private readonly uint _vao;
    private readonly uint _vbo;

    public ChamberGlow(GL gl, string vertSrc, string fragSrc)
    {
        _gl = gl;
        _program = new ShaderProgram(gl, vertSrc, fragSrc);

        // Full-screen triangle in NDC
        float[] verts = {
            -1f, -1f,
             3f, -1f,
            -1f,  3f,
        };

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

    public void Draw(AeroSpec spec, Matrix4x4 view, Matrix4x4 proj, Vector3 cameraPos, float time, float throttle)
    {
        // Depth write off, additive blending
        _gl.DepthMask(false);
        _gl.Enable(EnableCap.Blend);
        _gl.BlendFunc(BlendingFactor.One, BlendingFactor.One);

        _program.Use();

        Matrix4x4 vp = view * proj;
        if (!Matrix4x4.Invert(vp, out Matrix4x4 invVP))
            invVP = Matrix4x4.Identity;
        _program.SetMatrix4("uInvViewProj", invVP);
        _program.SetVec3("uCameraPos", cameraPos);
        _program.SetFloat("uTime", time);
        _program.SetFloat("uThrottle", throttle);

        // Chamber bounds in world space (slightly inside the shroud radius at chamber z)
        float r = spec.rShroudChamber - spec.minPrintWall * 0.5f;
        _program.SetVec3("uChamberMin", new Vector3(-r, -r, spec.zChBot));
        _program.SetVec3("uChamberMax", new Vector3( r,  r, spec.zChTop));

        _gl.BindVertexArray(_vao);
        _gl.DrawArrays(PrimitiveType.Triangles, 0, 3);
        _gl.BindVertexArray(0);

        // Restore default blending
        _gl.DepthMask(true);
        _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
    }

    public void Dispose()
    {
        _gl.DeleteBuffer(_vbo);
        _gl.DeleteVertexArray(_vao);
        _program.Dispose();
    }
}
