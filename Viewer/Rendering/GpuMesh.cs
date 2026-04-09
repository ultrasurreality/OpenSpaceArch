// GpuMesh.cs — uploads a PicoGK Mesh into a VAO/VBO pair for rendering.
// Vertex format: [position.xyz, normal.xyz] = 6 floats per vertex.
// Normals are computed per-triangle and averaged per-vertex (smooth shading).

using System.Numerics;
using PicoGK;
using Silk.NET.OpenGL;

namespace OpenSpaceArch.Viewer.Rendering;

public sealed class GpuMesh : IDisposable
{
    private readonly GL _gl;
    private readonly uint _vao;
    private readonly uint _vbo;
    private readonly uint _ebo;
    public uint IndexCount { get; }
    public BoundingSphere Bounds { get; }

    public unsafe GpuMesh(GL gl, Mesh mesh)
    {
        _gl = gl;

        int nVerts = mesh.nVertexCount();
        int nTris = mesh.nTriangleCount();

        // Compute vertex normals by averaging face normals
        var positions = new Vector3[nVerts];
        var normals = new Vector3[nVerts];
        for (int i = 0; i < nVerts; i++)
            positions[i] = mesh.vecVertexAt(i);

        var indices = new uint[nTris * 3];
        for (int i = 0; i < nTris; i++)
        {
            Triangle t = mesh.oTriangleAt(i);
            indices[i * 3 + 0] = (uint)t.A;
            indices[i * 3 + 1] = (uint)t.B;
            indices[i * 3 + 2] = (uint)t.C;

            Vector3 a = positions[t.A];
            Vector3 b = positions[t.B];
            Vector3 c = positions[t.C];
            Vector3 faceN = Vector3.Normalize(Vector3.Cross(b - a, c - a));
            normals[t.A] += faceN;
            normals[t.B] += faceN;
            normals[t.C] += faceN;
        }
        for (int i = 0; i < nVerts; i++)
            normals[i] = normals[i].LengthSquared() > 1e-6f ? Vector3.Normalize(normals[i]) : Vector3.UnitZ;

        // Interleave [pos.xyz, norm.xyz]
        var interleaved = new float[nVerts * 6];
        Vector3 bbMin = positions[0], bbMax = positions[0];
        for (int i = 0; i < nVerts; i++)
        {
            interleaved[i * 6 + 0] = positions[i].X;
            interleaved[i * 6 + 1] = positions[i].Y;
            interleaved[i * 6 + 2] = positions[i].Z;
            interleaved[i * 6 + 3] = normals[i].X;
            interleaved[i * 6 + 4] = normals[i].Y;
            interleaved[i * 6 + 5] = normals[i].Z;
            bbMin = Vector3.Min(bbMin, positions[i]);
            bbMax = Vector3.Max(bbMax, positions[i]);
        }

        Vector3 center = (bbMin + bbMax) * 0.5f;
        float radius = Vector3.Distance(bbMin, bbMax) * 0.5f;
        Bounds = new BoundingSphere(center, radius);
        IndexCount = (uint)(nTris * 3);

        _vao = _gl.GenVertexArray();
        _gl.BindVertexArray(_vao);

        _vbo = _gl.GenBuffer();
        _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _vbo);
        fixed (float* p = interleaved)
        {
            _gl.BufferData(BufferTargetARB.ArrayBuffer,
                (nuint)(interleaved.Length * sizeof(float)),
                p, BufferUsageARB.StaticDraw);
        }

        _ebo = _gl.GenBuffer();
        _gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, _ebo);
        fixed (uint* p = indices)
        {
            _gl.BufferData(BufferTargetARB.ElementArrayBuffer,
                (nuint)(indices.Length * sizeof(uint)),
                p, BufferUsageARB.StaticDraw);
        }

        const uint stride = 6 * sizeof(float);
        _gl.EnableVertexAttribArray(0);
        _gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, stride, (void*)0);
        _gl.EnableVertexAttribArray(1);
        _gl.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, stride, (void*)(3 * sizeof(float)));

        _gl.BindVertexArray(0);
    }

    public unsafe void Draw()
    {
        _gl.BindVertexArray(_vao);
        _gl.DrawElements(PrimitiveType.Triangles, IndexCount, DrawElementsType.UnsignedInt, (void*)0);
        _gl.BindVertexArray(0);
    }

    public void Dispose()
    {
        _gl.DeleteVertexArray(_vao);
        _gl.DeleteBuffer(_vbo);
        _gl.DeleteBuffer(_ebo);
    }
}
