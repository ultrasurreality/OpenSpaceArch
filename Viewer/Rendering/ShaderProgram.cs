// ShaderProgram.cs — minimal GLSL shader program wrapper.

using System.Numerics;
using Silk.NET.OpenGL;

namespace OpenSpaceArch.Viewer.Rendering;

public sealed class ShaderProgram : IDisposable
{
    private readonly GL _gl;
    public uint Handle { get; }

    public ShaderProgram(GL gl, string vertSource, string fragSource)
    {
        _gl = gl;
        uint vs = CompileShader(ShaderType.VertexShader, vertSource);
        uint fs = CompileShader(ShaderType.FragmentShader, fragSource);

        Handle = _gl.CreateProgram();
        _gl.AttachShader(Handle, vs);
        _gl.AttachShader(Handle, fs);
        _gl.LinkProgram(Handle);

        _gl.GetProgram(Handle, ProgramPropertyARB.LinkStatus, out int ok);
        if (ok == 0)
        {
            string log = _gl.GetProgramInfoLog(Handle);
            throw new Exception($"Shader link failed:\n{log}");
        }

        _gl.DetachShader(Handle, vs);
        _gl.DetachShader(Handle, fs);
        _gl.DeleteShader(vs);
        _gl.DeleteShader(fs);
    }

    private uint CompileShader(ShaderType type, string source)
    {
        uint id = _gl.CreateShader(type);
        _gl.ShaderSource(id, source);
        _gl.CompileShader(id);
        _gl.GetShader(id, ShaderParameterName.CompileStatus, out int ok);
        if (ok == 0)
        {
            string log = _gl.GetShaderInfoLog(id);
            throw new Exception($"Shader compile failed ({type}):\n{log}");
        }
        return id;
    }

    public void Use() => _gl.UseProgram(Handle);

    public int U(string name) => _gl.GetUniformLocation(Handle, name);

    public unsafe void SetMatrix4(string name, Matrix4x4 m)
    {
        _gl.UniformMatrix4(U(name), 1, false, (float*)&m);
    }

    public void SetVec3(string name, Vector3 v) => _gl.Uniform3(U(name), v.X, v.Y, v.Z);
    public void SetFloat(string name, float v) => _gl.Uniform1(U(name), v);
    public void SetInt(string name, int v) => _gl.Uniform1(U(name), v);

    public void Dispose() => _gl.DeleteProgram(Handle);
}
