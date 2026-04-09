// AppWindow.cs — Silk.NET window wrapper for OpenSpaceArch cinematic viewer.
// Owns the OpenGL 4.6 context and exposes Load/Render/Update events.

using Silk.NET.Maths;
using Silk.NET.OpenGL;
using Silk.NET.Windowing;

namespace OpenSpaceArch.Viewer.Window;

public sealed class AppWindow
{
    private IWindow _window = null!;
    public GL Gl { get; private set; } = null!;
    public Vector2D<int> Size => _window.Size;

    public event Action<GL>? OnLoad;
    public event Action<double, GL>? OnRender;
    public event Action<double>? OnUpdate;
    public event Action<Vector2D<int>>? OnResize;
    public event Action? OnClose;

    public static AppWindow Create(int width, int height, string title)
    {
        var options = WindowOptions.Default;
        options.Size = new Vector2D<int>(width, height);
        options.Title = title;
        options.API = new GraphicsAPI(
            ContextAPI.OpenGL,
            ContextProfile.Core,
            ContextFlags.ForwardCompatible,
            new APIVersion(4, 6));
        options.VSync = true;
        options.ShouldSwapAutomatically = true;

        var app = new AppWindow();
        app._window = Silk.NET.Windowing.Window.Create(options);
        app._window.Load += app.HandleLoad;
        app._window.Render += app.HandleRender;
        app._window.Update += app.HandleUpdate;
        app._window.FramebufferResize += app.HandleResize;
        app._window.Closing += () => app.OnClose?.Invoke();
        return app;
    }

    public IWindow RawWindow => _window;

    public void Run() => _window.Run();

    public void Close() => _window.Close();

    private void HandleLoad()
    {
        Gl = GL.GetApi(_window);
        Console.WriteLine($"OpenGL {Gl.GetStringS(StringName.Version)}");
        Console.WriteLine($"Renderer: {Gl.GetStringS(StringName.Renderer)}");
        OnLoad?.Invoke(Gl);
    }

    private void HandleRender(double deltaTime) => OnRender?.Invoke(deltaTime, Gl);
    private void HandleUpdate(double deltaTime) => OnUpdate?.Invoke(deltaTime);
    private void HandleResize(Vector2D<int> size)
    {
        Gl.Viewport(size);
        OnResize?.Invoke(size);
    }
}
