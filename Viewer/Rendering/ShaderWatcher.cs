// ShaderWatcher.cs — watches Viewer/Shaders/*.vert|*.frag for changes and sets a pending flag.
// The main thread calls ConsumePending() each frame and, if set, reloads shaders in the GL context.
//
// FileSystemWatcher events come on an IO thread — we never touch GL from there, we only flip a flag.

namespace OpenSpaceArch.Viewer.Rendering;

public sealed class ShaderWatcher : IDisposable
{
    private readonly FileSystemWatcher _watcher;
    private readonly string _shaderDir;
    private volatile bool _pending;
    private DateTime _lastDebounce = DateTime.MinValue;
    private readonly TimeSpan _debounceWindow = TimeSpan.FromMilliseconds(100);

    public string ShaderDir => _shaderDir;

    public ShaderWatcher(string shaderDir)
    {
        _shaderDir = Path.GetFullPath(shaderDir);
        if (!Directory.Exists(_shaderDir))
            throw new DirectoryNotFoundException($"Shader dir not found: {_shaderDir}");

        _watcher = new FileSystemWatcher(_shaderDir)
        {
            Filter = "*.*",
            NotifyFilter = NotifyFilters.LastWrite | NotifyFilters.FileName | NotifyFilters.Size,
            IncludeSubdirectories = false,
            EnableRaisingEvents = true,
        };
        _watcher.Changed += OnChange;
        _watcher.Created += OnChange;
        _watcher.Renamed += OnChange;

        Console.WriteLine($"[ShaderWatcher] Watching {_shaderDir}");
    }

    private void OnChange(object? sender, FileSystemEventArgs e)
    {
        if (!e.FullPath.EndsWith(".vert", StringComparison.OrdinalIgnoreCase) &&
            !e.FullPath.EndsWith(".frag", StringComparison.OrdinalIgnoreCase))
            return;

        // Debounce — editors save in bursts (temp file → rename → change) that fire 2-4 events
        var now = DateTime.Now;
        if ((now - _lastDebounce) < _debounceWindow) return;
        _lastDebounce = now;

        _pending = true;
        Console.WriteLine($"[ShaderWatcher] Detected: {Path.GetFileName(e.FullPath)}");
    }

    /// <summary>
    /// Returns true once when a change has been detected. Call from main GL thread each frame.
    /// After returning true, clears the pending flag.
    /// </summary>
    public bool ConsumePending()
    {
        if (!_pending) return false;
        _pending = false;
        return true;
    }

    public (string vert, string frag) ReadShaders(string vertName, string fragName)
    {
        // Retry briefly — editor may be mid-write when FileSystemWatcher fires
        for (int attempt = 0; attempt < 5; attempt++)
        {
            try
            {
                string v = File.ReadAllText(Path.Combine(_shaderDir, vertName));
                string f = File.ReadAllText(Path.Combine(_shaderDir, fragName));
                return (v, f);
            }
            catch (IOException) when (attempt < 4)
            {
                Thread.Sleep(40);
            }
        }
        throw new IOException($"Could not read shader files after retries");
    }

    public void Dispose() => _watcher.Dispose();
}
