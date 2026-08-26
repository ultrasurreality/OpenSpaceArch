// SPDX-License-Identifier: AGPL-3.0-or-later
//
// OpenSpaceArch — Open Computational Architecture for Aerospace Hardware
// Copyright (C) 2025-2026 ultrasurreality
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// PipelineController.cs — runs FluidFirst.Build on a background thread and
// exposes a thread-safe queue of per-stage StageResults to the main thread.
//
// Main thread must drain the queue every frame and turn each StageResult into
// a GpuMesh (GL context-bound, cannot be done on the background thread).

using System.Collections.Concurrent;
using System.Diagnostics;
using OpenSpaceArch.Engine;

namespace OpenSpaceArch.Viewer.Pipeline;

/// <summary>
/// A timing record for one completed pipeline stage. Used by the UI log panel.
/// </summary>
public readonly record struct StageTiming(
    int Epoch,                  // build instance number
    int Index,                  // stage index within this build
    StageId Stage,
    string Description,
    float LapSec,               // time this stage took
    float CumulativeSec);       // total time since build start

public sealed class PipelineController
{
    private readonly ConcurrentQueue<StageResult> _queue = new();
    private readonly object _logLock = new();
    private readonly List<StageTiming> _log = new();
    private CancellationTokenSource? _cts;
    private Task? _buildTask;
    private volatile int _stageCount;
    private volatile bool _running;
    private volatile int _epoch;          // monotonic build counter — old tasks know they're stale
    private Exception? _lastError;
    private readonly Stopwatch _wallClock = new();
    public float CurrentBuildElapsedSec => _running ? (float)_wallClock.Elapsed.TotalSeconds : _lastBuildTotalSec;
    private float _lastBuildTotalSec;

    public int StagesReceived => _stageCount;
    public bool IsRunning => _running;
    public Exception? LastError => _lastError;

    /// <summary>Thread-safe snapshot of the stage log (for UI display).</summary>
    public StageTiming[] SnapshotLog()
    {
        lock (_logLock)
            return _log.ToArray();
    }

    public void Regenerate(AeroSpec spec, BuildMode buildMode = BuildMode.ZSliceSlabs, int zSliceCount = 24)
    {
        // Cancel any in-flight build. The old task will throw OperationCanceledException
        // at the next stage boundary (PicoGK native voxelization can't be interrupted mid-call).
        _cts?.Cancel();
        _cts = new CancellationTokenSource();
        var token = _cts.Token;
        int myEpoch = Interlocked.Increment(ref _epoch);

        // Clear state
        while (_queue.TryDequeue(out _)) { }
        _stageCount = 0;
        _lastError = null;
        lock (_logLock) _log.Clear();
        _wallClock.Restart();
        _running = true;

        int stageIndex = 0;

        _buildTask = Task.Run(() =>
        {
            try
            {
                LogLine($"=== Build #{myEpoch} started ===");
                LogLine($"  Thrust:     {spec.F_thrust / 1000f:F1} kN");
                LogLine($"  Pc:         {spec.Pc / 1e5f:F0} bar");
                LogLine($"  O/F:        {spec.OF_ratio:F2}");
                LogLine($"  Voxel size: {spec.voxelSize:F2} mm");
                LogLine($"  Channels:   {spec.channelMode}");
                LogLine($"  Build mode: {buildMode}" + (buildMode == BuildMode.ZSliceSlabs ? $" ({zSliceCount} slabs)" : ""));
                LogLine("");

                var physTimer = Stopwatch.StartNew();
                Thermochemistry.Compute(spec);
                ChamberSizing.Compute(spec);
                HeatTransfer.Compute(spec);
                HeatTransfer.DerivePortPositions(spec);
                LogLine($"  [physics] {physTimer.Elapsed.TotalSeconds:F2}s (thermo + sizing + heat transfer + ports)");

                FluidFirst.Build(spec, stage =>
                {
                    if (token.IsCancellationRequested || myEpoch != _epoch)
                        throw new OperationCanceledException();

                    float cumulativeSec = (float)_wallClock.Elapsed.TotalSeconds;
                    var timing = new StageTiming(
                        myEpoch,
                        Interlocked.Increment(ref stageIndex),
                        stage.Stage,
                        stage.Description,
                        stage.ElapsedSec,
                        cumulativeSec);

                    lock (_logLock) _log.Add(timing);

                    _queue.Enqueue(stage);
                    Interlocked.Increment(ref _stageCount);

                    LogLine($"  [{cumulativeSec,6:F2}s] +{stage.ElapsedSec,5:F2}s  {stage.Stage,-18} {stage.Description}");
                }, buildMode, zSliceCount);

                _lastBuildTotalSec = (float)_wallClock.Elapsed.TotalSeconds;
                LogLine($"=== Build #{myEpoch} FINISHED in {_lastBuildTotalSec:F2}s ===");
            }
            catch (OperationCanceledException)
            {
                LogLine($"=== Build #{myEpoch} CANCELLED at {_wallClock.Elapsed.TotalSeconds:F2}s ===");
            }
            catch (Exception ex)
            {
                if (myEpoch == _epoch)
                    _lastError = ex;
                LogLine($"=== Build #{myEpoch} FAILED: {ex.GetType().Name}: {ex.Message} ===");
                LogLine(ex.StackTrace ?? "(no stack trace)");
            }
            finally
            {
                if (myEpoch == _epoch)
                    _running = false;
            }
        });
    }

    private static readonly object _consoleLock = new();
    private void LogLine(string msg)
    {
        lock (_consoleLock)
            Console.WriteLine($"[{DateTime.Now:HH:mm:ss.fff}] {msg}");
    }

    /// <summary>
    /// Dequeue all ready stages. Main thread iterates and uploads them to GPU.
    /// Returns the count popped this frame.
    /// </summary>
    public int DrainQueue(Action<StageResult> handler)
    {
        int count = 0;
        while (_queue.TryDequeue(out var stage))
        {
            handler(stage);
            count++;
        }
        return count;
    }
}
