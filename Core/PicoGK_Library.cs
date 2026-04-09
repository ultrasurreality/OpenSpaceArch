//
// SPDX-License-Identifier: Apache-2.0
//
// PicoGK Core (headless fork) — stripped of Viewer for custom cinematic renderer.
// Original PicoGK is developed and maintained by LEAP 71 — https://leap71.com
// Licensed under Apache License 2.0.
//
// This fork removes:
//  - Library.Go() Viewer orchestrator (113 lines)
//  - oViewer() accessor + Viewer field
//  - strFindLightSetupFile helper (viewer lights)
//
// This fork keeps:
//  - Native OpenVDB kernel init via _Init() / _Destroy() P/Invoke
//  - Instance constructor for headless mode (preserved from upstream)
//  - Log(), bContinueTask(), EndTask(), vecVoxelsToMm, MmToVoxels
//  - Static InitHeadless() / Shutdown() wrappers for convenient lifecycle
//

using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;

namespace PicoGK
{
    public partial class Library : IDisposable
    {
        /// <summary>
        /// Returns the library name (from the C++ side)
        /// </summary>
        public static string strName()
        {
            StringBuilder oBuilder = new StringBuilder(Library.nStringLength);
            _GetName(oBuilder);
            return oBuilder.ToString();
        }

        /// <summary>
        /// Returns the library version (from the C++ side)
        /// </summary>
        public static string strVersion()
        {
            StringBuilder oBuilder = new StringBuilder(Library.nStringLength);
            _GetVersion(oBuilder);
            return oBuilder.ToString();
        }

        /// <summary>
        /// Returns internal build info (build date/time of the C++ library)
        /// </summary>
        public static string strBuildInfo()
        {
            StringBuilder oBuilder = new StringBuilder(Library.nStringLength);
            _GetBuildInfo(oBuilder);
            return oBuilder.ToString();
        }

        /// <summary>
        /// Initialize PicoGK kernel headless (no Viewer window).
        /// Call once at application startup. Safe to call multiple times — idempotent.
        /// </summary>
        /// <param name="voxelSizeMM">Global voxel size in mm (e.g. 0.4)</param>
        /// <param name="logFolder">Optional log folder (defaults to Documents)</param>
        public static void InitHeadless(float voxelSizeMM, string? logFolder = null)
        {
            if (m_oHeadlessInstance != null)
                return;

            if (logFolder is null || logFolder == "")
                logFolder = Utils.strDocumentsFolder();

            if (!Directory.Exists(logFolder))
                Directory.CreateDirectory(logFolder);

            string logPath = Path.Combine(logFolder, "OpenSpaceArch.log");

            lock (oMtxLog)
            {
                if (oTheLog is null)
                    oTheLog = new LogFile(logPath);
            }

            Library.strLogFolder = logFolder;

            m_oHeadlessInstance = new Library(voxelSizeMM);

            Log($"PicoGK Core headless init");
            Log($"  Name:    {strName()}");
            Log($"  Version: {strVersion()}");
            Log($"  Voxel:   {voxelSizeMM} mm");
        }

        /// <summary>
        /// Shutdown PicoGK kernel. Call at application exit.
        /// </summary>
        public static void Shutdown()
        {
            m_oHeadlessInstance?.Dispose();
            m_oHeadlessInstance = null;

            lock (oMtxLog)
            {
                oTheLog?.Dispose();
                oTheLog = null;
            }
        }

        /// <summary>
        /// Headless instance constructor (preserved from upstream PicoGK).
        /// Initializes the native OpenVDB kernel via _Init().
        /// Dispose releases native resources via _Destroy().
        /// </summary>
        public Library(float _fVoxelSizeMM)
        {
            lock (mtxRunOnce)
            {
                if (bRunning)
                    throw new Exception("PicoGK only supports running one library config at one time");

                bRunning = true;
            }

            TestAssumptions();

            Debug.Assert(_fVoxelSizeMM > 0f);
            fVoxelSizeMM = _fVoxelSizeMM;

            try
            {
                _Init(fVoxelSizeMM);
            }
            catch (Exception)
            {
                throw new Exception(
                    $"Failed to load PicoGK Runtime. Make sure {Config.strPicoGKLib}.dll is accessible.");
            }
        }

        /// <summary>
        /// Checks whether the current task should continue.
        /// Preserved for compat with engine code that may call this.
        /// </summary>
        public static bool bContinueTask(bool bAppExitOnly = false)
        {
            return !m_bAppExit && (bAppExitOnly || m_bContinueTask);
        }

        public static void EndTask() => m_bContinueTask = false;
        public static void CancelEndTaskRequest() => m_bContinueTask = true;

        /// <summary>
        /// Signals that the app is about to exit (called by custom viewer on window close).
        /// </summary>
        public static void MarkAppExit() => m_bAppExit = true;

        static bool m_bAppExit = false;
        static bool m_bContinueTask = true;

        /// <summary>
        /// Thread-safe logging. If the log file is not open, falls back to Console.
        /// </summary>
        public static void Log(in string strFormat, params object[] args)
        {
            lock (oMtxLog)
            {
                if (oTheLog == null)
                {
                    Console.WriteLine(strFormat, args);
                    return;
                }

                oTheLog.Log(strFormat, args);
            }
        }

        /// <summary>
        /// Internal: tests that data types have the memory layout we assume
        /// so we don't run into interop issues with the C++ side.
        /// </summary>
        private static void TestAssumptions()
        {
            Vector3 vec3 = new();
            Vector2 vec2 = new();
            Matrix4x4 mat4 = new();
            Coord xyz = new(0, 0, 0);
            Triangle tri = new(0, 0, 0);
            ColorFloat clr = new(0f);
            BBox2 oBB2 = new();
            BBox3 oBB3 = new();

            Debug.Assert(sizeof(bool) == 1);
            Debug.Assert(Marshal.SizeOf(vec3) == ((32 * 3) / 8));
            Debug.Assert(Marshal.SizeOf(vec2) == ((32 * 2) / 8));
            Debug.Assert(Marshal.SizeOf(mat4) == ((32 * 16) / 8));
            Debug.Assert(Marshal.SizeOf(xyz) == ((32 * 3) / 8));
            Debug.Assert(Marshal.SizeOf(tri) == ((32 * 3) / 8));
            Debug.Assert(Marshal.SizeOf(clr) == ((32 * 4) / 8));
            Debug.Assert(Marshal.SizeOf(oBB2) == ((32 * 2 * 2) / 8));
            Debug.Assert(Marshal.SizeOf(oBB3) == ((32 * 3 * 2) / 8));
        }

        public static Vector3 vecVoxelsToMm(int x, int y, int z)
        {
            Vector3 vecMm = new();
            Vector3 vecVoxels = new Vector3((float)x, (float)y, (float)z);
            _VoxelsToMm(in vecVoxels, ref vecMm);
            return vecMm;
        }

        public static void MmToVoxels(Vector3 vecMm, out int x, out int y, out int z)
        {
            Vector3 vecResult = Vector3.Zero;
            _VoxelsToMm(in vecMm, ref vecResult);
            x = (int)(vecResult.X + 0.5f);
            y = (int)(vecResult.Y + 0.5f);
            z = (int)(vecResult.Z + 0.5f);
        }

        public static float fVoxelSizeMM = 0.0f;
        public static string strLogFolder = "";
        public static string strSrcFolder = "";

        private static readonly object oMtxLog = new object();
        private static LogFile? oTheLog = null;

        private static readonly object mtxRunOnce = new object();
        private static bool bRunning = false;
        private static Library? m_oHeadlessInstance = null;

        ~Library()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool bDisposing)
        {
            if (m_bDisposed)
                return;

            if (bDisposing)
            {
                _Destroy();
            }

            lock (mtxRunOnce)
            {
                if (bRunning)
                    bRunning = false;
            }

            m_bDisposed = true;
        }

        bool m_bDisposed = false;
    }
}
