//
// SPDX-License-Identifier: Apache-2.0
//
// PicoGK ("peacock") is a compact software kernel for computational geometry,
// specifically for use in Computational Engineering Models (CEM).
//
// For more information, please visit https://picogk.org
//
// PicoGK is developed and maintained by LEAP 71 - © 2023-2026 by LEAP 71
// https://leap71.com
//
// Computational Engineering will profoundly change our physical world in the
// years ahead. Thank you for being part of the journey.
//
// We have developed this library to be used widely, for both commercial and
// non-commercial projects alike. Therefore, we have released it under a
// permissive open-source license.
//
// The foundation of PicoGK is a thin layer on top of the powerful open-source
// OpenVDB project, which in turn uses many other Free and Open Source Software
// libraries. We are grateful to be able to stand on the shoulders of giants.
//
// LEAP 71 licenses this file to you under the Apache License, Version 2.0
// (the "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, THE SOFTWARE IS
// PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
//
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Headless fork note: `public partial class Viewer` P/Invoke block removed.
// Voxels/Mesh/Lattice/PolyLine interop retained — this is the core engine.
//

using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;

namespace PicoGK
{
    // private interfaces to external PicoGK Runtime library

    public partial class Library
    {
        public const int nStringLength = 255;

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Library_Init", CharSet = CharSet.Ansi)]
        private static extern void _Init(float fVoxelSizeMM);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Library_Destroy", CharSet = CharSet.Ansi)]
        private static extern void _Destroy();

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Library_GetName", CharSet = CharSet.Ansi)]
        private static extern void _GetName(StringBuilder psz);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Library_GetVersion", CharSet = CharSet.Ansi)]
        private static extern void _GetVersion(StringBuilder psz);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Library_GetBuildInfo", CharSet = CharSet.Ansi)]
        private static extern void _GetBuildInfo(StringBuilder psz);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Library_VoxelsToMm")]
        private static extern void _VoxelsToMm( in  Vector3 vecVoxelCoordinate,
                                                ref Vector3 vecMmCoordinate);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Library_MmToVoxels")]
        private static extern void _MmToVoxels( in  Vector3 vecMmCoordinate,
                                                ref Vector3 vecVoxelCoordinate);
    }

    public partial class Mesh : IDisposable
    {
        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_hCreate")]
        internal static extern IntPtr _hCreate();

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_hCreateFromVoxels")]
        internal static extern IntPtr _hCreateFromVoxels(IntPtr hVoxels);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_bIsValid")]
        private static extern bool _IsValid(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_Destroy")]
        private static extern void _Destroy(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_nAddVertex")]
        private static extern int _nAddVertex(IntPtr hThis,
                                                in Vector3 V);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_nVertexCount")]
        private static extern int _nVertexCount(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_GetVertex")]
        private static extern void _GetVertex(IntPtr hThis,
                                                int nVertex,
                                                ref Vector3 V);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_nAddTriangle")]
        private static extern int _nAddTriangle(IntPtr hThis,
                                                    in Triangle T);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_nTriangleCount")]
        private static extern int _nTriangleCount(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_GetTriangle")]
        private static extern void _GetTriangle(    IntPtr hThis,
                                                    int nTriangle,
                                                    ref Triangle T);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_GetTriangleV")]
        private static extern void _GetTriangleV(   IntPtr hThis,
                                                    int nTriangle,
                                                    ref Vector3 vecA,
                                                    ref Vector3 vecB,
                                                    ref Vector3 vecC);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Mesh_GetBoundingBox")]
        private static extern void _GetBoundingBox( IntPtr hThis,
                                                    ref BBox3 oBBox);

        // Dispose Pattern

        ~Mesh()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            // Dispose of unmanaged resources.
            Dispose(true);
            // Suppress finalization.
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool bDisposing)
        {
            if (m_bDisposed)
            {
                return;
            }

            if (bDisposing)
            {
                // dispose managed state (managed objects).
                // Nothing to do in this class
            }

            if (m_hThis != IntPtr.Zero)
            {
                _Destroy(m_hThis);
                m_hThis = IntPtr.Zero;
            }

            m_bDisposed = true;
        }

        bool            m_bDisposed = false;
        internal IntPtr m_hThis     = IntPtr.Zero;

    }

    public partial class Lattice : IDisposable
    {
        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Lattice_hCreate")]
        internal static extern IntPtr _hCreate();

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Lattice_bIsValid")]
        private static extern bool _bIsValid(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Lattice_Destroy")]
        private static extern void _Destroy(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Lattice_AddSphere")]
        private static extern void _AddSphere(  IntPtr      hThis,
                                                in Vector3  vecCenter,
                                                float       fRadius);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Lattice_AddBeam")]
        private static extern void _AddBeam(    IntPtr      hThis,
                                                in Vector3  vecA,
                                                in Vector3  vecB,
                                                float       fRadiusA,
                                                float       fRadiusB,
                                                bool        bRoundCap);

        // Dispose Pattern

        ~Lattice()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            // Dispose of unmanaged resources.
            Dispose(true);
            // Suppress finalization.
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool bDisposing)
        {
            if (m_bDisposed)
            {
                return;
            }

            if (bDisposing)
            {
                // dispose managed state (managed objects).
                // Nothing to do in this class
            }

            if (m_hThis != IntPtr.Zero)
            {
                _Destroy(m_hThis);
                m_hThis = IntPtr.Zero;
            }

            m_bDisposed = true;
        }

        bool            m_bDisposed = false;
        internal IntPtr m_hThis     = IntPtr.Zero;
    }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    delegate void CallbackScalarFieldTraverse(in Vector3 vec, float fValue);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    delegate void CallbackVectorFieldTraverse(in Vector3 vec, in Vector3 vecValue);

    public partial class Voxels : IDisposable
    {
        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_hCreate")]
        internal static extern IntPtr _hCreate();

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_hCreateCopy")]
        internal static extern IntPtr _hCreateCopy(IntPtr hSource);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_bIsValid")]
        private static extern bool _bIsValid(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_Destroy")]
        private static extern void _Destroy(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_BoolAdd")]
        private static extern void _BoolAdd(    IntPtr hThis,
                                                IntPtr hOther);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_BoolSubtract")]
        private static extern void _BoolSubtract(   IntPtr hThis,
                                                    IntPtr hOther);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_BoolIntersect")]
        private static extern void _BoolIntersect(  IntPtr hThis,
                                                    IntPtr hOther);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_BoolAddSmooth")]
        private static extern void _BoolAddSmooth(  IntPtr hThis,
                                                    IntPtr hOther,
                                                    float fSmoothDistance);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_Offset")]
        private static extern void _Offset( IntPtr hThis,
                                            float fOffset);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_DoubleOffset")]
        private static extern void _DoubleOffset(   IntPtr hThis,
                                                    float fOffset1,
                                                    float fOffset2);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_TripleOffset")]
        private static extern void _TripleOffset(   IntPtr hThis,
                                                    float fOffset);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_Gaussian")]
        private static extern void _Gaussian( IntPtr hThis,
                                            float fDistance);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_Median")]
        private static extern void _Median( IntPtr hThis,
                                            float fDistance);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_Mean")]
        private static extern void _Mean( IntPtr hThis,
                                          float fDistance);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_RenderMesh")]
        private static extern void _RenderMesh( IntPtr hThis,
                                                IntPtr hMesh);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate float CallbackImplicitDistance(in Vector3 vec);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_RenderImplicit")]
        private static extern void _RenderImplicit( IntPtr hThis,
                                                    in BBox3 oBounds,
                                                    CallbackImplicitDistance Callback);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_IntersectImplicit")]
        private static extern void _IntersectImplicit(  IntPtr hThis,
                                                        CallbackImplicitDistance Callback);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_RenderLattice")]
        private static extern void _RenderLattice(  IntPtr hThis,
                                                    IntPtr hLattice);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_ProjectZSlice")]
        private static extern void _ProjectZSlice(  IntPtr hThis,
                                                    float fStartX,
                                                    float fEndX);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_bIsInside")]
        private static extern bool _bIsInside(      IntPtr hThis,
                                                    in Vector3 vecTestPoint);


        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_bIsEqual")]
        private static extern bool _bIsEqual(   IntPtr hThis,
                                                IntPtr hOther);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_CalculateProperties")]
        private extern static void _CalculateProperties(    IntPtr hThis,
                                                            ref float pfVolume,
                                                            ref BBox3 oBBox);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_GetSurfaceNormal")]
        private extern static void _GetSurfaceNormal(IntPtr hThis,
                                                        in  Vector3 vecSurfacePoint,
                                                        ref Vector3 vecSurfaceNormal);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_bClosestPointOnSurface")]
        private extern static bool _bClosestPointOnSurface( IntPtr hThis,
                                                            in  Vector3 vecSearch,
                                                            ref Vector3 vecSurfacePoint);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_bRayCastToSurface")]
        private extern static bool _bRayCastToSurface(  IntPtr hThis,
                                                        in  Vector3 vecSearch,
                                                        in  Vector3 vecNormal,
                                                        ref Vector3 vecSurfacePoint);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_GetVoxelDimensions")]
        private extern static void _GetVoxelDimensions( IntPtr hThis,
                                                        ref int nXOrigin,
                                                        ref int nYOrigin,
                                                        ref int nZOrigin,
                                                        ref int nXSize,
                                                        ref int nYSize,
                                                        ref int nZSize);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_GetSlice")]
        private extern static void _GetVoxelSlice(  IntPtr hThis,
                                                    int nZSlice,
                                                    IntPtr afBuffer,
                                                    ref float fBackgroundValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Voxels_GetInterpolatedSlice")]
        private extern static void _GetInterpolatedVoxelSlice(  IntPtr hThis,
                                                                float fZSlice,
                                                                IntPtr afBuffer,
                                                                ref float fBackgroundValue);

        // Dispose Pattern

        ~Voxels()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            // Dispose of unmanaged resources.
            Dispose(true);
            // Suppress finalization.
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool bDisposing)
        {
            if (m_bDisposed)
            {
                return;
            }

            if (bDisposing)
            {
                // dispose managed state (managed objects).
                // Nothing to do in this class
            }

            if (m_hThis != IntPtr.Zero)
            {
                _Destroy(m_hThis);
                m_hThis = IntPtr.Zero;
            }

            m_bDisposed = true;
        }

        bool            m_bDisposed = false;
        internal IntPtr m_hThis     = IntPtr.Zero;
    }

    public partial class PolyLine : IDisposable
    {
        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "PolyLine_hCreate")]
        public static extern IntPtr _hCreate(in ColorFloat clr);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "PolyLine_bIsValid")]
        private static extern bool _bIsValid(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "PolyLine_Destroy")]
        private static extern void _Destroy(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "PolyLine_nAddVertex")]
        private static extern int _nAddVertex(  IntPtr hThis,
                                                in Vector3 vec);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "PolyLine_nVertexCount")]
        private static extern int _nVertexCount(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "PolyLine_GetVertex")]
        private static extern void _GetVertex(  IntPtr hThis,
                                                int nIndex,
                                                ref Vector3 vec);


        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "PolyLine_GetColor")]
        private static extern void _GetColor(   IntPtr hThis,
                                                ref ColorFloat clr);

        // Dispose Pattern

        ~PolyLine()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            // Dispose of unmanaged resources.
            Dispose(true);
            // Suppress finalization.
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool bDisposing)
        {
            if (m_bDisposed)
            {
                return;
            }

            if (bDisposing)
            {
                // dispose managed state (managed objects).
                // Nothing to do in this class
            }

            if (m_hThis != IntPtr.Zero)
            {
                _Destroy(m_hThis);
                m_hThis = IntPtr.Zero;
            }

            m_bDisposed = true;
        }

        bool            m_bDisposed = false;
        internal IntPtr m_hThis     = IntPtr.Zero;
    }

    // `public partial class Viewer` P/Invoke block removed in headless fork.
    // Original block contained Viewer_hCreate, Viewer_bPoll, Viewer_AddMesh,
    // Viewer_SetGroupVisible, etc. — all unused by custom cinematic renderer.

    public partial class ScalarField : IDisposable
    {
        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_hCreate")]
        public static extern IntPtr _hCreate();

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_hCreateCopy")]
        public static extern IntPtr _hCreateCopy(IntPtr hSource);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_hCreateFromVoxels")]
        public static extern IntPtr _hCreateFromVoxels(IntPtr hVoxels);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_hBuildFromVoxels")]
        public static extern IntPtr _hBuildFromVoxels(IntPtr hVoxels, float fScalarValue, float fSdThreshold);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_bIsValid")]
        private static extern bool _bIsValid(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_Destroy")]
        private static extern void _Destroy(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_SetValue")]
        private static extern void _SetValue(   IntPtr hThis,
                                                in Vector3 vecPosition,
                                                float fValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_bGetValue")]
        private static extern bool _bGetValue(   IntPtr hThis,
                                                 in  Vector3 vecPosition,
                                                 ref float fValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_RemoveValue")]
        private static extern bool _RemoveValue(    IntPtr hThis,
                                                    in  Vector3 vecPosition);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_GetVoxelDimensions")]
        private extern static void _GetVoxelDimensions( IntPtr hThis,
                                                        ref int nXOrigin,
                                                        ref int nYOrigin,
                                                        ref int nZOrigin,
                                                        ref int nXSize,
                                                        ref int nYSize,
                                                        ref int nZSize);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_GetSlice")]
        private extern static void _GetVoxelSlice(  IntPtr hThis,
                                                    int nZSlice,
                                                    IntPtr afBuffer);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ScalarField_TraverseActive")]
        private extern static void _TraverseActive( IntPtr hThis,
                                                    CallbackScalarFieldTraverse pfn);

        ~ScalarField() { Dispose(false); }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool bDisposing)
        {
            if (m_bDisposed) return;
            if (m_hThis != IntPtr.Zero)
            {
                _Destroy(m_hThis);
                m_hThis = IntPtr.Zero;
            }
            m_bDisposed = true;
        }

        bool m_bDisposed = false;
        internal IntPtr m_hThis = IntPtr.Zero;
    }

    public partial class VectorField : IDisposable
    {
        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_hCreate")]
        public static extern IntPtr _hCreate();

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_hCreateCopy")]
        public static extern IntPtr _hCreateCopy(IntPtr hSource);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_hCreateFromVoxels")]
        public static extern IntPtr _hCreateFromVoxels(IntPtr hVoxels);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_hBuildFromVoxels")]
        public static extern IntPtr _hBuildFromVoxels(IntPtr hVoxels, in Vector3 vecValue, float fSDThreshold);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_bIsValid")]
        private static extern bool _bIsValid(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_Destroy")]
        private static extern void _Destroy(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_SetValue")]
        private static extern void _SetValue(   IntPtr hThis,
                                                in Vector3 vecPosition,
                                                in Vector3 vecValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_bGetValue")]
        private static extern bool _bGetValue(   IntPtr hThis,
                                                 in  Vector3 vecPosition,
                                                 ref Vector3 vecValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_RemoveValue")]
        private static extern bool _RemoveValue(    IntPtr hThis,
                                                    in  Vector3 vecPosition);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "VectorField_TraverseActive")]
        private extern static void _TraverseActive( IntPtr hThis,
                                                    CallbackVectorFieldTraverse pfn);

        ~VectorField() { Dispose(false); }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool bDisposing)
        {
            if (m_bDisposed) return;
            if (m_hThis != IntPtr.Zero)
            {
                _Destroy(m_hThis);
                m_hThis = IntPtr.Zero;
            }
            m_bDisposed = true;
        }

        bool m_bDisposed = false;
        internal IntPtr m_hThis = IntPtr.Zero;
    }

    public partial class FieldMetadata
    {
        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_hFromVoxels")]
        internal static extern IntPtr _hFromVoxels(IntPtr hVoxels);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_hFromScalarField")]
        internal static extern IntPtr _hFromScalarField(IntPtr hScalarField);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_hFromVectorField")]
        internal static extern IntPtr _hFromVectorField(IntPtr hVectorField);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_Destroy")]
        private static extern void _Destroy(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_nCount")]
        private static extern int _nCount(IntPtr hThis);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_nNameLengthAt")]
        private static extern int _nNameLengthAt(IntPtr hThis, int nIndex);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_bGetNameAt", CharSet = CharSet.Ansi)]
        private static extern bool _bGetNameAt( IntPtr hThis, int nIndex, StringBuilder pszValueName, int nMaxStringLen);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_nTypeAt", CharSet = CharSet.Ansi)]
        private static extern int _nTypeAt(IntPtr hThis, string strName);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_nStringLengthAt", CharSet = CharSet.Ansi)]
        private static extern int _nStringLengthAt( IntPtr hThis, string strFieldName);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_bGetStringAt", CharSet = CharSet.Ansi)]
        private static extern bool _bGetStringAt(IntPtr hThis, string strFieldName, StringBuilder pszValue, int nMaxStringLen);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_bGetFloatAt", CharSet = CharSet.Ansi)]
        private static extern bool _bGetFloatAt(IntPtr hThis, string strFieldName, ref float fValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_bGetVectorAt", CharSet = CharSet.Ansi)]
        private static extern bool _bGetVectorAt(IntPtr hThis, string strFieldName, ref Vector3 vecValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_SetStringValue", CharSet = CharSet.Ansi)]
        private static extern void _SetStringValue(IntPtr hThis, string strFieldName, string strValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_SetFloatValue", CharSet = CharSet.Ansi)]
        private static extern void _SetFloatValue(IntPtr hThis, string strFieldName, float fValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "Metadata_SetVectorValue", CharSet = CharSet.Ansi)]
        private static extern void _SetVectorValue(IntPtr hThis, string strFieldName, in Vector3 vecValue);

        [DllImport(Config.strPicoGKLib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "MetaData_RemoveValue", CharSet = CharSet.Ansi)]
        private static extern void _RemoveValue(IntPtr hThis, string strFieldName);

        ~FieldMetadata() { Dispose(false); }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool bDisposing)
        {
            if (m_bDisposed) return;
            if (m_hThis != IntPtr.Zero)
            {
                _Destroy(m_hThis);
                m_hThis = IntPtr.Zero;
            }
            m_bDisposed = true;
        }

        bool m_bDisposed = false;
        internal IntPtr m_hThis = IntPtr.Zero;
    }
}
