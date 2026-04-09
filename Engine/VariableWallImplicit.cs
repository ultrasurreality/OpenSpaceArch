// VariableWallImplicit.cs — Per-voxel wall thickness from Barlow
//
// Replaces global voxOffset(wallThickness) in Step 4.
// Wall = f(z, r) from pressure formula. Thicker at high pressure, thinner where cooling works.
//
// Uses ScalarField(Voxels) for O(1) SDF queries (OpenVDB tree lookup).

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public class VariableWallImplicit : IImplicit
{
    readonly ScalarField _sdfVoids;  // SDF of all combined voids
    readonly AeroSpec _S;

    // Pre-sampled wall thickness
    readonly float[] _wallByZ;
    readonly int _nSamples;
    readonly float _zStart, _zEnd, _zStep;

    public VariableWallImplicit(Voxels voxAllVoids, AeroSpec S)
    {
        _sdfVoids = new ScalarField(voxAllVoids); // O(1) SDF queries via OpenVDB
        _S = S;

        _zStart = S.zTip;
        _zEnd = S.zTotal;
        _nSamples = 2000;
        _zStep = (_zEnd - _zStart) / (_nSamples - 1);
        _wallByZ = new float[_nSamples];

        for (int i = 0; i < _nSamples; i++)
        {
            float z = _zStart + i * _zStep;
            _wallByZ[i] = HeatTransfer.WallThickness(S, z);
        }
    }

    float WallAtZ(float z)
    {
        float t = (z - _zStart) / _zStep;
        int i = (int)t;
        if (i < 0) return _wallByZ[0];
        if (i >= _nSamples - 1) return _wallByZ[_nSamples - 1];
        float frac = t - i;
        return _wallByZ[i] + frac * (_wallByZ[i + 1] - _wallByZ[i]);
    }

    public float fSignedDistance(in Vector3 v)
    {
        // Query SDF of voids via ScalarField
        // bGetValue returns false if point is outside VDB narrow band
        if (!_sdfVoids.bGetValue(v, out float sdfRaw))
            return 5f; // outside tracked region → definitely outside shell

        float dVoid = sdfRaw * Library.fVoxelSizeMM; // convert to mm

        // Wall thickness at this point
        float wall = WallAtZ(v.Z);

        // Extra thickness at structural zones
        if (v.Z > _S.zInjector - 3f) wall *= 1.5f;
        if (v.Z < _S.zTip + 3f) wall *= 1.3f;

        wall = MathF.Max(wall, _S.minPrintWall);

        // Outer shell: inside if within 'wall' distance of void surface
        return dVoid - wall;
    }
}
