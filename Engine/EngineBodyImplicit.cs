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

// EngineBodyImplicit.cs — single-SDF engine core (ALTERNATIVE DESIGN, NOT WIRED IN)
//
// STATUS (2026-06-03): NOT part of the production build. EngineAssembly.Build →
// FluidFirst.Build, which composes the body from per-group Voxels with booleans.
// This class is the "one fused SDF for the whole core" idea, kept as a reference /
// future path. It is currently not instantiated anywhere. If you reactivate it,
// have EngineAssembly construct it (shroud + spike RevolutionSDFs + the two channel
// fields) and render once over GetBBox().
//
// v7 VOIDS-FIRST formula (2026-04-12):
//   dVoid = min(dGas, dChShroud, dChSpike)   -- union ALL fluid voids
//   solid = max(-dVoid, dVoid - wallT)        -- shell of wallT around union
//
// Philosophy: geometry EMERGES from fluid voids + wall thickness.
// No predetermined outer profile. No shellT estimation.
// Channels are PART of the void union, not carved from a shell.
// Wall wraps everything uniformly. Outer surface = consequence.
//
// This is the HelixHeatX pattern expressed as IImplicit:
//   voxOuter = voxInner.voxOffset(wallT)
//   voxSolid = voxOuter - voxInner
// ...but evaluated analytically per-voxel without intermediate voxelization.

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public class EngineBodyImplicit : IImplicit
{
    readonly AeroSpec _S;

    // Gas path boundaries (analytical SDFs)
    readonly RevolutionSDF _shroud;
    readonly RevolutionSDF _spike;

    // Cooling channel fields (unioned into void, not subtracted from shell)
    readonly List<IImplicit> _channelFields = new();

    // Pre-sampled z-dependent data
    readonly float[] _wallT;     // wall thickness (Barlow pressure formula)
    readonly int _nSamples;
    readonly float _zStart, _zEnd, _zStep;

    // Bounding box
    readonly BBox3 _bbox;

    public EngineBodyImplicit(
        AeroSpec S,
        RevolutionSDF shroud,
        RevolutionSDF spike,
        IImplicit? channelsShroud,
        IImplicit? channelsSpike)
    {
        _S = S;
        _shroud = shroud;
        _spike = spike;

        if (channelsShroud != null)
            _channelFields.Add(channelsShroud);
        if (channelsSpike != null)
            _channelFields.Add(channelsSpike);

        // Pre-sample wall thickness (Barlow only — no shellT estimation)
        _zStart = S.zTip;
        _zEnd = S.zInjector + 5f;
        _nSamples = 2000;
        _zStep = (_zEnd - _zStart) / (_nSamples - 1);
        _wallT = new float[_nSamples];

        for (int i = 0; i < _nSamples; i++)
        {
            float z = _zStart + i * _zStep;
            float wall = HeatTransfer.WallThickness(S, z);

            // Structural reinforcement zones
            if (z > S.zInjector - 3f) wall *= 1.5f;
            if (z < S.zTip + 3f) wall *= 1.3f;
            wall = MathF.Max(wall, S.minPrintWall);
            // Extra margin for smoothing erosion + outer wall coverage
            wall += S.minPrintWall;
            _wallT[i] = wall;
        }

        // Compute bounding box — use channel extent, not analytical shellT
        float maxR = 0f;
        for (float z = _zStart; z <= _zEnd; z += 1f)
        {
            float rSh = ChamberSizing.ShroudProfile(S, z);
            float wall = LerpWall(z);
            var (_, hS) = HeatTransfer.ChannelRect(S, z);
            maxR = MathF.Max(maxR, rSh + wall + hS + wall + 3f);
        }
        _bbox = new BBox3(
            new Vector3(-maxR, -maxR, _zStart - 2f),
            new Vector3( maxR,  maxR, _zEnd + 2f));
    }

    float LerpWall(float z)
    {
        float t = (z - _zStart) / _zStep;
        int i = (int)t;
        if (i < 0) return _wallT[0];
        if (i >= _nSamples - 1) return _wallT[_nSamples - 1];
        float f = t - i;
        return _wallT[i] + f * (_wallT[i + 1] - _wallT[i]);
    }

    public float fSignedDistance(in Vector3 v)
    {
        // VOIDS-FIRST: union all fluid voids, wrap wallT around the union.
        //
        // 1. Gas path = annular void (negative inside gas)
        float dShroud = _shroud.fSignedDistance(v);
        float dSpike  = _spike.fSignedDistance(v);
        float dGas    = MathF.Max(dShroud, -dSpike);

        // 2. Union gas with channel voids, WITH mutual exclusion.
        //    Each channel is clipped so it can't be closer than wallT to gas.
        //    This is HelixHeatX's voxHot -= voxCold.voxOffset(minWall) as IImplicit.
        float wallT = LerpWall(v.Z);
        float dVoid = dGas;
        for (int i = 0; i < _channelFields.Count; i++)
        {
            float dCh = _channelFields[i].fSignedDistance(v);
            float dChClipped = MathF.Max(dCh, wallT - dGas); // mutual exclusion
            dVoid = MathF.Min(dVoid, dChClipped);
        }

        // 3. Shell: solid where 0 < dVoid < wallT
        //    Outer surface = consequence of void geometry + wall thickness.
        return MathF.Max(-dVoid, dVoid - wallT);
    }

    public BBox3 GetBBox() => _bbox;
}
