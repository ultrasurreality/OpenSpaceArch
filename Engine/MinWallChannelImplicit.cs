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

// MinWallChannelImplicit.cs — v5: SDF-based mutual exclusion
//
// ORPHANED 2026-04-06: retained for potential v5 implicit-mode reactivation.
// Currently unreferenced after dead-code cleanup of FluidFirst.cs.
//
// Replaces voxOffset-subtract with analytical field composition:
//   max(dChannel, minWall(z) - dChamber)
//
// A point is "inside channel" only if it's also at least minWall
// away from the chamber surface. Per-voxel wall thickness from Barlow.

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public class MinWallChannelImplicit : IImplicit
{
    readonly ChannelFieldImplicit _channels;
    readonly IImplicit _chamberShroud;   // RevolutionSDF for shroud gas-side
    readonly IImplicit _chamberSpike;    // RevolutionSDF for spike gas-side
    readonly AeroSpec _S;
    readonly bool _isShroud;

    public MinWallChannelImplicit(
        ChannelFieldImplicit channels,
        IImplicit chamberShroud,
        IImplicit chamberSpike,
        AeroSpec S,
        bool isShroud)
    {
        _channels = channels;
        _chamberShroud = chamberShroud;
        _chamberSpike = chamberSpike;
        _S = S;
        _isShroud = isShroud;
    }

    public float fSignedDistance(in Vector3 v)
    {
        float dChannel = _channels.fSignedDistance(v);

        // Per-voxel wall thickness from Barlow pressure formula
        float minWall = MathF.Max(
            HeatTransfer.WallThickness(_S, v.Z),
            MathF.Max(_S.minPrintWall, 0.8f));

        // Distance to chamber surface (negative = inside chamber)
        // For shroud channels: must be away from shroud inner surface
        // For spike channels: must be away from spike outer surface
        float dChamber;
        if (_isShroud)
            dChamber = _chamberShroud.fSignedDistance(v);
        else
            dChamber = -_chamberSpike.fSignedDistance(v); // flip sign: inside spike = away from gas

        // Channel exists only where it's far enough from chamber
        // dChamber > 0 means outside the chamber body (where channels live)
        // We need dChamber >= minWall
        return MathF.Max(dChannel, minWall - dChamber);
    }
}
