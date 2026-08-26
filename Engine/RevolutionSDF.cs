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

using PicoGK;
using System.Numerics;

namespace OpenSpaceArch.Engine;

/// <summary>
/// Implicit signed distance field for a body of revolution.
/// Given r(z), creates a perfect solid of revolution with flat end caps.
/// </summary>
public class RevolutionSDF : IImplicit
{
    readonly float[] _r;
    readonly float _zMin, _zMax, _step;
    readonly int _n;

    public RevolutionSDF(Func<float, float> radiusFunc, float zMin, float zMax, int samples = 2000)
    {
        _zMin = zMin; _zMax = zMax; _n = samples;
        _step = (zMax - zMin) / (samples - 1);
        _r = new float[samples];
        for (int i = 0; i < samples; i++)
            _r[i] = radiusFunc(zMin + i * _step);
    }

    float R(float z)
    {
        float t = (z - _zMin) / _step;
        int i = (int)t;
        if (i < 0) return _r[0];
        if (i >= _n - 1) return _r[_n - 1];
        return _r[i] + (t - i) * (_r[i + 1] - _r[i]);
    }

    public float fSignedDistance(in Vector3 v)
    {
        float rxy = MathF.Sqrt(v.X * v.X + v.Y * v.Y);
        if (v.Z >= _zMin && v.Z <= _zMax) return rxy - R(v.Z);
        if (v.Z < _zMin)
        {
            float r0 = _r[0];
            if (rxy <= r0) return _zMin - v.Z;
            return MathF.Sqrt((rxy - r0) * (rxy - r0) + (_zMin - v.Z) * (_zMin - v.Z));
        }
        float rn = _r[_n - 1];
        if (rxy <= rn) return v.Z - _zMax;
        return MathF.Sqrt((rxy - rn) * (rxy - rn) + (v.Z - _zMax) * (v.Z - _zMax));
    }
}
