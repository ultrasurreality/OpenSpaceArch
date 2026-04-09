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
