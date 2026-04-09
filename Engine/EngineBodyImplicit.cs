// EngineBodyImplicit.cs — ONE SDF for the entire engine core
//
// Replaces the 9-voxel + boolean pipeline with a single IImplicit.
// Chamber void + cooling channels + variable wall = ONE equation.
// No voxel booleans. No OverOffset. No information loss.
//
// Formula: dSolid = max(-dVoid, dVoid - shellT)
//   dVoid < 0 → inside void → not solid
//   0 < dVoid < shellT → in wall → SOLID
//   dVoid > shellT → outside → not solid
//
// Insight source: LEAP 71 video analysis (2026-04-01) — confirmed that
// chamber wall + channels appear SIMULTANEOUSLY as ONE evaluated field.

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public class EngineBodyImplicit : IImplicit
{
    readonly AeroSpec _S;

    // Gas path boundaries (analytical SDFs)
    readonly RevolutionSDF _shroud;
    readonly RevolutionSDF _spike;

    // Cooling voids: each is (channel field, reference SDF, sign for "inside" vs "outside")
    private readonly record struct CoolingVoid(IImplicit Field, IImplicit RefSdf, float Sign);
    readonly List<CoolingVoid> _voids = new();

    // Pre-sampled z-dependent data
    readonly float[] _wallT;     // inner wall thickness (Barlow)
    readonly float[] _shellT;    // total shell thickness (wall + channels + outer)
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
            _voids.Add(new CoolingVoid(channelsShroud, shroud, +1f));   // outside shroud surface
        if (channelsSpike != null)
            _voids.Add(new CoolingVoid(channelsSpike, spike, -1f));     // inside spike surface

        // Pre-sample wall and shell thickness
        _zStart = S.zTip;
        _zEnd = S.zInjector + 5f;
        _nSamples = 2000;
        _zStep = (_zEnd - _zStart) / (_nSamples - 1);
        _wallT = new float[_nSamples];
        _shellT = new float[_nSamples];

        for (int i = 0; i < _nSamples; i++)
        {
            float z = _zStart + i * _zStep;

            // Inner wall from Barlow pressure formula
            float wall = HeatTransfer.WallThickness(S, z);

            // Structural reinforcement zones
            if (z > S.zInjector - 3f) wall *= 1.5f;
            if (z < S.zTip + 3f) wall *= 1.3f;
            wall = MathF.Max(wall, S.minPrintWall);
            _wallT[i] = wall;

            // Shell = inner wall + channel depth + outer wall
            float chDepth = 0f;
            if (z >= S.zCowl && z <= S.zInjector)
            {
                var (_, hS) = HeatTransfer.ChannelRect(S, z);
                var (_, hP) = HeatTransfer.ChannelRectSpike(S, z);
                chDepth = MathF.Max(hS, hP);
            }
            _shellT[i] = wall + chDepth + S.minPrintWall + 0.5f;
        }

        // Compute bounding box
        float maxR = 0f;
        for (float z = _zStart; z <= _zEnd; z += 1f)
        {
            float rSh = ChamberSizing.ShroudProfile(S, z);
            float shell = LerpShell(z);
            maxR = MathF.Max(maxR, rSh + shell + 3f);
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

    float LerpShell(float z)
    {
        float t = (z - _zStart) / _zStep;
        int i = (int)t;
        if (i < 0) return _shellT[0];
        if (i >= _nSamples - 1) return _shellT[_nSamples - 1];
        float f = t - i;
        return _shellT[i] + f * (_shellT[i + 1] - _shellT[i]);
    }

    public float fSignedDistance(in Vector3 v)
    {
        // 1. Gas path = annular void between shroud and spike
        float dShroud = _shroud.fSignedDistance(v);
        float dSpike  = _spike.fSignedDistance(v);
        float dGas    = MathF.Max(-dShroud, dSpike);

        // 2. Cooling channels with per-void min-wall exclusion
        //    For each void: channel field clipped by `wallT - dRef`, where
        //    dRef is the distance OUTSIDE its reference SDF (sign-flipped for inside)
        float wallT = LerpWall(v.Z);
        float dVoid = dGas;
        for (int i = 0; i < _voids.Count; i++)
        {
            var cv = _voids[i];
            float dRef  = cv.RefSdf.fSignedDistance(v) * cv.Sign;
            float dCh   = cv.Field.fSignedDistance(v);
            float dGood = MathF.Max(dCh, wallT - dRef);
            dVoid       = MathF.Min(dVoid, dGood);
        }

        // 3. Shell: solid where 0 < dVoid < shellThickness
        float shellT = LerpShell(v.Z);
        return MathF.Max(-dVoid, dVoid - shellT);
    }

    public BBox3 GetBBox() => _bbox;
}
