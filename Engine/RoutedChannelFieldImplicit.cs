// RoutedChannelFieldImplicit.cs — v5b: IImplicit for surface-routed channels
//
// Like ChannelFieldImplicit but works with arbitrary spine paths from SurfaceTurtle,
// not just helical spines. Uses spatial grid for nearest-segment search.
//
// Trade-off: O(1) helix unwrap → O(~1) grid lookup. Slightly slower but works on ANY surface.

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public class RoutedChannelFieldImplicit : IImplicit
{
    readonly AeroSpec _S;
    readonly bool _isShroud;

    // All spine segments across all channels
    readonly Vector3[] _segA;   // segment start points
    readonly Vector3[] _segB;   // segment end points
    readonly float[] _segZ;     // average z of segment (for physics lookup)
    readonly int[] _segSpineIdx;// which spine (channel) each segment belongs to
    readonly int _nSegments;
    readonly int _nSpines;

    // Per-channel multipliers (symmetry-breaking)
    readonly float[] _widthMult;  // [_nSpines]: per-channel random multiplier on halfW/halfH
    readonly float[] _thetaMod;   // [_nSpines]: sinusoidal θ-modulation factor on halfH

    // Spatial grid for fast nearest-segment lookup
    readonly int _gridX, _gridY, _gridZ;
    readonly float _gridMinX, _gridMinY, _gridMinZ;
    readonly float _cellSize;
    readonly List<int>[] _grid;  // each cell → list of segment indices

    // Pre-sampled physics (same as ChannelFieldImplicit)
    readonly float[] _halfW;
    readonly float[] _halfH;
    readonly float[] _superN;
    readonly int _nSamples;
    readonly float _zStart, _zEnd, _zStep;
    readonly float _ribPitch, _ribHeight, _minHalfH;

    // Bounding box
    float _bboxMinX, _bboxMinY, _bboxMinZ;
    float _bboxMaxX, _bboxMaxY, _bboxMaxZ;

    public RoutedChannelFieldImplicit(AeroSpec S, ChannelRouteResult route, bool isShroud)
    {
        _S = S;
        _isShroud = isShroud;
        _ribPitch = S.ribPitch;
        _ribHeight = S.ribHeight;
        _minHalfH = S.minChannel / 2f;

        var spines = route.Spines;
        _nSpines = spines.Count;

        // Collect all segments, tracking which spine (channel) each belongs to
        var segAList = new List<Vector3>();
        var segBList = new List<Vector3>();
        var segZList = new List<float>();
        var segSpineList = new List<int>();

        int spineIdx = 0;
        foreach (var spine in spines)
        {
            for (int i = 0; i < spine.Count - 1; i++)
            {
                segAList.Add(spine[i]);
                segBList.Add(spine[i + 1]);
                segZList.Add((spine[i].Z + spine[i + 1].Z) / 2f);
                segSpineList.Add(spineIdx);
            }
            spineIdx++;
        }

        _segA = segAList.ToArray();
        _segB = segBList.ToArray();
        _segZ = segZList.ToArray();
        _segSpineIdx = segSpineList.ToArray();
        _nSegments = _segA.Length;

        // Pre-compute per-channel multipliers (symmetry-breaking L3):
        //  - widthMult: random ±jitter — gives each channel its own "personality" (individual thickness)
        //  - thetaMod:  sinusoidal cos(harmonic · phi_i) — correlated azimuthal bulge
        //               coherent with fuel inlet at phi=0° and LOX inlet at phi=180° (harmonic=2)
        //
        // Seed for width axis is derived from isShroud so shroud and spike differ.
        int widthSeed = S.channelSeed + (isShroud ? 0 : ChannelRouter.SPIKE_SEED_OFFSET);
        _widthMult = new float[_nSpines];
        _thetaMod  = new float[_nSpines];
        for (int s = 0; s < _nSpines; s++)
        {
            _widthMult[s] = 1f + S.perChannelWidthJitterFraction
                              * ChannelRouter.DetRand(widthSeed, s, ChannelRouter.AXIS_WIDTH);
            float phi = (s < route.Phases.Length) ? route.Phases[s] : 0f;
            _thetaMod[s]  = 1f + S.heatFluxAngularAmplitude
                              * MathF.Cos(S.heatFluxAngularHarmonic * phi);
        }

        // Bounding box from segments
        _bboxMinX = _bboxMinY = _bboxMinZ = float.MaxValue;
        _bboxMaxX = _bboxMaxY = _bboxMaxZ = float.MinValue;
        float margin = 10f; // mm margin for channels + wall
        for (int i = 0; i < _nSegments; i++)
        {
            _bboxMinX = MathF.Min(_bboxMinX, MathF.Min(_segA[i].X, _segB[i].X));
            _bboxMinY = MathF.Min(_bboxMinY, MathF.Min(_segA[i].Y, _segB[i].Y));
            _bboxMinZ = MathF.Min(_bboxMinZ, MathF.Min(_segA[i].Z, _segB[i].Z));
            _bboxMaxX = MathF.Max(_bboxMaxX, MathF.Max(_segA[i].X, _segB[i].X));
            _bboxMaxY = MathF.Max(_bboxMaxY, MathF.Max(_segA[i].Y, _segB[i].Y));
            _bboxMaxZ = MathF.Max(_bboxMaxZ, MathF.Max(_segA[i].Z, _segB[i].Z));
        }
        _bboxMinX -= margin; _bboxMinY -= margin; _bboxMinZ -= margin;
        _bboxMaxX += margin; _bboxMaxY += margin; _bboxMaxZ += margin;

        // Build spatial grid
        _cellSize = 5f; // mm per cell
        _gridMinX = _bboxMinX; _gridMinY = _bboxMinY; _gridMinZ = _bboxMinZ;
        _gridX = (int)MathF.Ceiling((_bboxMaxX - _bboxMinX) / _cellSize) + 1;
        _gridY = (int)MathF.Ceiling((_bboxMaxY - _bboxMinY) / _cellSize) + 1;
        _gridZ = (int)MathF.Ceiling((_bboxMaxZ - _bboxMinZ) / _cellSize) + 1;

        int totalCells = _gridX * _gridY * _gridZ;
        _grid = new List<int>[totalCells];
        for (int c = 0; c < totalCells; c++)
            _grid[c] = new List<int>();

        // Insert segments into grid cells they overlap
        for (int i = 0; i < _nSegments; i++)
        {
            int x0 = CellX(MathF.Min(_segA[i].X, _segB[i].X) - 3f);
            int x1 = CellX(MathF.Max(_segA[i].X, _segB[i].X) + 3f);
            int y0 = CellY(MathF.Min(_segA[i].Y, _segB[i].Y) - 3f);
            int y1 = CellY(MathF.Max(_segA[i].Y, _segB[i].Y) + 3f);
            int z0 = CellZ(MathF.Min(_segA[i].Z, _segB[i].Z) - 3f);
            int z1 = CellZ(MathF.Max(_segA[i].Z, _segB[i].Z) + 3f);

            for (int cx = x0; cx <= x1; cx++)
            for (int cy = y0; cy <= y1; cy++)
            for (int cz = z0; cz <= z1; cz++)
            {
                int idx = GridIdx(cx, cy, cz);
                if (idx >= 0 && idx < totalCells)
                    _grid[idx].Add(i);
            }
        }

        // Pre-sample physics
        _zStart = _bboxMinZ + margin;
        _zEnd = _bboxMaxZ - margin;
        _nSamples = 2000;
        _zStep = (_zEnd - _zStart) / (_nSamples - 1);
        _halfW = new float[_nSamples];
        _halfH = new float[_nSamples];
        _superN = new float[_nSamples];

        float fadeLen = MathF.Max((_zEnd - _zStart) * 0.08f, 5f);
        float manifoldHalf = S.manifoldRadius * 0.8f;

        for (int i = 0; i < _nSamples; i++)
        {
            float z = _zStart + i * _zStep;
            float halfW, halfH;
            if (isShroud)
            {
                var (w, h) = HeatTransfer.ChannelRect(S, z);
                halfW = w / 2f;
                halfH = h / 2f;
            }
            else
            {
                var (w, h) = HeatTransfer.ChannelRectSpike(S, z);
                halfW = w / 2f;
                halfH = h / 2f;
            }

            // Fade transitions: channels merge with collectors at edges
            float fadeIn = Smoothstep(_zStart, _zStart + fadeLen, z);
            float fadeOut = Smoothstep(_zEnd, _zEnd - fadeLen, z);
            float fade = fadeIn * fadeOut;
            halfW = manifoldHalf + (halfW - manifoldHalf) * fade;
            halfH = manifoldHalf + (halfH - manifoldHalf) * fade;

            _halfW[i] = halfW;
            _halfH[i] = halfH;
            _superN[i] = 2.5f;
        }
    }

    int CellX(float x) => Math.Clamp((int)((x - _gridMinX) / _cellSize), 0, _gridX - 1);
    int CellY(float y) => Math.Clamp((int)((y - _gridMinY) / _cellSize), 0, _gridY - 1);
    int CellZ(float z) => Math.Clamp((int)((z - _gridMinZ) / _cellSize), 0, _gridZ - 1);
    int GridIdx(int cx, int cy, int cz) => cx + cy * _gridX + cz * _gridX * _gridY;

    float Lerp(float[] arr, float z)
    {
        float t = (z - _zStart) / _zStep;
        int i = (int)t;
        if (i < 0) return arr[0];
        if (i >= _nSamples - 1) return arr[_nSamples - 1];
        float frac = t - i;
        return arr[i] + frac * (arr[i + 1] - arr[i]);
    }

    public float fSignedDistance(in Vector3 v)
    {
        // Find nearest segment via grid
        int cx = CellX(v.X);
        int cy = CellY(v.Y);
        int cz = CellZ(v.Z);
        int cellIdx = GridIdx(cx, cy, cz);

        if (cellIdx < 0 || cellIdx >= _grid.Length)
            return 10f; // far outside

        float minDist2 = float.MaxValue;
        float nearestZ = v.Z;
        Vector3 nearestClosest = v;
        Vector3 nearestTangent = Vector3.UnitZ;
        int nearestSpineIdx = 0;

        var candidates = _grid[cellIdx];
        for (int c = 0; c < candidates.Count; c++)
        {
            int segIdx = candidates[c];
            Vector3 ab = _segB[segIdx] - _segA[segIdx];
            Vector3 ap = v - _segA[segIdx];
            float t = Math.Clamp(Vector3.Dot(ap, ab) / Vector3.Dot(ab, ab), 0f, 1f);
            Vector3 closest = _segA[segIdx] + ab * t;
            float d2 = Vector3.DistanceSquared(v, closest);
            if (d2 < minDist2)
            {
                minDist2 = d2;
                nearestZ = closest.Z;
                nearestClosest = closest;
                nearestTangent = ab;
                nearestSpineIdx = _segSpineIdx[segIdx];
            }
        }

        if (minDist2 > 100f)
            return MathF.Sqrt(minDist2);

        // Local frame decomposition at nearest spine point:
        // tangent = along spine segment
        // radial = from engine axis (Z) toward spine point (outward)
        // tangential = cross(tangent, radial) — along the surface circumference
        Vector3 tangent = nearestTangent;
        if (tangent.LengthSquared() < 1e-10f) tangent = Vector3.UnitZ;
        tangent = Vector3.Normalize(tangent);

        // Radial direction: from Z axis to spine point (in XY plane)
        Vector3 radialDir = new(nearestClosest.X, nearestClosest.Y, 0f);
        if (radialDir.LengthSquared() < 1e-10f) radialDir = Vector3.UnitX;
        radialDir = Vector3.Normalize(radialDir);

        // Tangential direction: perpendicular to both tangent and radial
        Vector3 tangentialDir = Vector3.Cross(tangent, radialDir);
        if (tangentialDir.LengthSquared() < 1e-10f)
            tangentialDir = Vector3.Cross(tangent, Vector3.UnitX);
        tangentialDir = Vector3.Normalize(tangentialDir);

        // Recompute radial to ensure orthogonal frame
        radialDir = Vector3.Cross(tangentialDir, tangent);
        radialDir = Vector3.Normalize(radialDir);

        // Project offset vector onto local frame
        Vector3 offset = v - nearestClosest;
        float dRadial = Vector3.Dot(offset, radialDir);
        float dTangential = Vector3.Dot(offset, tangentialDir);

        // Channel dimensions at this z.
        // θ-modulation is applied as SPATIAL function of voxel position (not per-channel):
        //   thetaMod = 1 + amp · cos(harmonic · atan2(v.Y, v.X))
        // This gives correlated azimuthal bulge (hot side thicker, cold side thinner) while
        // keeping the distance field CONTINUOUS — no spine jumping discontinuities that would
        // break marching cubes. Per-channel width jitter is intentionally omitted because it
        // is fundamentally discontinuous (step function at spine boundaries breaks watertight).
        float phiVoxel = MathF.Atan2(v.Y, v.X);
        float thetaMod = 1f + _S.heatFluxAngularAmplitude
                            * MathF.Cos(_S.heatFluxAngularHarmonic * phiVoxel);
        float halfW = Lerp(_halfW, nearestZ);                    // tangential half-width
        float halfH = Lerp(_halfH, nearestZ) * thetaMod;         // radial half-height (θ-modulated by position)
        float n = Lerp(_superN, nearestZ);

        // Turbulator ribs
        float arcLen = nearestZ - _zStart;
        float ribPhase = arcLen % _ribPitch;
        if (ribPhase < 0f) ribPhase += _ribPitch;
        if (ribPhase / _ribPitch < 0.25f)
        {
            float edge = Smoothstep(0f, 0.05f, ribPhase / _ribPitch)
                       * Smoothstep(0.25f, 0.20f, ribPhase / _ribPitch);
            halfH = MathF.Max(halfH - _ribHeight / 2f * edge, _minHalfH);
        }

        // SuperellipseSDF with proper local decomposition
        return SuperellipseSDF(dTangential, dRadial, halfW, halfH, n);
    }

    static float SuperellipseSDF(float x, float y, float halfW, float halfH, float n)
    {
        if (halfW < 0.01f || halfH < 0.01f) return 1f;
        float u = MathF.Abs(x) / halfW;
        float v = MathF.Abs(y) / halfH;
        float pu = u > 1e-6f ? MathF.Pow(u, n) : 0f;
        float pv = v > 1e-6f ? MathF.Pow(v, n) : 0f;
        float F = MathF.Pow(pu + pv, 1f / n) - 1f;
        return F * MathF.Min(halfW, halfH);
    }

    static float Smoothstep(float edge0, float edge1, float x)
    {
        float t = Math.Clamp((x - edge0) / (edge1 - edge0), 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    public BBox3 GetBBox()
    {
        return new BBox3(
            new Vector3(_bboxMinX, _bboxMinY, _bboxMinZ),
            new Vector3(_bboxMaxX, _bboxMaxY, _bboxMaxZ));
    }
}
