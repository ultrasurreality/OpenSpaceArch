// SurfaceTurtle.cs — Turtle walk on SDF surface
//
// Josefine's routing algorithm: turtle graphics on voxelized surface.
// Step → snap to surface → update heading. Simple, elegant, works on ANY surface.
//
// Source: LEAP71 GitHub issue, Ex_SurfaceTurtleWalkShowCase.txt
// PicoGK API: Voxels.bClosestPointOnSurface(), Voxels.vecSurfaceNormal()

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

public class SurfaceTurtle
{
    Vector3 _pos;        // current position ON surface
    Vector3 _heading;    // movement direction (tangent to surface)
    readonly Voxels _surface;  // SDF surface to walk on
    readonly List<Vector3> _path = new();
    readonly float _stepSize;

    public SurfaceTurtle(Voxels surface, Vector3 startPos, Vector3 initialHeading, float stepSize = 0.5f)
    {
        _surface = surface;
        _stepSize = stepSize;

        // Snap start position to surface
        if (_surface.bClosestPointOnSurface(startPos, out Vector3 snapped))
            _pos = snapped;
        else
            _pos = startPos;

        // Make heading tangent to surface
        Vector3 normal = _surface.vecSurfaceNormal(_pos);
        _heading = Vector3.Normalize(initialHeading - Vector3.Dot(initialHeading, normal) * normal);

        _path.Add(_pos);
    }

    /// Take one step: move in heading direction, snap to surface
    public bool Step()
    {
        return Step(_stepSize);
    }

    public bool Step(float length)
    {
        // 1. Propose new position
        Vector3 newPos = _pos + _heading * length;

        // 2. Snap to surface
        if (!_surface.bClosestPointOnSurface(newPos, out Vector3 snapped))
            return false; // surface ended

        // 3. Update heading: direction of actual movement, re-tangentialized
        Vector3 moveDir = snapped - _pos;
        if (moveDir.Length() < 1e-6f)
            return false; // stuck

        Vector3 normal = _surface.vecSurfaceNormal(snapped);
        Vector3 tangent = moveDir - Vector3.Dot(moveDir, normal) * normal;
        if (tangent.Length() < 1e-6f)
            return false;

        _heading = Vector3.Normalize(tangent);
        _pos = snapped;
        _path.Add(_pos);
        return true;
    }

    /// Turn heading by angle (degrees) around surface normal
    public void Turn(float angleDeg)
    {
        Vector3 normal = _surface.vecSurfaceNormal(_pos);
        float rad = angleDeg * MathF.PI / 180f;
        // Rodrigues rotation
        _heading = _heading * MathF.Cos(rad)
                 + Vector3.Cross(normal, _heading) * MathF.Sin(rad)
                 + normal * Vector3.Dot(normal, _heading) * (1f - MathF.Cos(rad));
        _heading = Vector3.Normalize(_heading);
    }

    /// Walk in a plane until z exceeds maxZ (for helical-like paths)
    /// planeNormal defines the plane — turtle walks along surface ∩ plane
    public void NavigateInPlane(Vector3 planeNormal, float maxZ, int maxSteps = 10000)
    {
        planeNormal = Vector3.Normalize(planeNormal);

        for (int i = 0; i < maxSteps; i++)
        {
            if (_pos.Z > maxZ) break;

            // Set heading to intersection of surface tangent plane and guide plane
            Vector3 surfNormal = _surface.vecSurfaceNormal(_pos);
            Vector3 desired = Vector3.Cross(surfNormal, planeNormal);
            if (desired.Length() < 1e-6f) break;
            desired = Vector3.Normalize(desired);

            // Ensure we go upward (positive Z component)
            if (desired.Z < 0) desired = -desired;

            _heading = desired;

            // Drift correction: if we've drifted away from plane, steer back
            float drift = Vector3.Dot(_pos, planeNormal);
            if (MathF.Abs(drift) > _stepSize * 2f)
            {
                Vector3 correction = -planeNormal * drift * 0.3f;
                correction -= Vector3.Dot(correction, surfNormal) * surfNormal; // keep tangent
                if (correction.Length() > 1e-6f)
                    _heading = Vector3.Normalize(_heading + correction * 0.5f);
            }

            if (!Step()) break;
        }
    }

    /// Walk with twist: like NavigateInPlane but the plane rotates with z
    /// Creates helical-like paths that follow the actual surface geometry
    public void NavigateHelical(float startAngle, float twistRate, float maxZ, int maxSteps = 10000)
    {
        for (int i = 0; i < maxSteps; i++)
        {
            if (_pos.Z > maxZ) break;

            // Plane normal rotates with z: creates helical guide
            float angle = startAngle + twistRate * _pos.Z;
            Vector3 planeNormal = new(MathF.Cos(angle), MathF.Sin(angle), 0f);

            Vector3 surfNormal = _surface.vecSurfaceNormal(_pos);
            Vector3 desired = Vector3.Cross(surfNormal, planeNormal);
            if (desired.Length() < 1e-6f) break;
            desired = Vector3.Normalize(desired);

            if (desired.Z < 0) desired = -desired;
            _heading = desired;

            // Drift correction
            float r = MathF.Sqrt(_pos.X * _pos.X + _pos.Y * _pos.Y);
            float currentAngle = MathF.Atan2(_pos.Y, _pos.X);
            float targetAngle = angle + MathF.PI / 2f; // perpendicular to plane normal
            float angleDiff = targetAngle - currentAngle;
            // Wrap to [-π, π]
            while (angleDiff > MathF.PI) angleDiff -= 2f * MathF.PI;
            while (angleDiff < -MathF.PI) angleDiff += 2f * MathF.PI;

            if (MathF.Abs(angleDiff) > 0.05f)
            {
                Vector3 tangential = new(-MathF.Sin(currentAngle), MathF.Cos(currentAngle), 0f);
                Vector3 correction = tangential * angleDiff * 0.2f;
                correction -= Vector3.Dot(correction, surfNormal) * surfNormal;
                if (correction.Length() > 1e-6f)
                    _heading = Vector3.Normalize(_heading + correction);
            }

            if (!Step()) break;
        }
    }

    public Vector3 Position => _pos;
    public Vector3 Heading => _heading;
    public List<Vector3> Path => _path;

    /// Post-process: smooth and re-snap (Josefine does 3 iterations)
    public void SmoothPath(int iterations = 3)
    {
        for (int iter = 0; iter < iterations; iter++)
        {
            if (_path.Count < 3) return;

            // Laplacian smooth
            var smoothed = new List<Vector3>(_path.Count);
            smoothed.Add(_path[0]); // keep endpoints
            for (int i = 1; i < _path.Count - 1; i++)
            {
                Vector3 avg = (_path[i - 1] + _path[i] + _path[i + 1]) / 3f;
                smoothed.Add(avg);
            }
            smoothed.Add(_path[^1]);

            // Re-snap to surface
            for (int i = 1; i < smoothed.Count - 1; i++)
            {
                if (_surface.bClosestPointOnSurface(smoothed[i], out Vector3 snapped))
                    smoothed[i] = snapped;
            }

            _path.Clear();
            _path.AddRange(smoothed);
        }
    }

    /// Decimate path: remove points that are nearly collinear
    public void DecimatePath(float angleThresholdDeg = 5f)
    {
        if (_path.Count < 3) return;

        float cosThreshold = MathF.Cos(angleThresholdDeg * MathF.PI / 180f);
        var decimated = new List<Vector3> { _path[0] };

        for (int i = 1; i < _path.Count - 1; i++)
        {
            Vector3 prev = Vector3.Normalize(_path[i] - decimated[^1]);
            Vector3 next = Vector3.Normalize(_path[i + 1] - _path[i]);
            if (Vector3.Dot(prev, next) < cosThreshold)
                decimated.Add(_path[i]); // keep points at turns
        }
        decimated.Add(_path[^1]);

        _path.Clear();
        _path.AddRange(decimated);
    }
}
