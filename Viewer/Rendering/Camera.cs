// Camera.cs — orbit camera for cinematic viewer.
// Spherical coordinates: pitch/yaw/distance around a target.
// Controls: left mouse drag = orbit, right mouse drag = pan, scroll = zoom.

using System.Numerics;

namespace OpenSpaceArch.Viewer.Rendering;

public sealed class Camera
{
    public Vector3 Target = Vector3.Zero;
    public float Pitch = MathF.PI * 0.25f;   // radians
    public float Yaw = MathF.PI * 0.5f;
    public float Distance = 250f;

    public float FovYRad = MathF.PI / 3f;  // 60°
    public float NearZ = 1f;
    public float FarZ = 10000f;

    public Vector3 Position
    {
        get
        {
            float cosP = MathF.Cos(Pitch);
            return Target + Distance * new Vector3(
                cosP * MathF.Cos(Yaw),
                cosP * MathF.Sin(Yaw),
                MathF.Sin(Pitch));
        }
    }

    public Matrix4x4 ViewMatrix => Matrix4x4.CreateLookAt(Position, Target, Vector3.UnitZ);

    public Matrix4x4 ProjectionMatrix(float aspectRatio) =>
        Matrix4x4.CreatePerspectiveFieldOfView(FovYRad, aspectRatio, NearZ, FarZ);

    public void Orbit(float dYaw, float dPitch)
    {
        Yaw += dYaw;
        Pitch = Math.Clamp(Pitch + dPitch, -MathF.PI / 2f + 0.01f, MathF.PI / 2f - 0.01f);
    }

    public void Zoom(float scrollDelta)
    {
        Distance = Math.Clamp(Distance * MathF.Exp(-scrollDelta * 0.15f), 20f, 3000f);
    }

    public void Pan(Vector2 screenDelta)
    {
        Vector3 forward = Vector3.Normalize(Target - Position);
        Vector3 right = Vector3.Normalize(Vector3.Cross(forward, Vector3.UnitZ));
        Vector3 up = Vector3.Normalize(Vector3.Cross(right, forward));
        float scale = Distance * 0.001f;
        Target += (-right * screenDelta.X + up * screenDelta.Y) * scale;
    }

    public void Frame(BoundingSphere sphere)
    {
        Target = sphere.Center;
        Distance = sphere.Radius * 2.5f;
    }
}

public readonly record struct BoundingSphere(Vector3 Center, float Radius);
