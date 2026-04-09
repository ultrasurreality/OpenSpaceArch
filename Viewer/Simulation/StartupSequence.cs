// StartupSequence.cs — time-keyed startup animation driving throttle, glow intensity,
// plume spawn rate, wall heat intensity.
//
// Phases:
//   0.0 - 0.3s  Ignition flash — plume spikes briefly, chamber flash
//   0.3 - 1.8s  Ramp up — throttle 0 → 1
//   1.8+        Steady state — throttle = 1

namespace OpenSpaceArch.Viewer.Simulation;

public sealed class StartupSequence
{
    public bool Active { get; private set; }
    public float Time { get; private set; }
    public float Throttle { get; private set; }
    public float IgnitionFlash { get; private set; }

    public const float IgnitionDuration = 0.3f;
    public const float RampDuration = 1.5f;

    public void Ignite()
    {
        Active = true;
        Time = 0f;
        Throttle = 0f;
        IgnitionFlash = 0f;
    }

    public void Reset()
    {
        Active = false;
        Time = 0f;
        Throttle = 0f;
        IgnitionFlash = 0f;
    }

    public void Update(float dt)
    {
        if (!Active) return;
        Time += dt;

        if (Time < IgnitionDuration)
        {
            // Quick flash — normalized bell around 0.15
            float t = Time / IgnitionDuration;
            IgnitionFlash = MathF.Sin(t * MathF.PI);
            Throttle = MathF.Pow(t, 0.5f) * 0.4f;
        }
        else if (Time < IgnitionDuration + RampDuration)
        {
            float t = (Time - IgnitionDuration) / RampDuration;
            IgnitionFlash = MathF.Max(0f, 1f - t * 3f);
            Throttle = 0.4f + 0.6f * SmoothStep(t);
        }
        else
        {
            IgnitionFlash = 0f;
            Throttle = 1f;
        }
    }

    private static float SmoothStep(float t)
    {
        t = Math.Clamp(t, 0f, 1f);
        return t * t * (3f - 2f * t);
    }
}
