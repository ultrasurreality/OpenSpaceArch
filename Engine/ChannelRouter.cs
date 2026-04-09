// ChannelRouter.cs — Route N cooling channels on engine surface via SurfaceTurtle
//
// Creates an offset surface (where channel centers live), spawns N turtles
// with INDIVIDUAL start angles and per-channel twistRates, each walks helically
// up the surface. Returns spine paths plus metadata needed for per-channel
// physics downstream (phi_i, twistRate_i).
//
// Symmetry-breaking: golden-ratio phyllotaxis + jitter for phi_i, per-channel
// twist jitter — driven by S.channelSeed (reproducible, cross-platform) and
// AeroSpec jitter fractions. If all jitters = 0 and phaseDistributionExponent = 0,
// behavior reduces to the old uniform 2π/N distribution.

using System.Numerics;
using PicoGK;

namespace OpenSpaceArch.Engine;

/// Result of routing N channels: paths + per-channel metadata for downstream physics.
public record ChannelRouteResult(
    List<List<Vector3>> Spines,
    float[] Phases,      // phi_i (radians) — used for θ-modulation of halfH
    float[] TwistRates); // twistRate_i (rad/mm) — per-channel

public static class ChannelRouter
{
    // Golden angle = π·(3 − √5) ≈ 2.39996 rad — fibonacci phyllotaxis constant.
    // Guarantees low-discrepancy sequence: any prefix of phi_i is quasi-uniform on [0, 2π).
    const float GOLDEN_ANGLE = 2.3999632297f;

    // Axis names for DetRand — stable strings.
    // CRITICAL: NEVER rename these — changes break reproducibility of existing STL.
    internal const string AXIS_PHI   = "phi";
    internal const string AXIS_TWIST = "twist";
    internal const string AXIS_WIDTH = "width";

    // Seed offset for spike system so shroud and spike get different random patterns
    // even with the same user-facing S.channelSeed.
    // Internal so RoutedChannelFieldImplicit can derive matching widthSeed for spike.
    internal const int SPIKE_SEED_OFFSET = 1000;

    /// Route shroud cooling channels on the outer surface of the chamber.
    /// Uses ANALYTICAL helical path on the revolution body — no turtle walk, no voxel
    /// discretization artifacts. Each channel gets individual phi_i and twistRate_i.
    public static ChannelRouteResult RouteShroudChannels(AeroSpec S, Voxels voxChamber)
    {
        Library.Log("  Routing shroud channels (analytical)...");

        float avgWall = HeatTransfer.WallThickness(S, S.zThroat);
        float avgHalfH = 1.5f;

        int nChannels = S.nChannelsShroud;
        float twistTotal = S.channelTwistTurns * 2f * MathF.PI;
        float zStart = S.zCowl + 3f;
        float zEnd = S.zInjector - 3f;
        float globalTwistRate = twistTotal / (zEnd - zStart);
        float stepSize = 3.0f;

        var allPaths = new List<List<Vector3>>();
        var phases = new List<float>();
        var twistRates = new List<float>();

        for (int i = 0; i < nChannels; i++)
        {
            float phiBase = GeneratePhiBase(
                i, nChannels, S.channelSeed,
                S.azimuthalJitterFraction, S.phaseDistributionExponent);
            float twistRate_i = GenerateTwistRate(
                i, globalTwistRate, S.channelSeed, S.twistJitterFraction);

            var path = BuildAnalyticalHelix(
                phiBase, twistRate_i, zStart, zEnd, stepSize,
                z => ChamberSizing.ShroudProfile(S, z) + avgWall + avgHalfH);

            if (path.Count >= 3)
            {
                allPaths.Add(path);
                phases.Add(phiBase);
                twistRates.Add(twistRate_i);
            }
        }

        Library.Log($"    Routed {allPaths.Count}/{nChannels} channels, avg {allPaths.Average(p => p.Count):F0} points each");
        LogChannels("shroud", phases, twistRates);
        return new ChannelRouteResult(allPaths, phases.ToArray(), twistRates.ToArray());
    }

    /// Route spike cooling channels on the inner surface of the spike (analytical).
    public static ChannelRouteResult RouteSpikeChannels(AeroSpec S, Voxels voxChamber)
    {
        Library.Log("  Routing spike channels (analytical)...");

        float avgWall = HeatTransfer.WallThickness(S, S.zThroat);
        float avgHalfH = 1.0f;

        int nChannels = S.nChannelsSpike;
        float twistTotal = 1.0f * 2f * MathF.PI; // 1 turn for spike
        float zStart = S.zCowl + 3f;
        float zEnd = S.zInjector - 3f;
        float globalTwistRate = twistTotal / (zEnd - zStart);
        float stepSize = 1.5f;

        var allPaths = new List<List<Vector3>>();
        var phases = new List<float>();
        var twistRates = new List<float>();

        int spikeSeed = S.channelSeed + SPIKE_SEED_OFFSET;

        for (int i = 0; i < nChannels; i++)
        {
            float phiBase = GeneratePhiBase(
                i, nChannels, spikeSeed,
                S.azimuthalJitterFraction, S.phaseDistributionExponent);
            float twistRate_i = GenerateTwistRate(
                i, globalTwistRate, spikeSeed, S.twistJitterFraction);

            // Spike channels live INSIDE the spike (negative offset).
            // Skip if radius would be too small (tip region).
            float rCheck = ChamberSizing.SpikeProfile(S, zStart) - avgWall - avgHalfH;
            if (rCheck < 2f) continue;

            var path = BuildAnalyticalHelix(
                phiBase, twistRate_i, zStart, zEnd, stepSize,
                z =>
                {
                    float rInner = ChamberSizing.SpikeProfile(S, z) - avgWall - avgHalfH;
                    return MathF.Max(rInner, 0.5f);
                });

            if (path.Count >= 3)
            {
                allPaths.Add(path);
                phases.Add(phiBase);
                twistRates.Add(twistRate_i);
            }
        }

        Library.Log($"    Routed {allPaths.Count}/{nChannels} spike channels");
        LogChannels("spike", phases, twistRates);
        return new ChannelRouteResult(allPaths, phases.ToArray(), twistRates.ToArray());
    }

    /// Build an analytical helical path on a revolution surface defined by r = radiusFunc(z).
    /// No turtle walk, no voxel artifacts — exact helix samples the profile at regular z steps.
    static List<Vector3> BuildAnalyticalHelix(
        float phiBase, float twistRate, float zStart, float zEnd, float stepSize,
        Func<float, float> radiusFunc)
    {
        var path = new List<Vector3>();
        int nSteps = (int)MathF.Ceiling((zEnd - zStart) / stepSize);
        for (int k = 0; k <= nSteps; k++)
        {
            float z = zStart + k * stepSize;
            if (z > zEnd) z = zEnd;
            float r = radiusFunc(z);
            float angle = phiBase + twistRate * (z - zStart);
            path.Add(new Vector3(r * MathF.Cos(angle), r * MathF.Sin(angle), z));
        }
        return path;
    }

    // ─────────────────────────────────────────────────────────────
    // Symmetry-breaking helpers
    // ─────────────────────────────────────────────────────────────

    /// Generate individual phi for channel `i`: golden-ratio phyllotaxis + jitter.
    /// Produces low-discrepancy aperiodic distribution — visually biological at N ≥ 10.
    ///   uniform_i = i · 2π/N                      (old behavior)
    ///   golden_i  = (i · φ_angle + seed·0.001) mod 2π
    ///   phi_i     = lerp(uniform_i, golden_i, distExp)
    ///             + DetRand(seed,i,"phi") · (2π/N) · jitterFrac
    static float GeneratePhiBase(int i, int N, int seed, float jitterFrac, float distExp)
    {
        float twoPi = 2f * MathF.PI;
        float uniform_i = i * twoPi / N;
        float golden_i  = (i * GOLDEN_ANGLE + seed * 0.001f) % twoPi;
        if (golden_i < 0f) golden_i += twoPi;
        // Lerp between uniform and golden via distExp (1.0 = full golden, 0.0 = uniform)
        float phi = uniform_i + (golden_i - uniform_i) * distExp;
        // Individual jitter within [−1, +1] · slot width · jitter fraction
        phi += DetRand(seed, i, AXIS_PHI) * (twoPi / N) * jitterFrac;
        // Wrap to [0, 2π)
        phi = phi % twoPi;
        if (phi < 0f) phi += twoPi;
        return phi;
    }

    /// Generate individual twist rate for channel `i`: global · (1 ± jitter).
    static float GenerateTwistRate(int i, float globalTwistRate, int seed, float jitterFrac)
    {
        return globalTwistRate * (1f + DetRand(seed, i, AXIS_TWIST) * jitterFrac);
    }

    /// Deterministic hash-based pseudo-random: returns value in [−1, +1].
    /// Uses integer hashing (Wang hash style) — reproducible across platforms,
    /// no dependency on System.Random state. Same (seed, idx, axis) → same output.
    /// This is the single source of randomness for symmetry-breaking; downstream
    /// code (RoutedChannelFieldImplicit) also calls this with AXIS_WIDTH.
    internal static float DetRand(int seed, int idx, string axis)
    {
        // Mix seed and idx via Wang-style integer hash
        uint h = (uint)seed;
        h ^= 0x9E3779B9u + ((uint)idx << 6) + ((uint)idx >> 2);
        // Stable 32-bit string hash (FNV-1a) — independent of .NET's randomized string.GetHashCode
        uint axisHash = 2166136261u;
        for (int k = 0; k < axis.Length; k++)
        {
            axisHash ^= axis[k];
            axisHash *= 16777619u;
        }
        h ^= axisHash + 0x9E3779B9u + (h << 6) + (h >> 2);
        // Wang hash final mix
        h = (h ^ 61u) ^ (h >> 16);
        h *= 9u;
        h ^= h >> 4;
        h *= 0x27D4EB2Du;
        h ^= h >> 15;
        // Normalize uint32 to [−1, +1]
        return (h / 4294967295f) * 2f - 1f;
    }

    static void LogChannels(string label, List<float> phases, List<float> twistRates)
    {
        int N = phases.Count;
        if (N == 0) return;
        Library.Log($"    per-channel metadata ({label}, {N} channels):");
        for (int i = 0; i < N; i++)
        {
            float phiDeg = phases[i] * 180f / MathF.PI;
            float twistDegPerMm = twistRates[i] * 180f / MathF.PI;
            Library.Log($"      [ch {i,2}] phi={phiDeg,6:F1}°  twist={twistDegPerMm,6:F3}°/mm");
        }
    }
}
