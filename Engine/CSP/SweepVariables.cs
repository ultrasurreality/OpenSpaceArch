// SweepVariables.cs — Phase 2 bounded random sweep search space.
//
// Defines which AeroSpec parameters the outer layer varies and their
// allowed ranges. Sample() draws a single AeroSpec with all 6 bounded
// variables randomized from a seeded Random and the two fixed inputs
// (thrust = mission spec, voxelSize = manufacturing constraint).
//
// See ARCHITECTURE.md — these
// 6 variables are the Phase 2 subset of the full bounded-variable set
// formalized in ARCHITECTURE.md.

namespace OpenSpaceArch.Engine.CSP;

/// <summary>
/// Bounded search space for the Phase 2 outer sweep. Each field is a
/// <see cref="BoundedFloat"/> whose [Min, Max] defines the sampling range.
/// Default ranges cover the feasible zone for a 5 kN LOX/CH4 aerospike
/// within CuCrZr material limits and LPBF manufacturing constraints.
/// </summary>
public sealed class SweepVariables
{
    /// <summary>Chamber pressure (Pa). 50–150 bar — below = poor Isp, above = Barlow violation.</summary>
    public BoundedFloat Pc = new(Min: 50e5f, Max: 150e5f, Current: 110e5f);

    /// <summary>Oxidizer-to-fuel mass ratio. 2.5–4.0 covers stoich peak Isp (~3.7 for LOX/CH4).</summary>
    public BoundedFloat OF = new(Min: 2.5f, Max: 4.0f, Current: 3.2f);

    /// <summary>Contraction ratio Ac/At. 3–6 gives stable combustion without excessive mass.</summary>
    public BoundedFloat CR = new(Min: 3.0f, Max: 6.0f, Current: 4.0f);

    /// <summary>Characteristic length L* (m). 0.3–1.2 covers short-to-long chambers for LOX/CH4.</summary>
    public BoundedFloat Lstar = new(Min: 0.3f, Max: 1.2f, Current: 0.4f);

    /// <summary>Safety factor on yield strength for wall sizing. 1.3–2.0 standard LPBF practice.</summary>
    public BoundedFloat SF = new(Min: 1.3f, Max: 2.0f, Current: 1.5f);

    /// <summary>Helical channel twist turns over channel length. 1–4 covers mild to aggressive swirl.</summary>
    public BoundedFloat TwistTurns = new(Min: 1.0f, Max: 4.0f, Current: 2.0f);

    /// <summary>
    /// Draws one <see cref="AeroSpec"/> with all 6 bounded variables sampled
    /// from <paramref name="rng"/> and thrust/voxel pinned to mission inputs.
    /// The returned spec has only INPUT fields populated — callers run
    /// <c>DesignSweep.ComputeSilentPublic</c> to fill COMPUTED fields.
    /// </summary>
    public AeroSpec Sample(Random rng, float fixedThrust, float fixedVoxel)
    {
        return new AeroSpec
        {
            F_thrust = fixedThrust,
            voxelSize = fixedVoxel,
            Pc = Pc.Sample(rng),
            OF_ratio = OF.Sample(rng),
            CR = CR.Sample(rng),
            Lstar = Lstar.Sample(rng),
            SF = SF.Sample(rng),
            channelTwistTurns = TwistTurns.Sample(rng),
            // Match DesignSweep: use real LPBF min rib wall, not 3×voxel
            minRibWall = 0.5f,
        };
    }

    /// <summary>Human-readable summary of the 6 ranges, one line.</summary>
    public string Summary() =>
        $"Pc[{Pc.Min / 1e5f:F0}-{Pc.Max / 1e5f:F0}bar] " +
        $"OF[{OF.Min:F1}-{OF.Max:F1}] " +
        $"CR[{CR.Min:F1}-{CR.Max:F1}] " +
        $"L*[{Lstar.Min:F2}-{Lstar.Max:F2}m] " +
        $"SF[{SF.Min:F1}-{SF.Max:F1}] " +
        $"Twist[{TwistTurns.Min:F1}-{TwistTurns.Max:F1}]";
}
