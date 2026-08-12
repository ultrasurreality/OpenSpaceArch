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
    /// <param name="centerBiased">
    /// When <c>false</c> (default) every variable is sampled UNIFORMLY across
    /// its full [Min, Max] range — the standard bounded random sweep that
    /// explores the whole box evenly. When <c>true</c> each variable is sampled
    /// from a clamped Gaussian centered at the midpoint of its range
    /// (<see cref="BoundedFloat.SampleGaussian"/>), which concentrates samples
    /// near the "nominal" design and is useful for a refinement pass once a
    /// uniform sweep has located a promising region.
    /// </param>
    public AeroSpec Sample(Random rng, float fixedThrust, float fixedVoxel, bool centerBiased = false)
    {
        float pc    = centerBiased ? Pc.SampleGaussian(rng)         : Pc.Sample(rng);
        float of    = centerBiased ? OF.SampleGaussian(rng)         : OF.Sample(rng);
        float cr    = centerBiased ? CR.SampleGaussian(rng)         : CR.Sample(rng);
        float lstar = centerBiased ? Lstar.SampleGaussian(rng)      : Lstar.Sample(rng);
        float sf    = centerBiased ? SF.SampleGaussian(rng)         : SF.Sample(rng);
        float twist = centerBiased ? TwistTurns.SampleGaussian(rng) : TwistTurns.Sample(rng);

        return BuildSpec(fixedThrust, fixedVoxel, pc, of, cr, lstar, sf, twist);
    }

    /// <summary>
    /// Draws one <see cref="AeroSpec"/> with all 6 bounded variables sampled from
    /// clamped Gaussians CENTERED on a known-good design <paramref name="center"/>
    /// (the best candidate found by a prior uniform exploration batch) rather than
    /// on the range midpoint. This is the refinement half of the adaptive sweep:
    /// it concentrates samples around the best-known point so the second batch
    /// hill-climbs toward the local optimum while still respecting every
    /// [Min, Max] bound (each value is clamped inside its range).
    ///
    /// The per-variable spread is <paramref name="stdFrac"/> of each range's span;
    /// a smaller value tightens the cloud around <paramref name="center"/>. Every
    /// variable reuses <see cref="BoundedFloat.SampleGaussian"/>, so determinism is
    /// preserved when <paramref name="rng"/> is seeded.
    /// </summary>
    /// <param name="center">
    /// The design to refine around. Only the 6 bounded INPUT fields are read
    /// (Pc, OF_ratio, CR, Lstar, SF, channelTwistTurns); COMPUTED fields are
    /// ignored. Each is mapped to a [0,1] fraction of its variable's range and
    /// used as the Gaussian center, so the cloud follows the best point even if
    /// it sits near a range edge (clamping then truncates the tail at the bound).
    /// </param>
    /// <param name="stdFrac">Std-dev as a fraction of each range's span. Default 0.12 ≈ a tight refinement cloud.</param>
    public AeroSpec SampleAround(Random rng, float fixedThrust, float fixedVoxel, AeroSpec center, float stdFrac = 0.12f)
    {
        float pc    = Pc.SampleGaussian(rng,         Frac(Pc,         center.Pc),                stdFrac);
        float of    = OF.SampleGaussian(rng,         Frac(OF,         center.OF_ratio),          stdFrac);
        float cr    = CR.SampleGaussian(rng,         Frac(CR,         center.CR),                stdFrac);
        float lstar = Lstar.SampleGaussian(rng,      Frac(Lstar,      center.Lstar),             stdFrac);
        float sf    = SF.SampleGaussian(rng,         Frac(SF,         center.SF),                stdFrac);
        float twist = TwistTurns.SampleGaussian(rng, Frac(TwistTurns, center.channelTwistTurns), stdFrac);

        return BuildSpec(fixedThrust, fixedVoxel, pc, of, cr, lstar, sf, twist);
    }

    /// <summary>[0,1] position of <paramref name="value"/> within <paramref name="b"/>'s range, clamped.</summary>
    private static float Frac(BoundedFloat b, float value) =>
        (b.Max > b.Min) ? Math.Clamp((value - b.Min) / (b.Max - b.Min), 0f, 1f) : 0.5f;

    /// <summary>Assembles an INPUT-only <see cref="AeroSpec"/> from the 6 sampled bounded values plus the 2 fixed inputs.</summary>
    private static AeroSpec BuildSpec(
        float fixedThrust, float fixedVoxel,
        float pc, float of, float cr, float lstar, float sf, float twist) =>
        new AeroSpec
        {
            F_thrust = fixedThrust,
            voxelSize = fixedVoxel,
            Pc = pc,
            OF_ratio = of,
            CR = cr,
            Lstar = lstar,
            SF = sf,
            channelTwistTurns = twist,
            // Match DesignSweep: use real LPBF min rib wall, not 3×voxel
            minRibWall = 0.5f,
        };

    /// <summary>Human-readable summary of the 6 ranges, one line.</summary>
    public string Summary() =>
        $"Pc[{Pc.Min / 1e5f:F0}-{Pc.Max / 1e5f:F0}bar] " +
        $"OF[{OF.Min:F1}-{OF.Max:F1}] " +
        $"CR[{CR.Min:F1}-{CR.Max:F1}] " +
        $"L*[{Lstar.Min:F2}-{Lstar.Max:F2}m] " +
        $"SF[{SF.Min:F1}-{SF.Max:F1}] " +
        $"Twist[{TwistTurns.Min:F1}-{TwistTurns.Max:F1}]";
}
