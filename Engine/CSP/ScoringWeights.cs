// ScoringWeights.cs — Phase 2 adjustable multi-objective scoring.
//
// Extracts the 5 positive objective weights and 4 penalty constants
// that DesignSweep.ComputeScore had hardcoded (lines 42-95). The
// Phase 2 SweepPanel exposes these as ImGui sliders so the user can
// re-score existing sweep results in real time without re-sampling.
//
// See ARCHITECTURE.md, section
// "scoring is multi-objective; the human sets the weights".

namespace OpenSpaceArch.Engine.CSP;

/// <summary>
/// Adjustable weights and penalties for the composite engine score.
/// Positive objectives sum nominally to 1.0 in the default preset but
/// the user can push them any direction — the final score is just
/// <c>Σ(w_i · metric_i) − Σ(penalty_i · violation_i)</c>, clamped to [0, 1].
/// </summary>
public sealed class ScoringWeights
{
    // ── Positive objectives (default sums to 1.0) ──

    /// <summary>Isp_SL normalized: 250s→0, 350s→1. Default 0.30.</summary>
    public float wIsp = 0.30f;

    /// <summary>T/W normalized: 50→0, 500→1. Default 0.20.</summary>
    public float wTW = 0.20f;

    /// <summary>Thermal margin = 1 − σ_thermal/σ_yield. Default 0.25.</summary>
    public float wThermal = 0.25f;

    /// <summary>Channel fit = min(1, circumference / needed). Default 0.15.</summary>
    public float wChannelFit = 0.15f;

    /// <summary>Compactness: 1 − (zTotal − 80)/200, 80mm→1, 280mm→0. Default 0.10.</summary>
    public float wCompactness = 0.10f;

    // ── Hard penalties (subtracted from score) ──

    /// <summary>Per 100 MPa of thermal stress above yield. Default 0.10.</summary>
    public float penaltyOverstress = 0.10f;

    /// <summary>Per spatial conflict from SpatialValidator. Default 0.02.</summary>
    public float penaltyConflict = 0.02f;

    /// <summary>Flat penalty if throat gap &lt; 1.5 mm. Default 0.30.</summary>
    public float penaltyThroatGap = 0.30f;

    /// <summary>Flat penalty if coolant velocity &gt; 50 m/s. Default 0.10.</summary>
    public float penaltyCoolant = 0.10f;

    /// <summary>Sum of positive weights — the maximum achievable score before penalties.</summary>
    public float PositiveSum => wIsp + wTW + wThermal + wChannelFit + wCompactness;

    /// <summary>Factory returning a new instance with the default preset above.</summary>
    public static ScoringWeights Default() => new();
}

/// <summary>
/// Single source of truth for engine evaluation: thermal-stress, the validity
/// predicates, the rough mass / thrust-to-weight estimate, and the composite
/// score. Previously this logic was copy-pasted across
/// <c>DesignSweep.Evaluate</c>, <c>DesignSweep.ComputeScore</c>,
/// <c>OuterSweep.EvaluateOne</c> and <c>SweepResult.ComputeScore</c>; all four
/// now delegate here so the physics never drifts apart between the grid sweep
/// and the bounded random sweep.
///
/// Pure math, no voxels — safe to call inside tight sweep loops.
/// </summary>
public static class EngineEvaluation
{
    /// <summary>
    /// Throat thermal stress (MPa) from the through-wall ΔT. Same formula the
    /// grid sweep and outer sweep both used inline. Warning-only — LEAP 71
    /// solves this via laser modulation, so it never blocks validity, but it
    /// does feed the scoring thermal-margin term and over-stress penalty.
    /// </summary>
    public static float ThermalStressMPa(AeroSpec S)
    {
        float deltaT   = S.qThroat * (S.wallThroat / 1000f) / S.k_wall;
        float sigma_th = S.E_mod * S.alpha_CTE * deltaT / (1f - S.nu_poisson);
        return sigma_th / 1e6f;
    }

    /// <summary>
    /// Runs the manufacturing / geometry validity predicates against a spec
    /// whose COMPUTED fields are already filled (i.e. after the silent physics
    /// pipeline). Returns the list of human-readable error strings (empty =
    /// valid) and reports the raw spatial-conflict count via
    /// <paramref name="spatialConflicts"/> for scoring penalties.
    ///
    /// Thermal stress is deliberately NOT a predicate here (warning-only).
    /// </summary>
    public static List<string> RunValidityChecks(AeroSpec S, out int spatialConflicts)
    {
        var errors = new List<string>();

        // 1. Channel diameter below powder-removal minimum
        if (S.chRadiusMin * 2 < S.minChannel)
            errors.Add($"ch_dia={S.chRadiusMin * 2:F2}<{S.minChannel}mm");

        // 2. Self-iteration diverged (coolant velocity too high)
        if (S.v_cool_max > 50f)
            errors.Add($"v_cool={S.v_cool_max:F0}>50m/s");

        // 3. Wall thinner than printable minimum
        if (S.wallThroat < S.minPrintWall)
            errors.Add($"wall={S.wallThroat:F2}<{S.minPrintWall}mm");

        // 4. Throat gap too small to print
        float throatGap = S.rShroudThroat - S.rSpikeThroat;
        if (throatGap < 1.5f)
            errors.Add($"gap={throatGap:F1}<1.5mm");

        // 5. Unrealistic heat flux
        if (S.qThroat / 1e6f > 100f)
            errors.Add($"q={S.qThroat / 1e6:F0}MW/m²");

        // 6. Spatial validation (channels vs shroud vs spike geometry)
        var conflicts = SpatialValidator.Validate(S);
        spatialConflicts = conflicts.Count;
        if (conflicts.Count > 0)
        {
            var groups = conflicts.GroupBy(c => $"{c.ElementA}v{c.ElementB}");
            foreach (var g in groups)
                errors.Add($"{g.Key}:{g.Count()}");
        }

        return errors;
    }

    /// <summary>
    /// Rough mass estimate (kg) — solid cylinder approximation around the
    /// chamber radius with a 35% fill factor for channels / voids. No voxels.
    /// </summary>
    public static float MassEstimateKg(AeroSpec S)
    {
        float rOuter    = S.rShroudChamber + 3f;            // mm
        float vol_mm3   = MathF.PI * rOuter * rOuter * S.zTotal;
        const float fillFactor = 0.35f;
        return vol_mm3 * 1e-9f * S.rho * fillFactor;
    }

    /// <summary>Thrust-to-weight from the rough mass estimate.</summary>
    public static float TWRatio(AeroSpec S, float massKg) =>
        S.F_thrust / (massKg * 9.81f);

    /// <summary>
    /// The composite [0, 1] score, parametrized by <see cref="ScoringWeights"/>.
    /// This is the ONE scoring formula. It is fed the raw metrics so both the
    /// grid sweep (live AeroSpec) and the outer sweep (stored SweepResult
    /// metrics) produce numerically identical scores from identical inputs.
    ///
    /// With <see cref="ScoringWeights.Default"/> this reproduces the original
    /// hardcoded constants exactly (0.30/0.20/0.25/0.15/0.10 objectives,
    /// 0.10/0.02/0.30/0.10 penalties).
    /// </summary>
    /// <param name="isp_SL">Sea-level specific impulse (s).</param>
    /// <param name="twRatio">Thrust-to-weight ratio.</param>
    /// <param name="sigmaThermal_MPa">Throat thermal stress (MPa).</param>
    /// <param name="zTotal_mm">Total engine length (mm).</param>
    /// <param name="spatialConflicts">Spatial-validator conflict count.</param>
    /// <param name="sigmaYield_MPa">Material yield strength (MPa).</param>
    /// <param name="circThroat">Available throat circumference (mm).</param>
    /// <param name="neededThroat">Circumference needed for the channels (mm).</param>
    /// <param name="throatGap">Shroud-minus-spike radial gap at throat (mm).</param>
    /// <param name="vCoolMax">Final coolant velocity (m/s).</param>
    public static float Score(
        ScoringWeights w,
        float isp_SL,
        float twRatio,
        float sigmaThermal_MPa,
        float zTotal_mm,
        int   spatialConflicts,
        float sigmaYield_MPa,
        float circThroat,
        float neededThroat,
        float throatGap,
        float vCoolMax)
    {
        float score = 0f;

        // ── Positive objectives ──

        // Isp: 250s→0, 350s→1
        float ispNorm = Math.Clamp((isp_SL - 250f) / 100f, 0f, 1f);
        score += w.wIsp * ispNorm;

        // T/W: 50→0, 500→1
        float twNorm = Math.Clamp((twRatio - 50f) / 450f, 0f, 1f);
        score += w.wTW * twNorm;

        // Thermal margin: σ_th far below yield = good
        float sigmaRatio    = sigmaThermal_MPa / sigmaYield_MPa;
        float thermalMargin = Math.Clamp(1f - sigmaRatio, 0f, 1f);
        score += w.wThermal * thermalMargin;

        // Channel fit: circumference vs needed
        float fitRatio = Math.Clamp(circThroat / MathF.Max(neededThroat, 1f), 0f, 2f) / 2f;
        score += w.wChannelFit * fitRatio;

        // Compactness: 80mm→1, 280mm→0
        float lengthNorm = Math.Clamp(1f - (zTotal_mm - 80f) / 200f, 0f, 1f);
        score += w.wCompactness * lengthNorm;

        // ── Penalties (subtractive) ──

        // Thermal stress above yield
        if (sigmaThermal_MPa > sigmaYield_MPa)
        {
            float over = (sigmaThermal_MPa - sigmaYield_MPa) / 100f;
            score -= w.penaltyOverstress * over;
        }

        // Spatial conflicts
        score -= w.penaltyConflict * spatialConflicts;

        // Throat gap too small
        if (throatGap < 1.5f) score -= w.penaltyThroatGap;

        // Coolant velocity too high
        if (vCoolMax > 50f) score -= w.penaltyCoolant;

        return Math.Clamp(score, 0f, 1f);
    }

    /// <summary>
    /// Convenience overload that derives every scoring input directly from a
    /// fully-computed <see cref="AeroSpec"/>. Used by the grid sweep, which has
    /// the live spec in hand.
    /// </summary>
    public static float Score(ScoringWeights w, AeroSpec S, float sigmaThermal_MPa, int spatialConflicts)
    {
        float massKg = MassEstimateKg(S);
        float tw     = TWRatio(S, massKg);

        float circThroat   = 2f * MathF.PI * (S.rShroudThroat + 2f);
        float neededThroat = S.nChannelsShroud * (S.chRadiusMin * 2f + S.minRibWall);
        float throatGap    = S.rShroudThroat - S.rSpikeThroat;

        return Score(
            w,
            isp_SL:           S.Isp_SL,
            twRatio:          tw,
            sigmaThermal_MPa: sigmaThermal_MPa,
            zTotal_mm:        S.zTotal,
            spatialConflicts: spatialConflicts,
            sigmaYield_MPa:   S.sigma_yield / 1e6f,
            circThroat:       circThroat,
            neededThroat:     neededThroat,
            throatGap:        throatGap,
            vCoolMax:         S.v_cool_max);
    }
}
