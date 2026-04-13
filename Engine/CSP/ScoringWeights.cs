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
