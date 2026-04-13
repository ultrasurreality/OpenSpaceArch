// SweepResult.cs — single-sample output from the Phase 2 outer sweep.
//
// Each SweepResult stores raw physics metrics (Isp, T/W, mass estimate,
// thermal stress, etc.) but NOT a score. The score is re-computed from
// ScoringWeights on every render pass so the user's weight sliders are
// instant — no need to re-sample on weight changes.
//
// The stored AeroSpec is a COPY after physics was computed, so it has
// all COMPUTED fields filled in (Tc, Cf, Dt, qThroat, nChannels, ...).
// This lets "Apply Best" push a complete winning spec to ControlPanel.

namespace OpenSpaceArch.Engine.CSP;

/// <summary>
/// One bounded-random-sweep sample: the AeroSpec that was generated,
/// whether it passed the 7 validity predicates, and raw performance
/// metrics. Score is computed on demand from <see cref="ScoringWeights"/>.
/// </summary>
public readonly record struct SweepResult(
    int Seed,
    AeroSpec Spec,
    bool Valid,
    string FailReasons,
    // Raw metrics (all in SI-ish units)
    float Isp_SL,
    float TWRatio,
    float MassEstimate_kg,
    float SigmaThermal_MPa,
    int SpatialConflicts,
    float zTotal_mm,
    float qThroat_MW,
    float ChRadiusMin_mm,
    int NChannels)
{
    /// <summary>
    /// Mirror of <c>DesignSweep.ComputeScore</c> but parametrized by
    /// <see cref="ScoringWeights"/> instead of the hardcoded constants.
    /// Reads only fields stored in this record plus <see cref="Spec"/> for
    /// material limits. Returns [0, 1] clamped.
    /// </summary>
    public float ComputeScore(ScoringWeights w)
    {
        float score = 0f;

        // ── Positive objectives ──

        // Isp: 250s→0, 350s→1
        float ispNorm = Math.Clamp((Isp_SL - 250f) / 100f, 0f, 1f);
        score += w.wIsp * ispNorm;

        // T/W: 50→0, 500→1
        float twNorm = Math.Clamp((TWRatio - 50f) / 450f, 0f, 1f);
        score += w.wTW * twNorm;

        // Thermal margin: σ_th far below yield = good
        float sigmaYield_MPa = Spec.sigma_yield / 1e6f;
        float sigmaRatio = SigmaThermal_MPa / sigmaYield_MPa;
        float thermalMargin = Math.Clamp(1f - sigmaRatio, 0f, 1f);
        score += w.wThermal * thermalMargin;

        // Channel fit: circumference vs needed
        float circThroat = 2f * MathF.PI * (Spec.rShroudThroat + 2f);
        float neededThroat = Spec.nChannelsShroud * (Spec.chRadiusMin * 2f + Spec.minRibWall);
        float fitRatio = Math.Clamp(circThroat / MathF.Max(neededThroat, 1f), 0f, 2f) / 2f;
        score += w.wChannelFit * fitRatio;

        // Compactness: 80mm→1, 280mm→0
        float lengthNorm = Math.Clamp(1f - (zTotal_mm - 80f) / 200f, 0f, 1f);
        score += w.wCompactness * lengthNorm;

        // ── Penalties ──

        // Thermal stress above yield
        if (SigmaThermal_MPa > sigmaYield_MPa)
        {
            float over = (SigmaThermal_MPa - sigmaYield_MPa) / 100f;
            score -= w.penaltyOverstress * over;
        }

        // Spatial conflicts
        score -= w.penaltyConflict * SpatialConflicts;

        // Throat gap too small
        float gap = Spec.rShroudThroat - Spec.rSpikeThroat;
        if (gap < 1.5f) score -= w.penaltyThroatGap;

        // Coolant velocity too high
        if (Spec.v_cool_max > 50f) score -= w.penaltyCoolant;

        return Math.Clamp(score, 0f, 1f);
    }
}
