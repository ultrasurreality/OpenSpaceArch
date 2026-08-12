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
    /// Composite [0, 1] score for this sample, parametrized by
    /// <see cref="ScoringWeights"/>. Delegates to the single shared formula in
    /// <see cref="EngineEvaluation.Score(ScoringWeights, float, float, float, float, int, float, float, float, float)"/>
    /// so the grid sweep and outer sweep can never drift apart. Channel-fit and
    /// throat-gap geometry are read from the stored <see cref="Spec"/> (its
    /// COMPUTED fields are already filled in).
    /// </summary>
    public float ComputeScore(ScoringWeights w)
    {
        float circThroat   = 2f * MathF.PI * (Spec.rShroudThroat + 2f);
        float neededThroat = Spec.nChannelsShroud * (Spec.chRadiusMin * 2f + Spec.minRibWall);
        float throatGap    = Spec.rShroudThroat - Spec.rSpikeThroat;

        return EngineEvaluation.Score(
            w,
            isp_SL:           Isp_SL,
            twRatio:          TWRatio,
            sigmaThermal_MPa: SigmaThermal_MPa,
            zTotal_mm:        zTotal_mm,
            spatialConflicts: SpatialConflicts,
            sigmaYield_MPa:   Spec.sigma_yield / 1e6f,
            circThroat:       circThroat,
            neededThroat:     neededThroat,
            throatGap:        throatGap,
            vCoolMax:         Spec.v_cool_max);
    }
}
