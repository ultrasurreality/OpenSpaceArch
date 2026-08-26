// SPDX-License-Identifier: AGPL-3.0-or-later
//
// OpenSpaceArch — Open Computational Architecture for Aerospace Hardware
// Copyright (C) 2025-2026 ultrasurreality
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// OuterSweep.cs — Phase 2 bounded random sweep driver.
//
// The OUTER layer of the two-layer CSP architecture. Loops N times:
// for each seed, samples a random AeroSpec from SweepVariables, runs
// forward physics via DesignSweep.ComputeSilentPublic, applies the 7
// validity predicates from DesignSweep.Evaluate, and stores raw
// metrics in a SweepResult. Score is computed on demand from
// ScoringWeights — not stored here — so the viewer can re-score
// instantly when the user drags weight sliders.
//
// Physics-only, no voxel builds. 500 samples ≈ 60 ms on a laptop.
// Foreground execution is fine; background Task.Run is Phase 2.1.
//
// See ARCHITECTURE.md, "SEARCH (CSP layer)" — the outer bounded random sweep.

namespace OpenSpaceArch.Engine.CSP;

/// <summary>
/// Bounded random design-space sweep. Reuses <see cref="DesignSweep"/>
/// silent physics + validity check logic while replacing its 8640-variant
/// grid enumeration with seed-based bounded sampling.
/// </summary>
public static class OuterSweep
{
    /// <summary>Input config for <see cref="Run"/>.</summary>
    public sealed class Config
    {
        /// <summary>Which AeroSpec fields to vary and in what ranges.</summary>
        public SweepVariables Vars = new();

        /// <summary>Thrust is pinned (mission spec). Default 5 kN.</summary>
        public float FixedThrust = 5000f;

        /// <summary>Voxel size is pinned (manufacturing). Default 0.4 mm.</summary>
        public float FixedVoxel = 0.4f;

        /// <summary>Number of random samples to draw.</summary>
        public int NumSamples = 500;

        /// <summary>Master seed for reproducibility — sample i uses <c>new Random(MasterSeed + i)</c>.</summary>
        public int MasterSeed = 42;

        /// <summary>
        /// Sampling mode. <c>false</c> (default) = uniform over the full range
        /// box — the standard exploratory sweep. <c>true</c> = clamped Gaussian
        /// centered on each variable's nominal value, concentrating samples
        /// near the middle of the design box for a refinement pass. See
        /// <see cref="SweepVariables.Sample"/>.
        /// Ignored when <see cref="Adaptive"/> is on (the adaptive driver chooses
        /// uniform for explore and best-centered Gaussian for refine itself).
        /// </summary>
        public bool CenterBiased = false;

        /// <summary>
        /// Two-phase ADAPTIVE sweep. <c>false</c> (default) keeps the original
        /// single-batch behaviour (uniform, or center-biased per
        /// <see cref="CenterBiased"/>). <c>true</c> runs an initial UNIFORM
        /// exploration batch over the whole box, finds the best-scoring valid
        /// candidate, then spends the remaining budget on a REFINE batch of
        /// best-centered clamped Gaussians (<see cref="SweepVariables.SampleAround"/>)
        /// to hill-climb toward that optimum. Total sample count
        /// (<see cref="NumSamples"/>) and master seed are unchanged, so this is an
        /// additive, non-breaking refinement of the same budget.
        /// </summary>
        public bool Adaptive = false;

        /// <summary>
        /// Fraction of <see cref="NumSamples"/> spent on the uniform exploration
        /// batch when <see cref="Adaptive"/> is on; the remainder is the refine
        /// batch. Default 0.5 (half explore, half refine). Clamped to [0.1, 0.9]
        /// so neither phase is starved. Ignored when <see cref="Adaptive"/> is off.
        /// </summary>
        public float ExploreFraction = 0.5f;

        /// <summary>
        /// Std-dev of the refine-batch Gaussians as a fraction of each variable's
        /// range span (see <see cref="SweepVariables.SampleAround"/>). Smaller =
        /// tighter cloud around the best point. Default 0.12. Ignored when
        /// <see cref="Adaptive"/> is off.
        /// </summary>
        public float RefineStdFrac = 0.12f;

        /// <summary>
        /// Weights used ONLY to pick the best explore-batch candidate to center
        /// the refine batch on. Does not affect the metrics stored in the
        /// returned <see cref="SweepResult"/>s (those are weight-free; the viewer
        /// re-scores on demand). Defaults to <see cref="ScoringWeights.Default"/>.
        /// Ignored when <see cref="Adaptive"/> is off.
        /// </summary>
        public ScoringWeights RefineWeights = ScoringWeights.Default();

        /// <summary>
        /// Optional progress callback invoked every ~50 samples with
        /// <c>(done, total)</c>. Called synchronously on the caller's thread.
        /// </summary>
        public Action<int, int>? OnProgress;
    }

    /// <summary>
    /// Runs the bounded random sweep synchronously and returns all results
    /// (valid + invalid). Caller sorts / filters / scores as needed.
    ///
    /// With <see cref="Config.Adaptive"/> off (default) this is the original
    /// single-batch sweep — uniform, or center-biased per
    /// <see cref="Config.CenterBiased"/>. With it on, it runs the two-phase
    /// explore→refine driver (<see cref="RunAdaptive"/>). Either way the total is
    /// exactly <see cref="Config.NumSamples"/> and every sample i uses
    /// <c>new Random(MasterSeed + i)</c>, so the run is fully deterministic.
    /// </summary>
    public static List<SweepResult> Run(Config cfg)
    {
        return cfg.Adaptive ? RunAdaptive(cfg) : RunSingleBatch(cfg);
    }

    /// <summary>
    /// Original single-batch driver: draws <see cref="Config.NumSamples"/> specs,
    /// each from <c>new Random(MasterSeed + i)</c>, using uniform sampling (or
    /// center-biased per <see cref="Config.CenterBiased"/>).
    /// </summary>
    private static List<SweepResult> RunSingleBatch(Config cfg)
    {
        var results = new List<SweepResult>(cfg.NumSamples);

        for (int i = 0; i < cfg.NumSamples; i++)
        {
            var rng = new Random(cfg.MasterSeed + i);
            AeroSpec spec = cfg.Vars.Sample(rng, cfg.FixedThrust, cfg.FixedVoxel, cfg.CenterBiased);

            results.Add(EvaluateOne(spec, cfg.MasterSeed + i));

            if (cfg.OnProgress != null && (i % 50 == 0))
                cfg.OnProgress(i, cfg.NumSamples);
        }

        cfg.OnProgress?.Invoke(cfg.NumSamples, cfg.NumSamples);
        return results;
    }

    /// <summary>
    /// Two-phase adaptive driver. Phase 1: a UNIFORM exploration batch of
    /// <c>round(NumSamples · ExploreFraction)</c> samples scans the whole 6-D box.
    /// Phase 2: the best-scoring VALID explore candidate (ranked by
    /// <see cref="Config.RefineWeights"/>) becomes the center of a REFINE batch of
    /// best-centered clamped Gaussians (<see cref="SweepVariables.SampleAround"/>)
    /// that spends the remaining budget hill-climbing toward it.
    ///
    /// Determinism: explore sample i and refine sample j keep using
    /// <c>new Random(MasterSeed + i)</c> over a single continuous index range
    /// <c>[0, NumSamples)</c>, so two runs with the same config produce identical
    /// results. The returned list contains BOTH batches (explore first, then
    /// refine) — total length == <see cref="Config.NumSamples"/>.
    ///
    /// Fallback: if the explore batch finds no valid candidate to center on, the
    /// refine batch falls back to the same uniform sampling as explore, so the
    /// sweep degrades gracefully to a plain uniform run of the full budget.
    /// </summary>
    private static List<SweepResult> RunAdaptive(Config cfg)
    {
        int total = cfg.NumSamples;
        var results = new List<SweepResult>(total);

        // Split the budget. Clamp the fraction so neither phase is starved, and
        // guarantee at least 1 explore sample whenever there is any budget.
        float frac = Math.Clamp(cfg.ExploreFraction, 0.1f, 0.9f);
        int nExplore = (int)MathF.Round(total * frac);
        nExplore = Math.Clamp(nExplore, total > 0 ? 1 : 0, total);

        // ── Phase 1: uniform exploration over the whole box ──
        // Track the best valid candidate as we go so we don't re-score twice.
        // AeroSpec is a reference type; null until the first valid sample, gated
        // by haveBest so the refine phase never dereferences it.
        float bestScore = float.NegativeInfinity;
        AeroSpec? bestSpec = null;
        bool haveBest = false;

        for (int i = 0; i < nExplore; i++)
        {
            var rng = new Random(cfg.MasterSeed + i);
            AeroSpec spec = cfg.Vars.Sample(rng, cfg.FixedThrust, cfg.FixedVoxel, centerBiased: false);

            SweepResult r = EvaluateOne(spec, cfg.MasterSeed + i);
            results.Add(r);

            if (r.Valid)
            {
                float s = r.ComputeScore(cfg.RefineWeights);
                if (s > bestScore)
                {
                    bestScore = s;
                    bestSpec  = r.Spec;   // COMPUTED fields filled — fine, SampleAround reads only INPUTs
                    haveBest  = true;
                }
            }

            if (cfg.OnProgress != null && (i % 50 == 0))
                cfg.OnProgress(i, total);
        }

        // ── Phase 2: refine around the best explore candidate ──
        // If exploration found nothing valid, fall back to uniform so the second
        // batch still contributes useful coverage instead of clustering on junk.
        for (int i = nExplore; i < total; i++)
        {
            var rng = new Random(cfg.MasterSeed + i);
            AeroSpec spec = haveBest
                ? cfg.Vars.SampleAround(rng, cfg.FixedThrust, cfg.FixedVoxel, bestSpec!, cfg.RefineStdFrac)
                : cfg.Vars.Sample(rng, cfg.FixedThrust, cfg.FixedVoxel, centerBiased: false);

            results.Add(EvaluateOne(spec, cfg.MasterSeed + i));

            if (cfg.OnProgress != null && (i % 50 == 0))
                cfg.OnProgress(i, total);
        }

        cfg.OnProgress?.Invoke(total, total);
        return results;
    }

    /// <summary>
    /// Runs physics + the shared validity predicates
    /// (<see cref="EngineEvaluation.RunValidityChecks"/>, identical to the grid
    /// sweep) on a single spec, then constructs a <see cref="SweepResult"/>.
    /// Catches physics exceptions so one bad sample does not abort the sweep.
    /// </summary>
    private static SweepResult EvaluateOne(AeroSpec spec, int seed)
    {
        try
        {
            DesignSweep.ComputeSilentPublic(spec);
        }
        catch
        {
            return new SweepResult(
                Seed: seed,
                Spec: spec,
                Valid: false,
                FailReasons: "PHYSICS FAILED",
                Isp_SL: 0f,
                TWRatio: 0f,
                MassEstimate_kg: 0f,
                SigmaThermal_MPa: 0f,
                SpatialConflicts: 0,
                zTotal_mm: 0f,
                qThroat_MW: 0f,
                ChRadiusMin_mm: 0f,
                NChannels: 0);
        }

        // Thermal stress at throat — warning-only, stored so scoring can penalize.
        float sigma_th_MPa = EngineEvaluation.ThermalStressMPa(spec);

        // Shared validity predicates (same set the grid sweep uses).
        var errors = EngineEvaluation.RunValidityChecks(spec, out int spatialConflicts);

        // Shared rough mass / thrust-to-weight estimate.
        float massKg = EngineEvaluation.MassEstimateKg(spec);
        float tw     = EngineEvaluation.TWRatio(spec, massKg);

        return new SweepResult(
            Seed: seed,
            Spec: spec,
            Valid: errors.Count == 0,
            FailReasons: errors.Count == 0 ? "OK" : string.Join("; ", errors),
            Isp_SL: spec.Isp_SL,
            TWRatio: tw,
            MassEstimate_kg: massKg,
            SigmaThermal_MPa: sigma_th_MPa,
            SpatialConflicts: spatialConflicts,
            zTotal_mm: spec.zTotal,
            qThroat_MW: spec.qThroat / 1e6f,
            ChRadiusMin_mm: spec.chRadiusMin,
            NChannels: spec.nChannelsShroud);
    }
}
