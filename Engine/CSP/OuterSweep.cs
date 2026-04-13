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
// See ARCHITECTURE.md, section
// "OUTER LAYER — Bounded Random Sweep".

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
        /// Optional progress callback invoked every ~50 samples with
        /// <c>(done, total)</c>. Called synchronously on the caller's thread.
        /// </summary>
        public Action<int, int>? OnProgress;
    }

    /// <summary>
    /// Runs the bounded random sweep synchronously and returns all results
    /// (valid + invalid). Caller sorts / filters / scores as needed.
    /// </summary>
    public static List<SweepResult> Run(Config cfg)
    {
        var results = new List<SweepResult>(cfg.NumSamples);

        for (int i = 0; i < cfg.NumSamples; i++)
        {
            var rng = new Random(cfg.MasterSeed + i);
            AeroSpec spec = cfg.Vars.Sample(rng, cfg.FixedThrust, cfg.FixedVoxel);

            results.Add(EvaluateOne(spec, cfg.MasterSeed + i));

            if (cfg.OnProgress != null && (i % 50 == 0))
                cfg.OnProgress(i, cfg.NumSamples);
        }

        cfg.OnProgress?.Invoke(cfg.NumSamples, cfg.NumSamples);
        return results;
    }

    /// <summary>
    /// Runs physics + the same 7 validity checks as
    /// <c>DesignSweep.Evaluate</c> on a single spec, then constructs a
    /// <see cref="SweepResult"/>. Catches physics exceptions so one bad
    /// sample does not abort the whole sweep.
    /// </summary>
    private static SweepResult EvaluateOne(AeroSpec spec, int seed)
    {
        var errors = new List<string>();

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

        // Thermal stress at throat (same formula as DesignSweep.Evaluate)
        float deltaT = spec.qThroat * (spec.wallThroat / 1000f) / spec.k_wall;
        float sigma_th = spec.E_mod * spec.alpha_CTE * deltaT / (1f - spec.nu_poisson);
        float sigma_th_MPa = sigma_th / 1e6f;

        // ── 7 validity checks ──

        // 1. Channel diameter below powder-removal minimum
        if (spec.chRadiusMin * 2 < spec.minChannel)
            errors.Add($"ch_dia={spec.chRadiusMin * 2:F2}<{spec.minChannel}mm");

        // 2. Self-iteration diverged (coolant velocity)
        if (spec.v_cool_max > 50f)
            errors.Add($"v_cool={spec.v_cool_max:F0}>50m/s");

        // 3. Wall thinner than printable minimum
        if (spec.wallThroat < spec.minPrintWall)
            errors.Add($"wall={spec.wallThroat:F2}<{spec.minPrintWall}mm");

        // 4. Throat gap too small to print
        float throatGap = spec.rShroudThroat - spec.rSpikeThroat;
        if (throatGap < 1.5f)
            errors.Add($"gap={throatGap:F1}<1.5mm");

        // 5. Unrealistic heat flux
        if (spec.qThroat / 1e6f > 100f)
            errors.Add($"q={spec.qThroat / 1e6:F0}MW/m²");

        // 6. Spatial validation (channels vs shroud vs spike geometry)
        var spatialConflicts = SpatialValidator.Validate(spec);
        if (spatialConflicts.Count > 0)
        {
            var groups = spatialConflicts.GroupBy(c => $"{c.ElementA}v{c.ElementB}");
            foreach (var g in groups)
                errors.Add($"{g.Key}:{g.Count()}");
        }

        // 7. (Thermal stress is warning-only per DesignSweep.Evaluate line 131 —
        //     we still store it in SigmaThermal_MPa so scoring can penalize it.)

        // ── Mass estimate (cylinder approximation, 35% fill factor) ──
        float rOuter = spec.rShroudChamber + 3f;
        float vol_mm3 = MathF.PI * rOuter * rOuter * spec.zTotal;
        const float fillFactor = 0.35f;
        float massKg = vol_mm3 * 1e-9f * spec.rho * fillFactor;
        float tw = spec.F_thrust / (massKg * 9.81f);

        return new SweepResult(
            Seed: seed,
            Spec: spec,
            Valid: errors.Count == 0,
            FailReasons: errors.Count == 0 ? "OK" : string.Join("; ", errors),
            Isp_SL: spec.Isp_SL,
            TWRatio: tw,
            MassEstimate_kg: massKg,
            SigmaThermal_MPa: sigma_th_MPa,
            SpatialConflicts: spatialConflicts.Count,
            zTotal_mm: spec.zTotal,
            qThroat_MW: spec.qThroat / 1e6f,
            ChRadiusMin_mm: spec.chRadiusMin,
            NChannels: spec.nChannelsShroud);
    }
}
