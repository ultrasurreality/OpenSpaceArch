// DesignSweep.cs — Перебор вариантов без вокселей
//
// Чистая математика: физика + валидация + ранжирование
// 1000 вариантов за ~1 секунду
//
// Использование: dotnet run -- sweep
// Или из кода: DesignSweep.Run()

using System.Diagnostics;
using OpenSpaceArch.Engine.CSP;

namespace OpenSpaceArch.Engine;

public static class DesignSweep
{
    // Результат одного варианта
    public record DesignResult(
        float Thrust,       // N
        float Pc_bar,       // bar
        float OF,           // O/F ratio
        float SF,           // safety factor
        float Twist,        // helical turns
        float CR,           // contraction ratio
        float Isp_SL,       // s
        float Isp_vac,      // s
        float mDot,         // kg/s
        float TotalLength,  // mm
        float MassEstimate, // kg (грубая оценка без вокселей)
        float TWRatio,      // thrust / (mass * g)
        float qThroat_MW,   // MW/m²
        float wallThroat,   // mm
        float chRadiusMin,  // mm
        float nChannels,    // shroud
        float vCoolMax,     // m/s (итоговая после self-iteration)
        float sigma_thermal,// MPa
        float Score,        // weighted composite score (0-1, higher = better)
        bool  IsValid,      // прошёл все проверки
        string Errors        // что не так
    );

    // Default scoring preset for the grid sweep. The grid sweep is fixed-weight
    // (no live sliders), so it uses the same default constants the Phase 2
    // SweepPanel ships with — kept as one shared instance to avoid allocating
    // per variant inside the 8640-iteration loop.
    static readonly ScoringWeights _gridWeights = ScoringWeights.Default();

    // Weighted composite scoring (PicoGK-Nozzle pattern).
    // Delegates to the single shared formula in EngineEvaluation so the grid
    // sweep, the outer random sweep, and SweepResult never drift apart. With
    // the default weights this is numerically identical to the old hardcoded
    // 0.30/0.20/0.25/0.15/0.10 objectives and 0.10/0.02/0.30/0.10 penalties.
    static float ComputeScore(AeroSpec S, float sigma_th_MPa, int spatialConflicts) =>
        EngineEvaluation.Score(_gridWeights, S, sigma_th_MPa, spatialConflicts);

    // Валидация одного варианта
    static DesignResult Evaluate(float thrust, float pc_bar, float of_ratio, float sf, float twist, float cr)
    {
        var S = new AeroSpec
        {
            F_thrust = thrust,
            Pc = pc_bar * 1e5f,
            OF_ratio = of_ratio,
            SF = sf,
            channelTwistTurns = twist,
            CR = cr,
            minRibWall = 0.5f  // реальный LPBF min, не 3×voxel
        };

        var errors = new List<string>();

        // Физика (без Library.Log — перехватываем)
        try
        {
            ComputeSilent(S);
        }
        catch
        {
            return new DesignResult(
                thrust, pc_bar, of_ratio, sf, twist, cr,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                false, "PHYSICS FAILED");
        }

        // ── Проверки ──
        // Термический стресс (warning only — print-time laser
        // modulation). НЕ блокируем — это ограничение материала, не геометрии.
        float sigma_th_MPa = EngineEvaluation.ThermalStressMPa(S);

        // Все геометрические / производственные предикаты — общий хелпер
        // (тот же набор, что использует outer sweep).
        errors.AddRange(EngineEvaluation.RunValidityChecks(S, out int spatialConflicts));

        // Грубая оценка массы (цилиндр с каналами, без вокселей)
        float massKg = EngineEvaluation.MassEstimateKg(S);
        float tw     = EngineEvaluation.TWRatio(S, massKg);

        // Composite score
        float score = ComputeScore(S, sigma_th_MPa, spatialConflicts);

        return new DesignResult(
            Thrust: thrust,
            Pc_bar: pc_bar,
            OF: of_ratio,
            SF: sf,
            Twist: twist,
            CR: cr,
            Isp_SL: S.Isp_SL,
            Isp_vac: S.Isp_vac,
            mDot: S.mDot,
            TotalLength: S.zTotal,
            MassEstimate: massKg,
            TWRatio: tw,
            qThroat_MW: S.qThroat / 1e6f,
            wallThroat: S.wallThroat,
            chRadiusMin: S.chRadiusMin,
            nChannels: S.nChannelsShroud,
            vCoolMax: S.v_cool_max,
            sigma_thermal: sigma_th_MPa,
            Score: score,
            IsValid: errors.Count == 0,
            Errors: errors.Count == 0 ? "OK" : string.Join("; ", errors)
        );
    }

    // Физика без Library.Log
    static void ComputeSilent(AeroSpec S)
    {
        // Термохимия — O/F-dependent interpolation
        float g0 = 9.80665f;
        float R0 = 8314f;
        float Pa_SL = 101325f;

        var (Tc, gamma, MW, cStar) = Thermochemistry.InterpolateCEA(S.OF_ratio);
        // First-order chamber-pressure correction (dissociation suppression).
        // MUST match Thermochemistry.Compute — without it the sweep would rank
        // designs on uncorrected 50-bar chemistry while the single build uses
        // corrected chemistry, so the two authoritative physics paths disagree
        // for Pc != Pc_cal (the sweep samples Pc over 50–150 bar). No-op at 50 bar.
        (Tc, gamma, MW, cStar) = Thermochemistry.ApplyPressureCorrection(S.Pc, Tc, gamma, MW, cStar);
        S.Tc = Tc;
        S.gamma = gamma;
        S.molWeight = MW;
        S.cStar = cStar;
        float TcRef = 3492f;
        S.mu_gas = 8.5e-5f * MathF.Pow(S.Tc / TcRef, 0.7f);
        S.Cp_transport = 2200f;
        S.Pr_gas = 0.55f;
        S.R_gas = R0 / S.molWeight;
        S.a_sound = MathF.Sqrt(S.gamma * S.R_gas * S.Tc);

        float g = S.gamma;
        float pressureRatio = Pa_SL / S.Pc;
        float exponent = (g - 1f) / g;
        S.Cf = MathF.Sqrt(
            (2f * g * g / (g - 1f))
            * MathF.Pow(2f / (g + 1f), (g + 1f) / (g - 1f))
            * (1f - MathF.Pow(pressureRatio, exponent)));
        float Cf_vac = S.Cf * 1.08f;
        S.Isp_SL = S.cStar * S.Cf / g0;
        S.Isp_vac = S.cStar * Cf_vac / g0;
        S.mDot = S.F_thrust / (S.Isp_SL * g0);

        // ChamberSizing — inline
        S.At = S.F_thrust / (S.Cf * S.Pc);
        S.Dt = 2f * MathF.Sqrt(S.At / MathF.PI);
        float gapFactor = 1f - MathF.Pow(1f - S.throatGapRatio, 2f);
        float rShroud_m = MathF.Sqrt(S.At / (MathF.PI * gapFactor));
        float rSpike_m = rShroud_m * (1f - S.throatGapRatio);
        S.rShroudThroat = rShroud_m * 1000f;
        S.rSpikeThroat = rSpike_m * 1000f;
        S.rSpikeChamber = S.rSpikeThroat * 1.2f;
        float AcNeeded = S.CR * S.At;
        float rSpikeCh_m = S.rSpikeChamber / 1000f;
        float rShroudCh_m = MathF.Sqrt(AcNeeded / MathF.PI + rSpikeCh_m * rSpikeCh_m);
        S.rShroudChamber = rShroudCh_m * 1000f;
        S.rSpikeTip = MathF.Max(S.voxelSize * 3f, 1.5f);
        float Ac = AcNeeded;
        float Lc_m = S.Lstar * S.At / Ac;
        S.Lc = Lc_m * 1000f;
        float deltaR_shroud = S.rShroudChamber - S.rShroudThroat;
        float tanAngle = MathF.Tan(S.convergentHalfAngle * MathF.PI / 180f);
        S.convergentDz = deltaR_shroud / tanAngle;
        S.domeDz = S.rSpikeChamber;
        float throatGap = S.rShroudThroat - S.rSpikeThroat;
        float spikeBelowThroat = throatGap * 8f;
        S.zTip = 0f;
        S.zThroat = spikeBelowThroat;
        S.zCowl = S.zThroat - throatGap * 1.5f;
        S.zChBot = S.zThroat + S.convergentDz;
        S.zChTop = S.zChBot + S.Lc;
        S.zInjector = S.zChTop + S.domeDz;
        S.zTotal = S.zInjector + 4f;
        S.mDot = S.Pc * S.At / S.cStar;

        // HeatTransfer — inline
        float Dt_m = S.Dt;
        float Rc = 0.382f * Dt_m / 2f;
        float recoveryFactor = MathF.Pow(S.Pr_gas, 0.33f);
        float T_aw = recoveryFactor * S.Tc;
        float T_wg = S.T_max_service;
        float baseBartz =
            0.026f / MathF.Pow(Dt_m, 0.2f)
            * MathF.Pow(S.mu_gas, 0.2f) * S.Cp_transport / MathF.Pow(S.Pr_gas, 0.6f)
            * MathF.Pow(S.Pc / S.cStar, 0.8f)
            * MathF.Pow(Dt_m / Rc, 0.1f);
        float hg_throat = baseBartz;
        float q_raw = hg_throat * (T_aw - T_wg);
        float filmReduction = 1f - S.filmCoolFraction * S.filmCoolEffectiveness
            * (T_aw - 300f) / (T_aw - T_wg);
        filmReduction = Math.Clamp(filmReduction, 0.3f, 1f);
        S.qThroat = q_raw * filmReduction;
        float rLocal_throat = MathF.Max(S.rShroudThroat, S.rSpikeThroat) / 1000f;
        float t_pressure = S.Pc * rLocal_throat / (S.sigma_yield / S.SF) * 1000f;
        S.wallThroat = MathF.Max(t_pressure, S.minPrintWall);

        S.mDot_fuel = S.mDot / (1f + S.OF_ratio);
        S.mDot_ox = S.mDot - S.mDot_fuel;
        S.mDot_cool_spike = S.mDot_ox * S.spikeCoolFraction;

        // Channel count iteration
        int N = 20;
        float rCh = 1f;
        for (int iter = 0; iter < 5; iter++)
        {
            float mPerCh = S.mDot_fuel / N;
            float A = mPerCh / (S.rho_coolant_shroud * S.v_cool_max);
            rCh = MathF.Max(MathF.Sqrt(A / MathF.PI) * 1000f, S.minChannel / 2f);
            float circ = 2f * MathF.PI * S.rShroudThroat;
            N = (int)MathF.Floor(circ / (2f * rCh + S.minPrintWall));
            N = Math.Clamp(N, 4, 64);
        }
        S.nChannelsShroud = N;
        S.chRadiusMin = rCh;
        float mPerChFinal = S.mDot_fuel / S.nChannelsShroud;
        float Ach = mPerChFinal / (S.rho_coolant_shroud * S.v_cool_min);
        S.chRadiusMax = MathF.Sqrt(Ach / MathF.PI) * 1000f;

        // Self-iteration for fitting
        float origVMax = S.v_cool_max;
        for (int attempt = 0; attempt < 10; attempt++)
        {
            bool fit = true;
            for (float zCheck = S.zCowl; zCheck <= S.zInjector; zCheck += 2f)
            {
                float rSh = ChamberSizing.ShroudProfile(S, zCheck);
                if (rSh < 2f) continue;
                var (cw, ch) = HeatTransfer.ChannelRect(S, zCheck);
                float wall = HeatTransfer.WallThickness(S, zCheck);
                float circ = 2f * MathF.PI * (rSh + wall + ch / 2f);
                float needed = S.nChannelsShroud * (cw + S.minRibWall);
                if (needed > circ * 0.95f) { fit = false; break; }
            }
            if (fit) break;
            S.v_cool_max *= 1.15f;
            S.v_cool_min *= 1.15f;
            N = 20; rCh = 1f;
            for (int iter = 0; iter < 5; iter++)
            {
                float mpc = S.mDot_fuel / N;
                float Ax = mpc / (S.rho_coolant_shroud * S.v_cool_max);
                rCh = MathF.Max(MathF.Sqrt(Ax / MathF.PI) * 1000f, S.minChannel / 2f);
                float cc = 2f * MathF.PI * S.rShroudThroat;
                N = (int)MathF.Floor(cc / (2f * rCh + S.minPrintWall));
                N = Math.Clamp(N, 4, 64);
            }
            S.nChannelsShroud = N;
            S.chRadiusMin = rCh;
            mPerChFinal = S.mDot_fuel / S.nChannelsShroud;
            Ach = mPerChFinal / (S.rho_coolant_shroud * S.v_cool_min);
            S.chRadiusMax = MathF.Sqrt(Ach / MathF.PI) * 1000f;
        }
    }

    /// <summary>
    /// Public wrapper around the silent physics pipeline for the Phase 2
    /// outer sweep (<see cref="CSP.OuterSweep"/>). Same implementation as
    /// <see cref="ComputeSilent"/> — exposes it to code outside this class
    /// without duplicating the 140-line physics sequence.
    /// </summary>
    public static void ComputeSilentPublic(AeroSpec S) => ComputeSilent(S);

    public static void RunSingle(AeroSpec S)
    {
        var sw = Stopwatch.StartNew();
        ComputeSilent(S);
        sw.Stop();
        Console.WriteLine($"Physics: {sw.ElapsedTicks * 1000000 / Stopwatch.Frequency} µs");
        Console.WriteLine($"  Isp={S.Isp_SL:F1}s, ṁ={S.mDot:F3}, q={S.qThroat/1e6:F1} MW/m², wall={S.wallThroat:F2}mm");
        Console.WriteLine($"  N={S.nChannelsShroud}, r_ch={S.chRadiusMin:F2}mm, v_max={S.v_cool_max:F1}m/s");
        SpatialValidator.PrintReport(S);
    }

    public static void Run()
    {
        Console.WriteLine("╔══════════════════════════════════════════╗");
        Console.WriteLine("║  Design Space Sweep — Physics Only       ║");
        Console.WriteLine("╚══════════════════════════════════════════╝\n");

        // Sweep ranges — ~10k variants (focused around sweet spot)
        float[] thrusts = { 1000, 1500, 2000, 3000, 4000, 5000, 7500, 10000 };
        float[] pressures = new float[15];
        for (int i = 0; i < 15; i++) pressures[i] = 20 + i * 5; // 20..90 step 5
        float[] of_ratios = { 2.6f, 2.8f, 3.0f, 3.2f, 3.4f, 3.6f };
        float[] safetyFactors = { 1.5f, 2.0f };
        float[] twists = { 1.5f, 2.0f };
        float[] crs = { 3.0f, 4.0f, 5.0f };
        // 8 × 15 × 6 × 2 × 2 × 3 = 8,640

        var results = new List<DesignResult>();
        var sw = Stopwatch.StartNew();

        foreach (float f in thrusts)
        foreach (float p in pressures)
        foreach (float of in of_ratios)
        foreach (float sf in safetyFactors)
        foreach (float tw in twists)
        foreach (float cr in crs)
            results.Add(Evaluate(f, p, of, sf, tw, cr));

        sw.Stop();

        // Stats
        int total = results.Count;
        int valid = results.Count(r => r.IsValid);

        Console.WriteLine($"Evaluated {total} variants in {sw.ElapsedMilliseconds} ms");
        Console.WriteLine($"Valid: {valid}/{total} ({100f*valid/total:F0}%)\n");

        // Error breakdown
        var errorGroups = results.Where(r => !r.IsValid)
            .SelectMany(r => r.Errors.Split("; ").Select(e => e.Split('=')[0].Trim()))
            .GroupBy(e => e)
            .OrderByDescending(g => g.Count());
        Console.WriteLine("Error breakdown:");
        foreach (var g in errorGroups)
            Console.WriteLine($"  {g.Key}: {g.Count()}");

        // Top 20 by composite SCORE (all variants, not just "valid")
        var allSorted = results.Where(r => r.Score > 0).OrderByDescending(r => r.Score).ToList();

        Console.WriteLine($"\nScore distribution: max={allSorted.FirstOrDefault()?.Score:F3}, " +
            $"median={allSorted.ElementAtOrDefault(allSorted.Count/2)?.Score:F3}, " +
            $"min={allSorted.LastOrDefault()?.Score:F3}");

        Console.WriteLine($"\n── Top 20 by Score ──\n");
        Console.WriteLine(
            $"{"Score",6} {"F(N)",7} {"Pc",4} {"O/F",4} {"SF",4} {"CR",4} " +
            $"{"Isp",5} {"T/W",5} {"q(MW)",6} {"σ_th",5} {"wall",5} {"N",3} {"Status",-12}");
        Console.WriteLine(new string('─', 100));

        foreach (var r in allSorted.Take(20))
        {
            Console.WriteLine(
                $"{r.Score,6:F3} {r.Thrust,7:F0} {r.Pc_bar,4:F0} {r.OF,4:F1} {r.SF,4:F1} {r.CR,4:F1} " +
                $"{r.Isp_SL,5:F0} {r.TWRatio,5:F0} {r.qThroat_MW,6:F1} {r.sigma_thermal,5:F0} " +
                $"{r.wallThroat,5:F2} {r.nChannels,3:F0} {(r.IsValid ? "OK" : "warn"),-12}");
        }

        // Best Score per thrust bucket — NOT a Pareto front, just the single
        // top-scored variant at each thrust level. Useful as a "one engine per
        // size class" shortlist, but it collapses the multi-objective trade-off
        // into the scalar Score. The real trade-off surface is printed below.
        Console.WriteLine($"\n── Best Score per thrust bucket ──\n");
        Console.WriteLine(
            $"{"Score",6} {"F(N)",7} {"Pc",4} {"O/F",4} {"SF",4} {"CR",4} " +
            $"{"Isp",5} {"T/W",5} {"σ_th",5}");
        Console.WriteLine(new string('─', 70));

        var perThrustBest = BestScorePerThrust(allSorted);
        foreach (var r in perThrustBest)
        {
            Console.WriteLine(
                $"{r.Score,6:F3} {r.Thrust,7:F0} {r.Pc_bar,4:F0} {r.OF,4:F1} {r.SF,4:F1} {r.CR,4:F1} " +
                $"{r.Isp_SL,5:F0} {r.TWRatio,5:F0} {r.sigma_thermal,5:F0}");
        }

        // REAL Pareto front over (max Isp, min mass, min thermal stress).
        // These are the genuinely non-dominated designs: no other variant beats
        // any one of them on all three objectives simultaneously.
        var pareto = ParetoFront(allSorted);
        Console.WriteLine(
            $"\n── Pareto front (max Isp · min mass · min σ_th): " +
            $"{pareto.Count} non-dominated of {allSorted.Count} ──\n");
        Console.WriteLine(
            $"{"Score",6} {"F(N)",7} {"Pc",4} {"O/F",4} {"SF",4} {"CR",4} " +
            $"{"Isp",5} {"mass",6} {"σ_th",5}");
        Console.WriteLine(new string('─', 76));

        foreach (var r in pareto.OrderByDescending(r => r.Isp_SL))
        {
            Console.WriteLine(
                $"{r.Score,6:F3} {r.Thrust,7:F0} {r.Pc_bar,4:F0} {r.OF,4:F1} {r.SF,4:F1} {r.CR,4:F1} " +
                $"{r.Isp_SL,5:F0} {r.MassEstimate,6:F2} {r.sigma_thermal,5:F0}");
        }
    }

    /// <summary>
    /// "One engine per size class" shortlist: the single highest-<c>Score</c>
    /// variant within each distinct thrust value. This is a SCALARIZED view —
    /// it bakes the multi-objective trade-off into the weighted Score and is
    /// NOT a Pareto front. For the genuine trade-off surface use
    /// <see cref="ParetoFront"/>.
    /// </summary>
    public static List<DesignResult> BestScorePerThrust(IEnumerable<DesignResult> results) =>
        results
            .Where(r => r.Score > 0)
            .GroupBy(r => r.Thrust)
            .Select(g => g.OrderByDescending(r => r.Score).First())
            .OrderBy(r => r.Thrust)
            .ToList();

    /// <summary>
    /// Computes the TRUE non-dominated (Pareto-optimal) set over three
    /// competing objectives:
    /// <list type="bullet">
    ///   <item>maximize <c>Isp_SL</c> (higher specific impulse is better),</item>
    ///   <item>minimize <c>MassEstimate</c> (lighter is better),</item>
    ///   <item>minimize <c>sigma_thermal</c> (lower thermal stress is better).</item>
    /// </list>
    /// A design A <i>dominates</i> B when A is at least as good as B on every
    /// objective and strictly better on at least one. The Pareto front is the
    /// set of designs dominated by nobody — the honest engineering trade-off
    /// surface, with no scalar weighting collapsing the choices.
    ///
    /// Only physically-meaningful variants (<c>Isp_SL &gt; 0</c>, finite mass)
    /// are considered, so failed-physics rows do not pollute the front.
    /// O(n²) pairwise comparison — fine for the ~8.6k grid sweep.
    ///
    /// Thin wrapper over the generic <see cref="ParetoFront{T}"/> so the grid
    /// sweep, the outer sweep, and the viewer all share ONE domination kernel.
    /// </summary>
    public static List<DesignResult> ParetoFront(IEnumerable<DesignResult> results) =>
        ParetoFront(results, r => (r.Isp_SL, r.MassEstimate, r.sigma_thermal));

    /// <summary>
    /// Generic non-dominated (Pareto-optimal) front over the same three-objective
    /// trade-off — maximize the first selected value (Isp), minimize the second
    /// (mass) and third (thermal stress). The single domination kernel used by
    /// every caller: <see cref="DesignResult"/> (the grid sweep) goes through the
    /// overload above, and the Phase 2 viewer feeds <c>SweepResult</c> samples
    /// through this method directly so the score-ranking view and the Pareto view
    /// can never disagree about what "non-dominated" means.
    ///
    /// <paramref name="objectives"/> projects each item to
    /// <c>(ispMax, massMin, sigmaMin)</c>. Degenerate rows (non-positive Isp or
    /// mass, or negative stress) are filtered out so failed-physics samples never
    /// pollute the front. O(n²) pairwise comparison — fine for the ≤8.6k grid and
    /// the ≤5k viewer sweeps.
    /// </summary>
    public static List<T> ParetoFront<T>(
        IEnumerable<T> results,
        Func<T, (float ispMax, float massMin, float sigmaMin)> objectives)
    {
        // Project once, keep the original item alongside its 3 objectives so we
        // never re-invoke the selector inside the O(n²) inner loop.
        var pts = new List<(T Item, float Isp, float Mass, float Sigma)>();
        foreach (var r in results)
        {
            var (isp, mass, sigma) = objectives(r);
            if (isp > 0f && mass > 0f && sigma >= 0f)
                pts.Add((r, isp, mass, sigma));
        }

        // A dominates B  ⇔  A no worse on all 3 objectives AND strictly better on ≥1.
        static bool Dominates(
            (T Item, float Isp, float Mass, float Sigma) a,
            (T Item, float Isp, float Mass, float Sigma) b)
        {
            bool noWorse =
                a.Isp   >= b.Isp   &&   // max Isp
                a.Mass  <= b.Mass  &&   // min mass
                a.Sigma <= b.Sigma;     // min thermal stress

            bool strictlyBetter =
                a.Isp   >  b.Isp   ||
                a.Mass  <  b.Mass  ||
                a.Sigma <  b.Sigma;

            return noWorse && strictlyBetter;
        }

        var front = new List<T>();
        foreach (var candidate in pts)
        {
            bool dominated = false;
            foreach (var other in pts)
            {
                if (Dominates(other, candidate)) { dominated = true; break; }
            }
            if (!dominated)
                front.Add(candidate.Item);
        }
        return front;
    }
}
