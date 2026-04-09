// DesignSweep.cs — Перебор вариантов без вокселей
//
// Чистая математика: физика + валидация + ранжирование
// 1000 вариантов за ~1 секунду
//
// Использование: dotnet run -- sweep
// Или из кода: DesignSweep.Run()

using System.Diagnostics;

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

    // Weighted composite scoring (PicoGK-Nozzle pattern)
    // Each metric normalized to 0-1, then weighted. Penalties subtract.
    static float ComputeScore(AeroSpec S, float sigma_th_MPa, int spatialConflicts)
    {
        float score = 0f;

        // ── Positive objectives (weights sum to 1.0) ──

        // Isp: higher = better. Normalize: 250s=0, 350s=1
        float ispNorm = Math.Clamp((S.Isp_SL - 250f) / 100f, 0f, 1f);
        score += 0.30f * ispNorm;

        // T/W: higher = better. Normalize via mass estimate
        float rOuter = S.rShroudChamber + 3f;
        float vol = MathF.PI * rOuter * rOuter * S.zTotal;
        float massKg = vol * 1e-9f * S.rho * 0.35f;
        float tw = S.F_thrust / (massKg * 9.81f);
        float twNorm = Math.Clamp((tw - 50f) / 450f, 0f, 1f); // 50=0, 500=1
        score += 0.20f * twNorm;

        // Thermal margin: σ_thermal far below yield = good
        float sigmaRatio = sigma_th_MPa / (S.sigma_yield / 1e6f);
        float thermalMargin = Math.Clamp(1f - sigmaRatio, 0f, 1f); // 1=no stress, 0=at yield
        score += 0.25f * thermalMargin;

        // Channel fit: channels easily fit in circumference = good
        float circThroat = 2f * MathF.PI * (S.rShroudThroat + 2f);
        float neededThroat = S.nChannelsShroud * (S.chRadiusMin * 2f + S.minRibWall);
        float fitRatio = Math.Clamp(circThroat / MathF.Max(neededThroat, 1f), 0f, 2f) / 2f;
        score += 0.15f * fitRatio;

        // Compactness: shorter engine = better (lighter structure, less material)
        float lengthNorm = Math.Clamp(1f - (S.zTotal - 80f) / 200f, 0f, 1f); // 80mm=1, 280mm=0
        score += 0.10f * lengthNorm;

        // ── Penalties (subtractive) ──

        // Thermal stress above yield: -0.1 per 100 MPa over
        if (sigma_th_MPa > S.sigma_yield / 1e6f)
        {
            float over = (sigma_th_MPa - S.sigma_yield / 1e6f) / 100f;
            score -= 0.10f * over;
        }

        // Spatial conflicts: -0.02 per conflict
        score -= 0.02f * spatialConflicts;

        // Throat gap too small: hard penalty
        float gap = S.rShroudThroat - S.rSpikeThroat;
        if (gap < 1.5f) score -= 0.3f;

        // Coolant velocity unreasonable
        if (S.v_cool_max > 50f) score -= 0.1f;

        return Math.Clamp(score, 0f, 1f);
    }

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

        // 1. Термический стресс (warning only — LEAP 71 solves via laser modulation)
        float deltaT = S.qThroat * (S.wallThroat / 1000f) / S.k_wall;
        float sigma_th = S.E_mod * S.alpha_CTE * deltaT / (1f - S.nu_poisson);
        float sigma_th_MPa = sigma_th / 1e6f;
        // НЕ блокируем — это ограничение материала, не геометрии

        // 2. Канал уже минимума для удаления порошка
        if (S.chRadiusMin * 2 < S.minChannel)
            errors.Add($"ch_dia={S.chRadiusMin*2:F2}<{S.minChannel}mm");

        // 3. Self-iteration не сошлась (скорость ушла слишком высоко)
        if (S.v_cool_max > 50f)
            errors.Add($"v_cool={S.v_cool_max:F0}>50m/s");

        // 4. Стенка тоньше минимума печати (не должно быть, но проверим)
        if (S.wallThroat < S.minPrintWall)
            errors.Add($"wall={S.wallThroat:F2}<{S.minPrintWall}mm");

        // 5. Throat gap слишком маленький для печати
        float throatGap = S.rShroudThroat - S.rSpikeThroat;
        if (throatGap < 1.5f)
            errors.Add($"gap={throatGap:F1}<1.5mm");

        // 6. Тепловой поток нереально высокий
        if (S.qThroat / 1e6f > 100f)
            errors.Add($"q={S.qThroat/1e6:F0}MW/m²");

        // 7. Пространственная валидация
        var spatialConflicts = SpatialValidator.Validate(S);
        if (spatialConflicts.Count > 0)
        {
            var groups = spatialConflicts.GroupBy(c => $"{c.ElementA}v{c.ElementB}");
            foreach (var g in groups)
                errors.Add($"{g.Key}:{g.Count()}");
        }

        // Грубая оценка массы (цилиндр с каналами, без вокселей)
        float rOuter = S.rShroudChamber + 3f; // mm, примерно
        float vol_mm3 = MathF.PI * rOuter * rOuter * S.zTotal; // грубый цилиндр
        float fillFactor = 0.35f; // ~35% заполнения (каналы, пустоты)
        float massKg = vol_mm3 * 1e-9f * S.rho * fillFactor;
        float tw = thrust / (massKg * 9.81f);

        // Composite score
        float score = ComputeScore(S, sigma_th_MPa, spatialConflicts.Count);

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

        // Pareto front: best Score per thrust
        Console.WriteLine($"\n── Pareto front: best Score per thrust ──\n");
        Console.WriteLine(
            $"{"Score",6} {"F(N)",7} {"Pc",4} {"O/F",4} {"SF",4} {"CR",4} " +
            $"{"Isp",5} {"T/W",5} {"σ_th",5}");
        Console.WriteLine(new string('─', 70));

        var pareto = allSorted
            .GroupBy(r => r.Thrust)
            .Select(g => g.OrderByDescending(r => r.Score).First())
            .OrderBy(r => r.Thrust);
        foreach (var r in pareto)
        {
            Console.WriteLine(
                $"{r.Score,6:F3} {r.Thrust,7:F0} {r.Pc_bar,4:F0} {r.OF,4:F1} {r.SF,4:F1} {r.CR,4:F1} " +
                $"{r.Isp_SL,5:F0} {r.TWRatio,5:F0} {r.sigma_thermal,5:F0}");
        }
    }
}
