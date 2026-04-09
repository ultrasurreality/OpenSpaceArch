// HeatTransfer.cs — Step 3: Bartz heat flux → wall thickness → channel sizing
// Source: Knowledge/Physics_Formulas/Bartz_Heat_Transfer.md
//
// GOTCHA: Use Cp_transport=2200, NOT Cp_equilibrium=6795!
// GOTCHA: AtOverA for annular geometry = (rSh_t²-rSp_t²)/(rSh²-rSp²)

using PicoGK;

namespace OpenSpaceArch.Engine;

public static class HeatTransfer
{
    public static void Compute(AeroSpec S)
    {
        Library.Log("── Step 3: Heat Transfer ──");

        float Dt_m = S.Dt;  // m — throat equivalent diameter
        float Rc = 0.382f * Dt_m / 2f;  // m — throat radius of curvature (standard)
        float At_mm2 = S.At * 1e6f;  // mm²

        // Recovery factor for adiabatic wall temperature
        float recoveryFactor = MathF.Pow(S.Pr_gas, 0.33f);  // ~0.82
        float T_aw = recoveryFactor * S.Tc;  // K

        // Assumed gas-side wall temperature (iterate if needed)
        float T_wg = S.T_max_service;  // K — design to material limit

        // ── Bartz at throat (peak heat flux)
        // hg = (0.026/Dt^0.2) × (μ^0.2 × Cp / Pr^0.6) × (Pc/c*)^0.8 × (Dt/Rc)^0.1 × (At/A)^0.9
        float baseBartz =
            0.026f / MathF.Pow(Dt_m, 0.2f)
            * MathF.Pow(S.mu_gas, 0.2f) * S.Cp_transport / MathF.Pow(S.Pr_gas, 0.6f)
            * MathF.Pow(S.Pc / S.cStar, 0.8f)
            * MathF.Pow(Dt_m / Rc, 0.1f);

        // At throat: At/A = 1.0
        float hg_throat = baseBartz * MathF.Pow(1.0f, 0.9f);
        float q_raw = hg_throat * (T_aw - T_wg);

        // Film cooling: reduces effective T_aw near the wall
        // η_fc × fraction → effective reduction of driving temperature difference
        float filmReduction = 1f - S.filmCoolFraction * S.filmCoolEffectiveness
            * (T_aw - 300f) / (T_aw - T_wg); // 300K = film coolant temperature
        filmReduction = Math.Clamp(filmReduction, 0.3f, 1f);
        S.qThroat = q_raw * filmReduction;

        Library.Log($"  hg(throat)={hg_throat:F0} W/(m²·K)");
        Library.Log($"  T_aw={T_aw:F0} K, T_wg={T_wg:F0} K");
        Library.Log($"  q(raw)={q_raw/1e6:F1} MW/m², film={filmReduction:F2}, q(eff)={S.qThroat/1e6:F1} MW/m²");

        // ── Wall thickness at throat (from pressure + LPBF minimum)
        // t = Pc × r / (σ_yield / SF)
        float rLocal_throat = MathF.Max(S.rShroudThroat, S.rSpikeThroat) / 1000f;  // m
        float t_pressure = S.Pc * rLocal_throat / (S.sigma_yield / S.SF) * 1000f;  // mm
        S.wallThroat = MathF.Max(t_pressure, S.minPrintWall);

        // ── Thermal stress check at throat
        float deltaT = S.qThroat * (S.wallThroat / 1000f) / S.k_wall;  // K
        float sigma_thermal = S.E_mod * S.alpha_CTE * deltaT / (1f - S.nu_poisson);

        Library.Log($"  wall(throat)={S.wallThroat:F2} mm (pressure={t_pressure:F2}, min={S.minPrintWall})");
        Library.Log($"  ΔT(throat)={deltaT:F0} K, σ_thermal={sigma_thermal/1e6:F0} MPa (yield={S.sigma_yield/1e6:F0})");

        // ── Coolant mass flow (propellant split)
        S.mDot_fuel = S.mDot / (1f + S.OF_ratio);
        S.mDot_ox = S.mDot - S.mDot_fuel;
        S.mDot_cool_spike = S.mDot_ox * S.spikeCoolFraction;

        // ── Channel count + radius: iterative solve
        // N = 2πR / (2·r_ch + wall),  r_ch = sqrt(ṁ/(N·ρ·v·π))
        int N = 20;
        float rCh = 1f;
        for (int iter = 0; iter < 5; iter++)
        {
            float mPerCh = S.mDot_fuel / N;
            float A = mPerCh / (S.rho_coolant_shroud * S.v_cool_max);
            rCh = MathF.Max(MathF.Sqrt(A / MathF.PI) * 1000f, S.minChannel / 2f);
            float circ = 2f * MathF.PI * S.rShroudThroat;
            N = (int)MathF.Floor(circ / (2f * rCh + S.minPrintWall));
            N = Math.Clamp(N, 8, 32);
        }
        S.nChannelsShroud = N;
        S.chRadiusMin = rCh;

        // Chamber radius at min velocity
        float mPerChFinal = S.mDot_fuel / S.nChannelsShroud;
        float Ach = mPerChFinal / (S.rho_coolant_shroud * S.v_cool_min);
        S.chRadiusMax = MathF.Sqrt(Ach / MathF.PI) * 1000f;

        // ── Spike channel count (LOX properties)
        int Ns = 12;
        float rChS = 1f;
        for (int iter = 0; iter < 5; iter++)
        {
            float mPerCh = S.mDot_cool_spike / Ns;
            float A = mPerCh / (S.rho_coolant_spike * S.v_cool_max);
            rChS = MathF.Max(MathF.Sqrt(A / MathF.PI) * 1000f, S.minChannel / 2f);
            float circ = 2f * MathF.PI * S.rSpikeThroat;
            Ns = (int)MathF.Floor(circ / (2f * rChS + S.minPrintWall));
            Ns = Math.Clamp(Ns, 4, 24);
        }
        S.nChannelsSpike = Ns;

        // ── Verification: channels fit?
        float circShroud = 2f * MathF.PI * S.rShroudThroat;
        float needShroud = S.nChannelsShroud * (2f * S.chRadiusMin + S.minPrintWall);
        if (needShroud > circShroud)
            Library.Log($"  WARNING: shroud channels don't fit! Need {needShroud:F1}mm, have {circShroud:F1}mm");

        // ── Self-iteration: verify channels fit at ALL z-stations
        float origVMax = S.v_cool_max;
        float origVMin = S.v_cool_min;
        for (int attempt = 0; attempt < 10; attempt++)
        {
            bool fit = true;
            for (float zCheck = S.zCowl; zCheck <= S.zInjector; zCheck += 2f)
            {
                float rSh = ChamberSizing.ShroudProfile(S, zCheck);
                if (rSh < 2f) continue;
                var (cw, ch) = ChannelRect(S, zCheck);
                float wall = WallThickness(S, zCheck);
                float circ = 2f * MathF.PI * (rSh + wall + ch / 2f);
                float needed = S.nChannelsShroud * (cw + S.minRibWall);
                if (needed > circ * 0.95f) { fit = false; break; }
            }
            if (fit) break;

            // Doesn't fit → increase velocity (shrinks channels), re-solve N
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
                N = Math.Clamp(N, 8, 32);
            }
            S.nChannelsShroud = N;
            S.chRadiusMin = rCh;
            mPerChFinal = S.mDot_fuel / S.nChannelsShroud;
            Ach = mPerChFinal / (S.rho_coolant_shroud * S.v_cool_min);
            S.chRadiusMax = MathF.Sqrt(Ach / MathF.PI) * 1000f;
            Library.Log($"  Iteration {attempt+1}: v_max={S.v_cool_max:F1} → N={S.nChannelsShroud}, R={S.chRadiusMin:F2}mm");
        }

        Library.Log($"  ṁ_fuel={S.mDot_fuel:F3} kg/s, ṁ_cool_spike={S.mDot_cool_spike:F3} kg/s");
        Library.Log($"  Shroud: {S.nChannelsShroud} ch × R={S.chRadiusMin:F2}mm (throat) → {S.chRadiusMax:F2}mm (chamber)");
        Library.Log($"  Spike:  {S.nChannelsSpike} ch × R={rChS:F2}mm (throat)");
        if (S.v_cool_max != origVMax)
            Library.Log($"  Self-iteration adjusted velocity: {origVMax:F1} → {S.v_cool_max:F1} m/s");
    }

    /// Heat flux q(z) at any axial station (W/m²)
    public static float HeatFlux(AeroSpec S, float z)
    {
        float At_mm2 = S.At * 1e6f;
        float A_mm2 = ChamberSizing.AnnularArea(S, z);
        if (A_mm2 < 1f) return 0f;

        float AtOverA = Math.Clamp(At_mm2 / A_mm2, 0.01f, 1.0f);
        float hgNorm = MathF.Pow(AtOverA, 0.9f);

        float recoveryFactor = MathF.Pow(S.Pr_gas, 0.33f);
        float T_aw = recoveryFactor * S.Tc;

        return S.qThroat * hgNorm;  // W/m² (scaled from throat peak)
    }

    /// Wall thickness at z (mm) — from pressure formula
    public static float WallThickness(AeroSpec S, float z)
    {
        float rLocal = MathF.Max(ChamberSizing.ShroudProfile(S, z),
                                  ChamberSizing.SpikeProfile(S, z));
        if (rLocal < 1f) return S.minPrintWall;

        float r_m = rLocal / 1000f;
        float t = S.Pc * r_m / (S.sigma_yield / S.SF) * 1000f;  // mm
        return MathF.Max(t, S.minPrintWall);
    }

    /// Cooling channel radius at z (mm) — from mass flow: r = sqrt(ṁ/(N·ρ·v·π))
    public static float ChannelRadius(AeroSpec S, float z)
    {
        float q = HeatFlux(S, z);
        if (q < 1f) return S.chRadiusMax;

        // Velocity proportional to heat flux: fast at throat (hot), slow in chamber (cool)
        float qNorm = Math.Clamp(q / S.qThroat, 0f, 1f);
        float v = S.v_cool_min + (S.v_cool_max - S.v_cool_min) * qNorm;

        // A = ṁ_per_ch / (ρ × v),  r = sqrt(A / π)
        float mPerCh = S.mDot_fuel / S.nChannelsShroud;
        float A = mPerCh / (S.rho_coolant_shroud * v);
        return MathF.Max(MathF.Sqrt(A / MathF.PI) * 1000f, S.minChannel / 2f);
    }

    /// Spike channel radius at z (mm) — LOX coolant, separate circuit
    public static float ChannelRadiusSpike(AeroSpec S, float z)
    {
        float q = HeatFlux(S, z);
        if (q < 1f) return S.chRadiusMax * 0.7f;

        float qNorm = Math.Clamp(q / S.qThroat, 0f, 1f);
        float v = S.v_cool_min + (S.v_cool_max - S.v_cool_min) * qNorm;

        float mPerCh = S.mDot_cool_spike / S.nChannelsSpike;
        float A = mPerCh / (S.rho_coolant_spike * v);
        return MathF.Max(MathF.Sqrt(A / MathF.PI) * 1000f, S.minChannel / 2f);
    }

    /// Rectangular channel dimensions at z (mm): width (tangential) × height (radial)
    /// Width from available circumference, height = A / width
    public static (float w, float h) ChannelRect(AeroSpec S, float z)
    {
        float q = HeatFlux(S, z);
        if (q < 1f) return (S.chRadiusMax * 2f, S.chRadiusMax);

        float qNorm = Math.Clamp(q / S.qThroat, 0f, 1f);
        float v = S.v_cool_min + (S.v_cool_max - S.v_cool_min) * qNorm;
        float mPerCh = S.mDot_fuel / S.nChannelsShroud;
        float A_mm2 = mPerCh / (S.rho_coolant_shroud * v) * 1e6f;

        // Width from circumferential spacing
        float rSh = ChamberSizing.ShroudProfile(S, z);
        float wall = WallThickness(S, z);
        float rCenter = rSh + wall + 2f;
        float w = 2f * MathF.PI * rCenter / S.nChannelsShroud - S.minRibWall;
        w = MathF.Max(w, S.minChannel);

        // Height from area / width
        float h = A_mm2 / w;
        h = MathF.Max(h, S.minChannel);

        // Aspect ratio limit 5:1
        if (w / h > 5f) h = w / 5f;
        if (h / w > 5f) w = h / 5f;

        return (w, h);
    }

    /// Spike rectangular channel dimensions (mm) — LOX coolant
    public static (float w, float h) ChannelRectSpike(AeroSpec S, float z)
    {
        float q = HeatFlux(S, z);
        if (q < 1f) return (S.chRadiusMax, S.chRadiusMax * 0.7f);

        float qNorm = Math.Clamp(q / S.qThroat, 0f, 1f);
        float v = S.v_cool_min + (S.v_cool_max - S.v_cool_min) * qNorm;
        float mPerCh = S.mDot_cool_spike / S.nChannelsSpike;
        float A_mm2 = mPerCh / (S.rho_coolant_spike * v) * 1e6f;

        float rSp = ChamberSizing.SpikeProfile(S, z);
        float wall = WallThickness(S, z);
        float rCenter = rSp - wall - 2f;
        if (rCenter < 2f) return (S.minChannel, S.minChannel);
        float w = 2f * MathF.PI * rCenter / S.nChannelsSpike - S.minRibWall;
        w = MathF.Max(w, S.minChannel);

        float h = A_mm2 / w;
        h = MathF.Max(h, S.minChannel);

        if (w / h > 5f) h = w / 5f;
        if (h / w > 5f) w = h / 5f;

        return (w, h);
    }
}
