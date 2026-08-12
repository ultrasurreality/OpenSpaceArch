// HeatTransfer.cs — Step 3: Bartz heat flux → wall thickness → channel sizing
// Source: REFERENCES.md (Bartz 1957; Huzel & Huang)
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

        // Recovery factor for adiabatic wall temperature
        float recoveryFactor = MathF.Pow(S.Pr_gas, 0.33f);  // ~0.82
        float T_aw = recoveryFactor * S.Tc;  // K

        // Assumed gas-side wall temperature (iterate if needed)
        float T_wg = S.T_max_service;  // K — design to material limit

        // ── Bartz at throat (peak heat flux)
        // hg = baseBartz × (At/A)^0.9 × σ,  where σ is the Mach/wall-temperature
        // boundary-layer correction (= 1 at the throat by construction below).
        // baseBartz collapses everything in the Bartz correlation that does NOT
        // vary along z; it is recomputed identically in BaseBartz(S) so the
        // per-station HeatFlux() and this throat anchor share one definition.
        float baseBartz = BaseBartz(S);

        // At throat: At/A = 1.0, M = 1.0 → σ/σ_throat = 1.0
        float hg_throat = baseBartz * MathF.Pow(1.0f, 0.9f);
        float q_raw = hg_throat * (T_aw - T_wg);

        // Film cooling: reduces effective T_aw near the wall
        // η_fc × fraction → effective reduction of driving temperature difference
        float filmReduction = FilmReduction(S, T_aw, T_wg);
        S.qThroat = q_raw * filmReduction;

        Library.Log($"  hg(throat)={hg_throat:F0} W/(m²·K)");
        Library.Log($"  T_aw={T_aw:F0} K, T_wg={T_wg:F0} K");
        Library.Log($"  q(raw)={q_raw/1e6:F1} MW/m², film={filmReduction:F2}, q(eff)={S.qThroat/1e6:F1} MW/m²");

        // ── Throat sanity anchor: cross-check the calibrated scalar S.qThroat
        // against an INDEPENDENT per-station Bartz evaluation at z=zThroat.
        // HeatFluxLocal() solves local Mach from the area ratio and rebuilds q
        // from σ(M)/area/ΔT/film. At the throat M must resolve to ≈1, so the two
        // numbers must agree closely; a large gap means the Mach inverter or
        // branch selection is broken. Tolerance ~15% per the fidelity spec.
        float qThroat_perStation = HeatFluxLocal(S, S.zThroat);
        float anchorDiff = S.qThroat > 1f
            ? MathF.Abs(qThroat_perStation - S.qThroat) / S.qThroat
            : 0f;
        Library.Log($"  q(throat) anchor: scalar={S.qThroat/1e6:F1} MW/m², " +
                    $"per-station={qThroat_perStation/1e6:F1} MW/m² (Δ={anchorDiff*100:F1}%)");
        if (anchorDiff > 0.15f)
            Library.Log($"  WARNING: per-station Bartz disagrees with throat anchor by " +
                        $"{anchorDiff*100:F1}% (>15%) — check Mach inversion at the throat");

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

    /// Derive port positions from chamber geometry (v7: ports are OUTPUT).
    /// Called after Compute() so z-stations are available.
    public static void DerivePortPositions(AeroSpec S)
    {
        // Fuel inlet — near cowl (cold CH4 enters at bottom of shroud channels)
        S.fuelPortZ   = S.zCowl + 3f;
        S.fuelPortPhi = 0f;                  // 0° = +X axis

        // LOX inlet — near injector (LOX enters at top, feeds spike channels)
        S.loxPortZ   = S.zInjector - 5f;
        S.loxPortPhi = MathF.PI;             // 180° = -X axis

        // Igniter — 30% up the chamber from bottom
        S.igniterPortZ   = S.zChBot + (S.zChTop - S.zChBot) * 0.3f;
        S.igniterPortPhi = MathF.PI / 2f;    // 90° = +Y axis
    }

    /// Heat flux q(z) at any axial station (W/m²)
    ///
    /// Per-station Bartz: instead of scaling the single throat value by area ratio
    /// alone, this recomputes the gas-side coefficient hg(z) from the LOCAL Mach
    /// number (solved from the local area ratio) and the Bartz boundary-layer /
    /// wall-temperature correction σ(M). The area term (At/A)^0.9 and the σ(M)
    /// correction together replace the old single (At/A)^0.9 factor. By
    /// construction the result equals the calibrated S.qThroat at z=zThroat
    /// (M→1, area ratio→1, σ/σ_throat→1), so the throat stays the peak anchor and
    /// every caller's q/qThroat ∈ [0,1] invariant is preserved (see numeric proof
    /// in the backlog note: σ/σ_throat·(At/A)^0.9 ≤ 1 on both Mach branches).
    public static float HeatFlux(AeroSpec S, float z)
        => HeatFluxLocal(S, z);

    /// Absolute per-station Bartz heat flux (W/m²) — the physics core behind
    /// HeatFlux(). Kept separate so Compute() can call it for the throat anchor
    /// cross-check without any dependency on the cached S.qThroat scalar.
    private static float HeatFluxLocal(AeroSpec S, float z)
    {
        float At_mm2 = S.At * 1e6f;
        float A_mm2 = ChamberSizing.AnnularArea(S, z);
        if (A_mm2 < 1f) return 0f;

        // Local area ratio At/A ∈ (0,1]; A ≥ At everywhere except the throat.
        float AtOverA = Math.Clamp(At_mm2 / A_mm2, 0.01f, 1.0f);
        float areaTerm = MathF.Pow(AtOverA, 0.9f);

        // Local Mach from the isentropic area–Mach relation. Below the throat
        // (toward the spike tip / external expansion) the flow is supersonic;
        // at and above the throat (convergent + chamber) it is subsonic.
        bool supersonic = z < S.zThroat;
        float areaRatio = 1f / AtOverA;                 // A/At ≥ 1
        float M = MachFromAreaRatio(S.gamma, areaRatio, supersonic);

        // Bartz σ correction, normalised by its throat value so the area term
        // alone sets the throat magnitude (σ(M=1)/σ_throat = 1).
        float sigmaNorm = BartzSigma(S, M) / BartzSigma(S, 1f);

        // Driving temperature difference + film reduction (same model as the
        // throat anchor in Compute()).
        float recoveryFactor = MathF.Pow(S.Pr_gas, 0.33f);
        float T_aw = recoveryFactor * S.Tc;
        float T_wg = S.T_max_service;
        float filmReduction = FilmReduction(S, T_aw, T_wg);

        float hg = BaseBartz(S) * areaTerm * sigmaNorm;
        return hg * (T_aw - T_wg) * filmReduction;       // W/m²
    }

    /// z-independent part of the Bartz correlation (W/(m²·K) up to the area+σ
    /// factors). Depends only on throat diameter and gas transport properties —
    /// all already-populated public AeroSpec fields. Single source of truth so
    /// Compute()'s throat value and the per-station HeatFlux agree exactly.
    private static float BaseBartz(AeroSpec S)
    {
        float Dt_m = S.Dt;
        float Rc   = 0.382f * Dt_m / 2f;            // throat radius of curvature
        return 0.026f / MathF.Pow(Dt_m, 0.2f)
            * MathF.Pow(S.mu_gas, 0.2f) * S.Cp_transport / MathF.Pow(S.Pr_gas, 0.6f)
            * MathF.Pow(S.Pc / S.cStar, 0.8f)
            * MathF.Pow(Dt_m / Rc, 0.1f);
    }

    /// Bartz boundary-layer / wall-temperature correction factor σ(M).
    /// σ = 1 / [ (½·(Twg/Tc)·(1+(γ-1)/2·M²) + ½)^(0.8-0.2ω) · (1+(γ-1)/2·M²)^(0.2ω) ]
    /// with ω≈0.6 (viscosity–temperature exponent). Standard form, e.g.
    /// Huzel & Huang, "Modern Engineering for Design of LRPE", Bartz correlation.
    private static float BartzSigma(AeroSpec S, float M)
    {
        const float omega = 0.6f;
        float stag = 1f + 0.5f * (S.gamma - 1f) * M * M;   // 1 + (γ-1)/2·M²
        float Twg_over_Tc = S.T_max_service / S.Tc;
        float term1 = MathF.Pow(0.5f * Twg_over_Tc * stag + 0.5f, 0.8f - 0.2f * omega);
        float term2 = MathF.Pow(stag, 0.2f * omega);
        return 1f / (term1 * term2);
    }

    /// Film-cooling reduction of the gas-side driving ΔT (shared by the throat
    /// anchor and the per-station evaluation so they reconcile exactly).
    private static float FilmReduction(AeroSpec S, float T_aw, float T_wg)
    {
        float r = 1f - S.filmCoolFraction * S.filmCoolEffectiveness
            * (T_aw - 300f) / (T_aw - T_wg);   // 300K = film coolant temperature
        return Math.Clamp(r, 0.3f, 1f);
    }

    /// Invert the isentropic area–Mach relation A/At = f(M, γ) for M, on the
    /// requested branch. Exception-free bracketed bisection (the area–Mach
    /// relation is monotonic on each branch), so it is safe in the per-voxel
    /// hot path that PhysicsFields/HeatProfile drive HeatFlux() from.
    ///   subsonic   branch: M ∈ (eps, 1]      (chamber + convergent)
    ///   supersonic branch: M ∈ [1, Mmax]     (below throat, expanding flow)
    private static float MachFromAreaRatio(float gamma, float areaRatio, bool supersonic)
    {
        if (areaRatio <= 1f) return 1f;                 // throat (or numerical ≤1)

        float lo, hi;
        if (supersonic) { lo = 1f;       hi = 12f;   }  // M up to 12 → AR huge
        else            { lo = 0.001f;   hi = 1f;    }  // subsonic

        // AreaRatio(M) = (1/M)·[ (2/(γ+1))·(1+(γ-1)/2·M²) ]^((γ+1)/(2(γ-1)))
        float exp = (gamma + 1f) / (2f * (gamma - 1f));
        float ARof(float M)
        {
            float stag = 1f + 0.5f * (gamma - 1f) * M * M;
            return (1f / M) * MathF.Pow(2f / (gamma + 1f) * stag, exp);
        }

        // Clamp target into the achievable range of this branch to avoid running
        // off the monotone interval (e.g. AR beyond hi → return the hi Mach).
        float arHi = ARof(hi);
        if (supersonic && areaRatio >= arHi) return hi;

        // Fixed-iteration bisection (≈24 steps → ~1e-3 in M over [0,12]).
        for (int i = 0; i < 24; i++)
        {
            float mid = 0.5f * (lo + hi);
            float ar  = ARof(mid);
            // ARof is decreasing in M on the subsonic branch, increasing on the
            // supersonic branch — move the bound that keeps the root bracketed.
            bool tooSmall = supersonic ? (ar < areaRatio) : (ar > areaRatio);
            if (tooSmall) lo = mid; else hi = mid;
        }
        return 0.5f * (lo + hi);
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
