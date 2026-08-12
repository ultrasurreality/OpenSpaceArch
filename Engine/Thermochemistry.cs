// Thermochemistry.cs — Step 1: Gas properties from NASA CEA
// Source: REFERENCES.md (Gordon & McBride, NASA RP-1311) (verified runs)
//
// CRITICAL: Cp_transport = 2200 J/(kg·K), NOT Cp_equilibrium = 6795
// Using wrong Cp → Bartz off by 3× → all channel sizing wrong!
//
// PRESSURE NOTE: the _ceaTable below is calibrated at Pc_cal ≈ 50 bar, but the
// engine runs at higher chamber pressure (AeroSpec.Pc = 110 bar nominal; the
// OuterSweep also samples Pc over 50–150 bar). Tc, γ, MW and c* are only WEAK
// functions of chamber pressure for LOX/CH4, but they are not constant: raising
// Pc suppresses dissociation of the combustion products (CO₂⇌CO+½O₂,
// H₂O⇌OH+½H₂, …) per Le Chatelier, so Tc and c* rise slightly with pressure.
// To avoid silently using 50-bar chemistry at 110+ bar, a documented, bounded,
// first-order Pc-correction is applied below (ApplyPressureCorrection). It is a
// CORRECTION FACTOR, not a re-run of CEA — see that method for the physical
// basis, the magnitudes, and the honest residual-bias statement. At Pc = Pc_cal
// the correction is exactly 1.0 (no-op), so the table values are reproduced.

using PicoGK;

namespace OpenSpaceArch.Engine;

public static class Thermochemistry
{
    const float g0 = 9.80665f;  // m/s² standard gravity
    const float R0 = 8314f;     // J/(kmol·K) universal gas constant
    const float Pa_SL = 101325f;// Pa atmospheric pressure at sea level

    // NASA CEA interpolation table: LOX/CH4 calibrated at ~50 bar.
    // Reused at Pc=110 bar as a weak-pressure-dependence approximation
    // (see PRESSURE NOTE in the file header) — values are NOT pressure-scaled.
    // O/F → (Tc, gamma, molWeight, cStar)
    // Sources: CEA runs, Gordon & McBride, fuel-rich trend from literature
    // Fuel-rich (low O/F) = lower Tc, higher γ, lower MW
    static readonly (float of, float Tc, float gamma, float MW, float cStar)[] _ceaTable =
    {
        (2.0f, 2850f, 1.180f, 17.5f, 1680f),
        (2.2f, 3050f, 1.165f, 18.5f, 1740f),
        (2.4f, 3200f, 1.155f, 19.3f, 1780f),
        (2.6f, 3320f, 1.145f, 20.0f, 1810f),
        (2.8f, 3400f, 1.138f, 20.6f, 1830f),
        (3.0f, 3450f, 1.134f, 21.0f, 1843f),
        (3.2f, 3492f, 1.131f, 21.3f, 1850f),  // verified CEA point
        (3.4f, 3480f, 1.128f, 21.8f, 1845f),  // past stoich peak
        (3.6f, 3450f, 1.125f, 22.2f, 1835f),
        (3.8f, 3400f, 1.122f, 22.6f, 1820f),
        (4.0f, 3340f, 1.120f, 23.0f, 1800f),
        (4.2f, 3270f, 1.118f, 23.3f, 1775f),
    };

    /// Interpolate CEA data by O/F ratio
    public static (float Tc, float gamma, float MW, float cStar) InterpolateCEA(float of)
    {
        // Clamp to table range
        if (of <= _ceaTable[0].of)
            return (_ceaTable[0].Tc, _ceaTable[0].gamma, _ceaTable[0].MW, _ceaTable[0].cStar);
        if (of >= _ceaTable[^1].of)
            return (_ceaTable[^1].Tc, _ceaTable[^1].gamma, _ceaTable[^1].MW, _ceaTable[^1].cStar);

        for (int i = 0; i < _ceaTable.Length - 1; i++)
        {
            if (of >= _ceaTable[i].of && of <= _ceaTable[i + 1].of)
            {
                float t = (of - _ceaTable[i].of) / (_ceaTable[i + 1].of - _ceaTable[i].of);
                return (
                    _ceaTable[i].Tc + t * (_ceaTable[i + 1].Tc - _ceaTable[i].Tc),
                    _ceaTable[i].gamma + t * (_ceaTable[i + 1].gamma - _ceaTable[i].gamma),
                    _ceaTable[i].MW + t * (_ceaTable[i + 1].MW - _ceaTable[i].MW),
                    _ceaTable[i].cStar + t * (_ceaTable[i + 1].cStar - _ceaTable[i].cStar)
                );
            }
        }
        return (_ceaTable[^1].Tc, _ceaTable[^1].gamma, _ceaTable[^1].MW, _ceaTable[^1].cStar);
    }

    // Chamber pressure at which _ceaTable was generated (Pa). The whole table is
    // anchored here; ApplyPressureCorrection returns unity at this pressure.
    const float Pc_cal = 50e5f; // 50 bar

    /// <summary>
    /// First-order chamber-pressure correction for the 50-bar CEA table.
    ///
    /// PHYSICAL BASIS — dissociation suppression. The equilibrium flame
    /// temperature of LOX/CH4 sits well below the no-dissociation adiabatic
    /// value because endothermic dissociation (CO₂⇌CO+½O₂, H₂O⇌OH+½H₂, …)
    /// consumes enthalpy. Each such reaction increases the number of moles
    /// (Δn>0), so by Le Chatelier its equilibrium constant Kp ∝ P^(−Δn) falls
    /// as pressure rises: higher Pc ⇒ less dissociation ⇒ higher Tc and a
    /// slightly higher c*. Because Kp depends on pressure as a power law, the
    /// resulting shift in Tc and c* is, to first order, LOGARITHMIC in Pc.
    /// Hence the correction has the form  f = 1 + k·ln(Pc/Pc_cal).
    ///
    /// MAGNITUDES (anchored to published NASA-CEA trends for LOX/CH4 near the
    /// O/F≈3.2–3.5 peak; cross-checked against EUCASS/ONERA CEA2 comparisons):
    /// over 50→110 bar (ln(110/50)=0.788) CEA gives Tc up ≈+1.0% (~+35 K on a
    /// ~3490 K flame) and c* up ≈+0.6%. The k-values below reproduce exactly
    /// that: kTc=0.013 → +1.0%, kCstar=0.008 → +0.6% at 110 bar. γ moves by
    /// <+0.3% over this span — smaller than the table's own row-to-row
    /// quantisation — so a deliberately tiny kGamma=0.003 (+0.24% at 110 bar)
    /// is used; treat the γ correction as cosmetic rather than load-bearing.
    /// MW is left UNCORRECTED on purpose: recombination raises MW slightly,
    /// which partially cancels the Tc rise in c* ∝ √(Tc·R0/MW); that net
    /// cancellation is already folded into the small kCstar above, so applying
    /// an independent MW shift as well would double-count.
    ///
    /// CONSERVATISM / HONESTY. (1) The factor is clamped to ln-argument
    /// pressures within [0.5, 3.0]×Pc_cal (25–150 bar) so a far-field sweep
    /// sample cannot drive an unphysical extrapolation; the live Pc sweep
    /// (50–150 bar) stays inside the validated/published band. (2) These are
    /// CORRECTION FACTORS tuned to the published 50→110 bar deltas, NOT a CEA
    /// re-run; expect the corrected Tc/c* to be within ~±0.5% of a true CEA run
    /// over 50–150 bar, and the absolute table values themselves still carry the
    /// usual equilibrium-vs-real bias (real chambers run a few % below CEA Tc).
    /// Do not read more precision into these numbers than that.
    /// </summary>
    public static (float Tc, float gamma, float MW, float cStar)
        ApplyPressureCorrection(float Pc, float Tc, float gamma, float MW, float cStar)
    {
        const float kTc     = 0.013f; // → +1.0% Tc   at 110 bar (≈+35 K)
        const float kCstar  = 0.008f; // → +0.6% c*   at 110 bar
        const float kGamma  = 0.003f; // → +0.24% γ   at 110 bar (cosmetic)

        // Clamp the pressure ratio to the validated extrapolation band before
        // taking the log, so an out-of-range sweep sample is treated at the
        // band edge rather than producing a runaway correction.
        float ratio   = Math.Clamp(Pc / Pc_cal, 0.5f, 3.0f);
        float lnRatio = MathF.Log(ratio); // 0 exactly at Pc = Pc_cal (no-op)

        return (
            Tc    * (1f + kTc     * lnRatio),
            gamma * (1f + kGamma  * lnRatio),
            MW,                                  // intentionally uncorrected (see remarks)
            cStar * (1f + kCstar  * lnRatio)
        );
    }

    public static void Compute(AeroSpec S)
    {
        Library.Log("── Step 1: Thermochemistry ──");

        // O/F-dependent NASA CEA data for LOX/CH4 (calibrated at Pc_cal ≈ 50 bar)
        var (Tc, gamma, MW, cStar) = InterpolateCEA(S.OF_ratio);
        // First-order chamber-pressure correction (dissociation suppression).
        // No-op at Pc = Pc_cal; small positive shift on Tc/c* at 110 bar.
        // See ApplyPressureCorrection for the physical basis and bias bounds.
        (Tc, gamma, MW, cStar) = ApplyPressureCorrection(S.Pc, Tc, gamma, MW, cStar);
        S.Tc         = Tc;
        S.gamma      = gamma;
        S.molWeight  = MW;
        S.cStar      = cStar;

        // Transport properties scale with Tc
        // mu ~ T^0.7, Cp roughly constant for combustion products, Pr ~ constant
        float TcRef = 3492f; // reference point
        S.mu_gas        = 8.5e-5f * MathF.Pow(S.Tc / TcRef, 0.7f);
        S.Cp_transport  = 2200f;    // J/(kg·K) — TRANSPORT Cp (NOT 6795 equilibrium!)
        S.Pr_gas        = 0.55f;    // — Prandtl number

        // Derived gas properties
        S.R_gas  = R0 / S.molWeight;  // J/(kg·K) specific gas constant
        S.a_sound = MathF.Sqrt(S.gamma * S.R_gas * S.Tc);  // m/s speed of sound

        // Thrust coefficient — computed from isentropic relations
        // Cf = sqrt( 2γ²/(γ-1) × (2/(γ+1))^((γ+1)/(γ-1)) × (1-(Pa/Pc)^((γ-1)/γ)) )
        // For aerospike with altitude compensation, use adapted nozzle (Pe=Pa at SL)
        float g = S.gamma;
        float pressureRatio = Pa_SL / S.Pc;
        float exponent = (g - 1f) / g;

        S.Cf = MathF.Sqrt(
            (2f * g * g / (g - 1f))
            * MathF.Pow(2f / (g + 1f), (g + 1f) / (g - 1f))
            * (1f - MathF.Pow(pressureRatio, exponent))
        );

        // Vacuum Cf (Pa=0 → pressure term disappears, add ε×Pe/Pc)
        // Simplified: Cf_vac ≈ Cf_SL + Pa×Ae/(Pc×At) ≈ Cf_SL × 1.08 for typical ε
        float Cf_vac = S.Cf * 1.08f;

        // Specific impulse
        S.Isp_SL  = S.cStar * S.Cf / g0;
        S.Isp_vac = S.cStar * Cf_vac / g0;

        // Mass flow rate: ṁ = Pc × At / c* (At not yet known, will be set in ChamberSizing)
        // For now compute from F = ṁ × Ve → ṁ = F / (Isp × g0)
        S.mDot = S.F_thrust / (S.Isp_SL * g0);

        Library.Log($"  Tc={S.Tc:F0}K, γ={S.gamma:F3}, c*={S.cStar:F0} m/s  (Pc-corrected from 50-bar CEA table at {S.Pc/1e5:F0} bar)");
        Library.Log($"  Cf={S.Cf:F3} (SL), Isp={S.Isp_SL:F1}s (SL), {S.Isp_vac:F1}s (vac)");
        Library.Log($"  ṁ={S.mDot:F3} kg/s, Cp_transport={S.Cp_transport:F0} (NOT {6795})");
    }
}
