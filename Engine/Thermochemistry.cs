// Thermochemistry.cs — Step 1: Gas properties from NASA CEA
// Source: Knowledge/Physics_Formulas/NASA_CEA_Data.md (verified runs)
//
// CRITICAL: Cp_transport = 2200 J/(kg·K), NOT Cp_equilibrium = 6795
// Using wrong Cp → Bartz off by 3× → all channel sizing wrong!

using PicoGK;

namespace OpenSpaceArch.Engine;

public static class Thermochemistry
{
    const float g0 = 9.80665f;  // m/s² standard gravity
    const float R0 = 8314f;     // J/(kmol·K) universal gas constant
    const float Pa_SL = 101325f;// Pa atmospheric pressure at sea level

    // NASA CEA interpolation table: LOX/CH4 at ~50 bar
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

    public static void Compute(AeroSpec S)
    {
        Library.Log("── Step 1: Thermochemistry ──");

        // O/F-dependent NASA CEA data for LOX/CH4
        var (Tc, gamma, MW, cStar) = InterpolateCEA(S.OF_ratio);
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

        Library.Log($"  Tc={S.Tc:F0}K, γ={S.gamma:F3}, c*={S.cStar:F0} m/s");
        Library.Log($"  Cf={S.Cf:F3} (SL), Isp={S.Isp_SL:F1}s (SL), {S.Isp_vac:F1}s (vac)");
        Library.Log($"  ṁ={S.mDot:F3} kg/s, Cp_transport={S.Cp_transport:F0} (NOT {6795})");
    }
}
