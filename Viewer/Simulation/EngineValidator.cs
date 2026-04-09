// EngineValidator.cs — checks whether the generated engine geometry + physics
// can actually produce thrust. If any check fails, the IGNITE button is
// disabled and the reason is displayed.
//
// Phase 1 (2026-04-09): Extended with Slack + SlackNormalized fields. Each
// check computes how far the current state is from its violation boundary,
// normalized to [-1, +1] so the new ConstraintsPanel can render live slack
// bars reactive to slider changes. Added 4 new physics checks (thermal
// stress, coolant capacity, acoustic separation, residence time).
//
// Architecture context: this file IS the constraint library for the
// two-layer CSP architecture. Current checks validate a single (post-
// physics) AeroSpec state. The outer layer (Phase 2) will sample many
// specs and keep only the passing ones.
//
// See ~/LEAP71_Knowledge/Инсайты/Архитектурный синтез CSP.md and
// ~/LEAP71_Knowledge/Инсайты/Формализация constraints для aerospike.md

using OpenSpaceArch.Engine;

namespace OpenSpaceArch.Viewer.Simulation;

/// <summary>
/// One constraint check result.
/// <para>
/// <see cref="Slack"/>: signed distance from current state to the violation
/// boundary, in the check's native units. Positive = safe margin, zero = at
/// boundary, negative = violated.
/// </para>
/// <para>
/// <see cref="SlackNormalized"/>: same signal mapped to [-1, +1] via a
/// per-check reference scale, for UI bar rendering.
/// </para>
/// </summary>
public readonly record struct CheckResult(
    string Name,
    bool Passed,
    string Detail,
    float Slack,
    float SlackNormalized);

public readonly record struct Viability(bool IsViable, string Headline, IReadOnlyList<CheckResult> Checks);

public static class EngineValidator
{
    /// <summary>
    /// Helper to build a <see cref="CheckResult"/> with normalized slack.
    /// <paramref name="refScale"/> is the positive reference value that maps
    /// to SlackNormalized = 1.0 (fully satisfied). Must be > 0.
    /// </summary>
    private static CheckResult Make(string name, string detail, float slack, float refScale)
    {
        float norm = (refScale > 1e-9f)
            ? Math.Clamp(slack / refScale, -1f, 1f)
            : (slack >= 0f ? 1f : -1f);
        return new CheckResult(name, slack >= 0f, detail, slack, norm);
    }

    public static Viability Check(AeroSpec S)
    {
        var checks = new List<CheckResult>();

        // ══════════════════════════════════════════════════════════
        // PHYSICS SANITY
        // ══════════════════════════════════════════════════════════

        // C1. Thrust > 100 N
        {
            float slack = S.F_thrust - 100f;
            checks.Add(Make("Thrust > 100 N",
                $"{S.F_thrust / 1000f:F2} kN",
                slack, refScale: 4900f));
        }

        // C2. Mass flow > 1 g/s
        {
            float slack = S.mDot - 0.001f;
            checks.Add(Make("Mass flow > 1 g/s",
                $"{S.mDot * 1000f:F1} g/s",
                slack, refScale: 3f));
        }

        // C3. Isp > 100 s
        {
            float slack = S.Isp_SL - 100f;
            checks.Add(Make("Isp > 100 s",
                $"{S.Isp_SL:F0} s SL",
                slack, refScale: 200f));
        }

        // C4. Chamber T plausible (1500 .. 5000 K) — two-sided
        {
            float slack = Math.Min(S.Tc - 1500f, 5000f - S.Tc);
            checks.Add(Make("Chamber T plausible",
                $"{S.Tc:F0} K (need 1500..5000)",
                slack, refScale: 1500f));
        }

        // ══════════════════════════════════════════════════════════
        // THROAT GEOMETRY
        // ══════════════════════════════════════════════════════════

        // C5. Throat gap > 0.5 mm
        float throatGap = S.rShroudThroat - S.rSpikeThroat;
        {
            float slack = throatGap - 0.5f;
            checks.Add(Make("Throat gap > 0.5 mm",
                $"{throatGap:F2} mm (shroud {S.rShroudThroat:F2} - spike {S.rSpikeThroat:F2})",
                slack, refScale: 5f));
        }

        // C6. Throat area > 1 mm²
        {
            float slack = S.At - 1f;
            checks.Add(Make("Throat area > 1 mm^2",
                $"{S.At:F1} mm^2",
                slack, refScale: 100f));
        }

        // ══════════════════════════════════════════════════════════
        // WALL INTEGRITY
        // ══════════════════════════════════════════════════════════

        float wall = S.wallThroat;

        // C7. Wall >= min print
        {
            float slack = wall - S.minPrintWall;
            checks.Add(Make("Wall >= min print",
                $"{wall:F2} mm (min print {S.minPrintWall:F2} mm)",
                slack, refScale: 1.5f));
        }

        // C8. Burst safety > 1.2 (Barlow hoop stress — equivalent to predicate P8)
        float sigmaAllow = S.sigma_yield;    // Pa, from AeroSpec material
        float tBurst = S.Pc * S.rShroudThroat * 1e-3f / sigmaAllow * 1000f;  // mm required
        float burstSafety = (tBurst > 1e-6f) ? wall / tBurst : 999f;
        {
            float slack = burstSafety - 1.2f;
            checks.Add(Make("Burst safety > 1.2",
                $"ratio {burstSafety:F2} (need {tBurst * 1.2f:F2} mm, have {wall:F2} mm)",
                slack, refScale: 2f));
        }

        // ══════════════════════════════════════════════════════════
        // COOLING CHANNELS
        // ══════════════════════════════════════════════════════════

        // C9. Shroud channels >= 8
        {
            float slack = S.nChannelsShroud - 8f;
            checks.Add(Make("Shroud channels >= 8",
                $"{S.nChannelsShroud}",
                slack, refScale: 40f));
        }

        // C10. Spike channels >= 4
        {
            float slack = S.nChannelsSpike - 4f;
            checks.Add(Make("Spike channels >= 4",
                $"{S.nChannelsSpike}",
                slack, refScale: 20f));
        }

        // C11. Min channel radius >= 0.25 mm
        {
            float slack = S.chRadiusMin - 0.25f;
            checks.Add(Make("Min channel R >= 0.25 mm",
                $"min {S.chRadiusMin:F2} mm, max {S.chRadiusMax:F2} mm",
                slack, refScale: 1f));
        }

        // ══════════════════════════════════════════════════════════
        // VOXEL RESOLUTION
        // ══════════════════════════════════════════════════════════

        // C12. Voxel res < throat/2
        {
            float slack = throatGap * 0.5f - S.voxelSize;
            checks.Add(Make("Voxel res < throat/2",
                $"voxel {S.voxelSize:F2} mm vs throat/2 {throatGap * 0.5f:F2} mm",
                slack, refScale: 2f));
        }

        // ══════════════════════════════════════════════════════════
        // PHASE 1 NEW CHECKS — formalized in
        // Инсайты/Формализация constraints для aerospike.md
        // ══════════════════════════════════════════════════════════

        // C13. Thermal stress at throat (predicate P9 — thermal + Lame)
        // σ_th = E · α · ΔT / (1 - ν), where ΔT = q · t / k (Fourier drop across wall)
        // Must stay under σ_yield / SF.
        {
            float k = S.k_wall;       // W/(m·K)
            float t_m = wall * 1e-3f; // wall thickness in meters
            float dT = (k > 1e-6f) ? S.qThroat * t_m / k : 0f;
            float sigma_th = S.E_mod * S.alpha_CTE * dT / Math.Max(1e-6f, 1f - S.nu_poisson);
            float sigma_allow_th = S.sigma_yield / Math.Max(0.1f, S.SF);
            float slack = (sigma_allow_th - sigma_th) / 1e6f;   // MPa
            checks.Add(Make("Thermal stress < yield/SF",
                $"σ_th {sigma_th / 1e6f:F0} MPa vs allow {sigma_allow_th / 1e6f:F0} MPa (ΔT {dT:F0} K)",
                slack, refScale: 200f));
        }

        // C14. Coolant thermal capacity
        // Approx: ṁ_fuel · Cp · ΔT_allow ≥ q_throat · A_hot_throat
        // ΔT_allow = 600 K (CH4 decomposition ceiling minus inlet ~120 K).
        // A_hot_throat = 2π · r_shroud_throat · L_throat_region (≈ 2·Dt)
        {
            float rT = S.rShroudThroat * 1e-3f;    // m
            float Dt_m = 2f * rT;                   // throat diameter, m
            float A_hot = 2f * MathF.PI * rT * (2f * Dt_m);   // ~2Dt long region, m²
            float Q_throat = S.qThroat * A_hot;     // W

            float dT_allow = 600f;                  // K, conservative CH4 heating envelope
            float Q_capacity = S.mDot_fuel * S.Cp_coolant_shroud * dT_allow;   // W

            float slack = (Q_capacity - Q_throat) / 1e3f;   // kW
            checks.Add(Make("Coolant capacity > throat Q",
                $"capacity {Q_capacity / 1e3f:F1} kW vs throat {Q_throat / 1e3f:F1} kW",
                slack, refScale: 100f));
        }

        // C15. Acoustic 1L mode > 3 · combustion frequency (predicate S1)
        // f_1L ≈ a_sound / (2 · Lc)            (longitudinal half-wave)
        // f_comb ≈ c_star / L*                 (characteristic chamber turnover)
        {
            float Lc_m = S.Lc * 1e-3f;              // m
            float f_1L = (Lc_m > 1e-6f) ? S.a_sound / (2f * Lc_m) : 0f;
            float f_comb = (S.Lstar > 1e-6f) ? S.cStar / S.Lstar : 0f;
            float slack = f_1L - 3f * f_comb;
            checks.Add(Make("f_1L > 3 · f_comb",
                $"1L {f_1L:F0} Hz vs 3·comb {3f * f_comb:F0} Hz",
                slack, refScale: 3000f));
        }

        // C16. Combustion residence time (characteristic chamber)
        // τ_stay ≈ L* / c_star. Minimum ~1 ms for stable combustion
        // (Sutton RPE ch. 5, typical range 1-5 ms).
        {
            float tau = (S.cStar > 1e-6f) ? S.Lstar / S.cStar * 1e3f : 0f;  // ms
            float slack = tau - 1f;
            checks.Add(Make("Residence time > 1 ms",
                $"τ_stay {tau:F2} ms (L*={S.Lstar:F2} m, c*={S.cStar:F0} m/s)",
                slack, refScale: 4f));
        }

        // ══════════════════════════════════════════════════════════
        // AGGREGATE
        // ══════════════════════════════════════════════════════════

        bool allPassed = checks.TrueForAll(c => c.Passed);
        int failed = checks.FindAll(c => !c.Passed).Count;

        string headline = allPassed
            ? $"All {checks.Count} checks passed - engine ready"
            : $"{failed}/{checks.Count} checks failed - cannot ignite";

        return new Viability(allPassed, headline, checks);
    }
}
