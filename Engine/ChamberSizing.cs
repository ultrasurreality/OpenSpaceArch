// ChamberSizing.cs — Step 2: Compute all geometric dimensions from physics
// Source: Knowledge/Physics_Formulas/Nozzle_Design.md
//
// Every dimension is derived from thrust, Pc, and thermochemistry.
// "Geometry is the last thing you output" — we only compute NUMBERS here, no voxels.

using PicoGK;

namespace OpenSpaceArch.Engine;

public static class ChamberSizing
{
    public static void Compute(AeroSpec S)
    {
        Library.Log("── Step 2: Chamber Sizing ──");

        // ── Throat area from thrust equation: F = Cf × Pc × At
        S.At = S.F_thrust / (S.Cf * S.Pc);                     // m²
        S.Dt = 2f * MathF.Sqrt(S.At / MathF.PI);               // m — equivalent diameter

        // ── Annular throat radii
        // For aerospike: At = π × (rShroud² - rSpike²)
        // throatGapRatio = (rShroud - rSpike) / rShroud
        // → rSpike = rShroud × (1 - gapRatio)
        // → At = π × rShroud² × (1 - (1-gapRatio)²)
        // → rShroud = sqrt(At / (π × (1 - (1-gapRatio)²)))
        float gapFactor = 1f - MathF.Pow(1f - S.throatGapRatio, 2f);
        float rShroud_m = MathF.Sqrt(S.At / (MathF.PI * gapFactor));
        float rSpike_m  = rShroud_m * (1f - S.throatGapRatio);

        S.rShroudThroat = rShroud_m * 1000f;  // mm
        S.rSpikeThroat  = rSpike_m * 1000f;    // mm

        // ── Chamber radii from contraction ratio: CR = Ac/At
        // For annular: Ac = π × (rShroudCh² - rSpikeCh²) = CR × At
        // Keep spike chamber radius slightly larger than throat (smooth expansion)
        // rSpikeCh = rSpikeThroat × 1.2 (gentle outward taper)
        S.rSpikeChamber = S.rSpikeThroat * 1.2f;
        // rShroudCh from: π(rShroudCh² - rSpikeCh²) = CR × At
        float AcNeeded = S.CR * S.At;  // m²
        float rSpikeCh_m = S.rSpikeChamber / 1000f;
        float rShroudCh_m = MathF.Sqrt(AcNeeded / MathF.PI + rSpikeCh_m * rSpikeCh_m);
        S.rShroudChamber = rShroudCh_m * 1000f;  // mm

        // ── Spike tip radius (minimum printable feature at 0.3mm voxel)
        S.rSpikeTip = MathF.Max(S.voxelSize * 3f, 1.5f);  // mm

        // ── Chamber length from L*
        // L* = V_chamber / At → V = L* × At
        // V = Ac × Lc (cylindrical section only, ignoring convergent volume)
        float Ac = AcNeeded;  // m²
        float Lc_m = S.Lstar * S.At / Ac;
        S.Lc = Lc_m * 1000f;  // mm

        // ── Convergent section (throat → chamber bottom)
        // Half-angle = convergentHalfAngle degrees
        // ΔR = rShroudChamber - rShroudThroat (radial distance to cover)
        float deltaR_shroud = S.rShroudChamber - S.rShroudThroat;
        float tanAngle = MathF.Tan(S.convergentHalfAngle * MathF.PI / 180f);
        S.convergentDz = deltaR_shroud / tanAngle;  // mm

        // ── Dome closure (injector end)
        // Dome angle = maxOverhang (45°) for LPBF self-supporting
        // Height = rSpikeChamber / tan(45°) = rSpikeChamber
        S.domeDz = S.rSpikeChamber;  // mm (45° cone to close spike)

        // ── Spike contour length (tip to throat)
        // Spike needs enough length for the convergent nozzle to work
        // Longer spike for visible aerospike shape (≈8× throat gap)
        float throatGap = S.rShroudThroat - S.rSpikeThroat;
        float spikeBelowThroat = throatGap * 8f;  // mm

        // ── Z-stations (all measured from tip = 0)
        S.zTip     = 0f;
        S.zThroat  = spikeBelowThroat;
        S.zCowl    = S.zThroat - throatGap * 1.5f;  // cowl starts slightly below throat
        S.zChBot   = S.zThroat + S.convergentDz;
        S.zChTop   = S.zChBot + S.Lc;
        S.zInjector = S.zChTop + S.domeDz;
        S.zTotal   = S.zInjector + 4f;  // small margin at top

        // ── Update mass flow (now that At is known, cross-check)
        float mDot_check = S.Pc * S.At / S.cStar;
        Library.Log($"  ṁ check: {S.mDot:F3} (from F/Ve) vs {mDot_check:F3} (from Pc×At/c*)");
        S.mDot = mDot_check;  // use the more fundamental formula

        // ── Log results
        Library.Log($"  At={S.At*1e6:F1} mm², Dt_eq={S.Dt*1000:F1} mm");
        Library.Log($"  Spike: tip={S.rSpikeTip:F1}, throat={S.rSpikeThroat:F1}, chamber={S.rSpikeChamber:F1} mm");
        Library.Log($"  Shroud: throat={S.rShroudThroat:F1}, chamber={S.rShroudChamber:F1} mm");
        Library.Log($"  Gap@throat: {throatGap:F1} mm");
        Library.Log($"  Lc={S.Lc:F1} mm, convergent={S.convergentDz:F1} mm, dome={S.domeDz:F1} mm");
        Library.Log($"  Z: tip={S.zTip:F1} cowl={S.zCowl:F1} throat={S.zThroat:F1} chBot={S.zChBot:F1} chTop={S.zChTop:F1} inj={S.zInjector:F1} total={S.zTotal:F1}");
    }

    // ── Profile functions: radius as function of z (mm → mm)
    // These are called by HeatTransfer and FluidVolumes

    /// Spike gas-side surface profile r(z)
    public static float SpikeProfile(AeroSpec S, float z)
    {
        if (z <= S.zTip) return S.rSpikeTip;

        // Tip to throat: convex S-curve (Angelino approximation)
        if (z <= S.zThroat)
        {
            float t = (z - S.zTip) / (S.zThroat - S.zTip);
            float s = 0.5f * (1f - MathF.Cos(MathF.PI * t * t));
            return S.rSpikeTip + (S.rSpikeThroat - S.rSpikeTip) * s;
        }

        // Throat to chamber bottom: smooth expansion
        if (z <= S.zChBot)
        {
            float t = (z - S.zThroat) / (S.zChBot - S.zThroat);
            float s = 0.5f * (1f - MathF.Cos(MathF.PI * t));
            return S.rSpikeThroat + (S.rSpikeChamber - S.rSpikeThroat) * s;
        }

        // Chamber: constant
        if (z <= S.zChTop) return S.rSpikeChamber;

        // Dome closure: linear taper to zero (injector)
        if (z <= S.zInjector)
        {
            float t = (z - S.zChTop) / (S.zInjector - S.zChTop);
            return S.rSpikeChamber * MathF.Max(0f, 1f - t);
        }

        return 0f;
    }

    /// Shroud inner (gas-side) surface profile r(z)
    public static float ShroudProfile(AeroSpec S, float z)
    {
        if (z < S.zCowl) return 0f;

        // Cowl to throat: sharp convergent
        if (z < S.zThroat)
        {
            float t = (z - S.zCowl) / (S.zThroat - S.zCowl);
            // Converge from wider entry to throat radius
            float cowlEntry = S.rShroudThroat + (S.rShroudChamber - S.rShroudThroat) * 0.4f;
            return cowlEntry + (S.rShroudThroat - cowlEntry) * t;
        }

        // Throat to chamber bottom: smooth expansion
        if (z <= S.zChBot)
        {
            float t = (z - S.zThroat) / (S.zChBot - S.zThroat);
            float s = 0.5f * (1f - MathF.Cos(MathF.PI * t));
            return S.rShroudThroat + (S.rShroudChamber - S.rShroudThroat) * s;
        }

        // Chamber: constant
        if (z <= S.zChTop) return S.rShroudChamber;

        // Shroud top taper (slight inward for dome)
        if (z <= S.zInjector)
        {
            float t = (z - S.zChTop) / (S.zInjector - S.zChTop);
            return S.rShroudChamber * (1f - 0.3f * t);
        }

        return S.rShroudChamber * 0.7f;
    }

    /// Annular area at z (for Bartz At/A calculation)
    public static float AnnularArea(AeroSpec S, float z)
    {
        float rS = SpikeProfile(S, z);
        float rSh = ShroudProfile(S, z);
        if (rSh <= rS) return 0f;
        return MathF.PI * (rSh * rSh - rS * rS);  // mm²
    }
}
