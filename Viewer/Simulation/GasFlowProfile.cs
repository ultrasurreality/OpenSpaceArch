// GasFlowProfile.cs - quasi-1D isentropic flow through the aerospike nozzle.
//
// At each z-station along the axis we compute:
//   A(z) = pi * (rShroud^2 - rSpike^2)  annular area
//   M(z) from area-Mach relation (subsonic before throat, supersonic after)
//   T(z) = Tc / (1 + (gamma-1)/2 * M^2)
//   P(z) = Pc * (T(z)/Tc)^(gamma/(gamma-1))
//   rho(z) = rho_c * (T(z)/Tc)^(1/(gamma-1))
//   u(z) = M(z) * sqrt(gamma * R_gas * T(z))
//
// This feeds the particle system with real gas velocities in m/s.
// The throat is detected as the minimum of A(z). Reference A* = A_throat.
//
// Aerospike note: the "exit" of an aerospike is at the spike tip. We treat the
// region between throat and tip as the supersonic expansion, using the shroud
// as the outer streamline (slip boundary in vacuum). Past the tip, the gas
// continues with Ve from isentropic expansion at A_exit / A*.

using OpenSpaceArch.Engine;

namespace OpenSpaceArch.Viewer.Simulation;

public sealed class GasFlowProfile
{
    public int N { get; private set; }
    public float[] Z  = Array.Empty<float>();
    public float[] RSpike = Array.Empty<float>();
    public float[] RShroud = Array.Empty<float>();
    public float[] A = Array.Empty<float>();     // mm^2
    public float[] Mach = Array.Empty<float>();
    public float[] T_K = Array.Empty<float>();
    public float[] P_Pa = Array.Empty<float>();
    public float[] Rho = Array.Empty<float>();   // kg/m^3
    public float[] U_ms = Array.Empty<float>();  // m/s

    public float ZThroat { get; private set; }
    public int ThroatIdx { get; private set; }
    public float AThroat_mm2 { get; private set; }
    public float Gamma { get; private set; }
    public float RGas { get; private set; }
    public float Tc { get; private set; }
    public float Pc { get; private set; }

    public void Compute(AeroSpec S, int samples = 256)
    {
        N = samples;
        Z = new float[N];
        RSpike = new float[N];
        RShroud = new float[N];
        A = new float[N];
        Mach = new float[N];
        T_K = new float[N];
        P_Pa = new float[N];
        Rho = new float[N];
        U_ms = new float[N];

        Gamma = S.gamma;
        Tc = S.Tc;
        Pc = S.Pc;
        RGas = S.R_gas;

        // Gas path is bounded axially by spike tip on bottom and injector on top
        float zStart = S.zTip;
        float zEnd = S.zInjector;

        // Sample geometry
        float aMin = float.MaxValue;
        int iThroat = 0;
        for (int i = 0; i < N; i++)
        {
            float t = i / (float)(N - 1);
            float z = zStart + t * (zEnd - zStart);
            Z[i] = z;
            RSpike[i] = MathF.Max(0f, ChamberSizing.SpikeProfile(S, z));
            RShroud[i] = MathF.Max(0f, ChamberSizing.ShroudProfile(S, z));
            float a = MathF.PI * (RShroud[i] * RShroud[i] - RSpike[i] * RSpike[i]); // mm^2
            if (a <= 0f) a = 0.01f;
            A[i] = a;
            if (a < aMin)
            {
                aMin = a;
                iThroat = i;
            }
        }
        AThroat_mm2 = aMin;
        ThroatIdx = iThroat;
        ZThroat = Z[iThroat];

        // Chamber density from perfect gas law
        float rho_c = Pc / (RGas * Tc);

        // Compute Mach, T, P, rho, u
        for (int i = 0; i < N; i++)
        {
            float areaRatio = A[i] / aMin;
            bool supersonic = i > iThroat;
            float M = SolveMachFromAreaRatio(areaRatio, Gamma, supersonic);
            Mach[i] = M;

            float tempFactor = 1f + (Gamma - 1f) * 0.5f * M * M;
            T_K[i] = Tc / tempFactor;
            P_Pa[i] = Pc * MathF.Pow(tempFactor, -Gamma / (Gamma - 1f));
            Rho[i] = rho_c * MathF.Pow(tempFactor, -1f / (Gamma - 1f));
            U_ms[i] = M * MathF.Sqrt(Gamma * RGas * T_K[i]);
        }
    }

    /// <summary>
    /// Solves the isentropic area-Mach relation:
    ///   A/A* = (1/M) * [(2/(gamma+1)) * (1 + (gamma-1)/2 * M^2)]^((gamma+1)/(2(gamma-1)))
    /// for either subsonic or supersonic branch via bisection.
    /// </summary>
    private static float SolveMachFromAreaRatio(float areaRatio, float gamma, bool supersonic)
    {
        if (areaRatio <= 1.0001f) return 1f;

        float lo = supersonic ? 1.0001f : 0.01f;
        float hi = supersonic ? 10f : 0.9999f;

        for (int it = 0; it < 60; it++)
        {
            float mid = 0.5f * (lo + hi);
            float r = AreaRatio(mid, gamma);
            // The subsonic branch is decreasing in M, supersonic is increasing
            if (supersonic)
            {
                if (r < areaRatio) lo = mid; else hi = mid;
            }
            else
            {
                if (r > areaRatio) lo = mid; else hi = mid;
            }
        }
        return 0.5f * (lo + hi);
    }

    private static float AreaRatio(float M, float gamma)
    {
        float g1 = gamma + 1f;
        float gm1 = gamma - 1f;
        float inner = (2f / g1) * (1f + gm1 * 0.5f * M * M);
        float pow = MathF.Pow(inner, g1 / (2f * gm1));
        return pow / M;
    }

    /// <summary>
    /// Linear interpolation of gas velocity [m/s] at z. Returns 0 outside the nozzle.
    /// </summary>
    public float VelocityAt(float z)
    {
        return Sample(z, U_ms);
    }

    public float MachAt(float z) => Sample(z, Mach);
    public float TempAt(float z) => Sample(z, T_K);
    public float PressAt(float z) => Sample(z, P_Pa);

    private float Sample(float z, float[] data)
    {
        if (N == 0) return 0f;
        if (z <= Z[0]) return data[0];
        if (z >= Z[N - 1]) return data[N - 1];
        float step = Z[1] - Z[0];
        float t = (z - Z[0]) / step;
        int i = (int)t;
        if (i >= N - 1) return data[N - 1];
        float f = t - i;
        return data[i] + f * (data[i + 1] - data[i]);
    }
}
