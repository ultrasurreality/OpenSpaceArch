"""pymoo NSGA-III sweep for OpenSpaceArch.

================================================================================
  !!!  NON-AUTHORITATIVE PHYSICS  -  EXPLORATION AID ONLY  !!!
================================================================================
This script is a FAST, STANDALONE design-space explorer. It does NOT call the
C# engine. It is a hand-maintained numpy PORT of the authoritative C# silent
physics (Engine/DesignSweep.ComputeSilent → the same path OuterSweep uses) so
NSGA-III can evaluate ~100k candidates per second. The Pareto front it returns
is a MAP OF THE TERRAIN, not a validated engine result.

ALWAYS treat the C# pipeline (Thermochemistry -> ChamberSizing -> HeatTransfer
-> EngineValidator -> OuterSweep) as the single source of truth. After picking a
promising region here, re-run the in-app "Quick" sweep (real C# physics) and
"Apply Best" to get authoritative numbers.

────────────────────────────────────────────────────────────────────────────────
RECONCILIATION STATUS (2026-06-03, [pymoo-unify]):
  The numpy physics below was reconciled CONSTANT-FOR-CONSTANT and
  FORMULA-FOR-FORMULA against Engine/DesignSweep.ComputeSilent — the exact code
  the C# OuterSweep evaluates. Every reconciled value carries a `# C#:` comment
  pointing at its source field/line so a future edit to the C# side is easy to
  mirror here. Specifically NOW ALIGNED (previously diverged):
    * mu_gas scales with Tc:  8.5e-5·(Tc/3492)^0.7      (was a fixed 8.5e-5)
    * Bartz uses the FULL ComputeSilent form: explicit Cp_transport=2200, the
      Pr^0.6 divisor, AND the throat-curvature term (Dt/Rc)^0.1, Rc=0.382·Dt/2
      (the curvature term and explicit Cp were previously dropped).
    * recovery factor Pr^0.33·Tc and wall gas-side temp T_wg=T_max_service=800K
      replace the hardcoded "0.82·Tc − 800".
    * FILM COOLING is now applied exactly as C# FilmReduction:
      1 − filmFrac·filmEff·(T_aw−300)/(T_aw−T_wg), clamped [0.3, 1].
    * Annular throat/chamber sizing from throatGapRatio=0.2 and CR (replaces the
      fixed 0.36 annulus-area-ratio guess); wall thickness uses max(rSh,rSp) and
      σ_yield=250e6; thermal stress uses E=120e9, α=17e-6, ν=0.33, k=320.
    * mass = π·(rShroudChamber+3)²·zTotal·0.35·ρ  (the C# EngineEvaluation.
      MassEstimateKg formula) replaces the old bounding-cylinder guess.
    * g0 = 9.80665 (was 9.81).

  WHAT IS STILL, BY DESIGN, NOT THE C# RESULT (the loud, intentional caveats):
    * THIS IS A PORT, NOT A CALL. It can silently drift if the C# physics is
      edited and this file is not. The `# C#:` anchor comments and the
      _ASSUMED_CONSTANTS self-check below exist to make drift loud, but only the
      in-app C# "Quick" sweep is authoritative.
    * THRUST is pinned F = 5000 N. This MATCHES OuterSweep.Config.FixedThrust
      (the C# outer sweep also pins thrust at the 5 kN mission spec), so it is
      not a divergence from the outer sweep — but it does mean this screen is
      only valid for the 5 kN reference engine.
    * NO Pc-correction (ApplyPressureCorrection) is applied — deliberately,
      because ComputeSilent ALSO skips it (only the full Thermochemistry.Compute
      path used by the viewer/headless build applies it). So at the OuterSweep
      level this is a faithful match, but both diverge ≲1% from the
      pressure-corrected viewer numbers above 50 bar.
    * THE CHANNEL-FIT SELF-ITERATION IS NOT REPRODUCED. ComputeSilent runs two
      coupled loops this _evaluate omits entirely: (a) a 5-iteration channel-count
      solve for N (shroud channels) and r_ch from m_dot_fuel/(ρ·v_cool_max), and
      (b) an up-to-10-attempt fit loop that walks z-stations zCowl→zInjector and,
      whenever N channels don't fit the local circumference, ramps
      v_cool_max·=1.15 / v_cool_min·=1.15 and re-solves N. The numpy port has NO
      velocity ramp and NO N solve at all. This is SAFE for the Pareto objectives
      and constraints because they are independent of that loop: the loop only
      mutates channel geometry (S.nChannelsShroud, chRadiusMin/Max, v_cool_max/min),
      whereas the three objectives (Isp, mass, σ_thermal) and the two constraints
      (wall-thickness band) are driven solely by thermochemistry, chamber sizing,
      the throat-anchored Bartz qThroat, and wallThroat — none of which read back
      from N or v_cool. So the front this screen maps is faithful; it simply does
      not carry the channel-count / coolant-velocity columns ComputeSilent ends on.
      (Also absent, as a corollary: any per-z σ(M) channel inversion.)
    * Voxelized solid mass, SpatialValidator conflicts, and the full 16-predicate
      EngineValidator are NOT run here; validity here is the 2 hard pymoo
      constraints (wall thickness band) only.

CONTRACT (do not break - SweepPanel.cs / LoadParetoJson parses this):
  CLI:   python PymooSweep.py <n_gen> <pop_size> <output.json>
  Stdout (no out_path) OR <output.json> file content is:
    { "n_eval": int, "elapsed_s": float, "pareto_size": int,
      "solutions": [ { "Isp","mass","stress_MPa",
                       "Pc","OF","CR","Lstar","SF","Twist" }, ... ] }
  Progress/banner -> stderr.
================================================================================
"""
import sys, json, time
import numpy as np
from pymoo.core.problem import Problem
from pymoo.algorithms.moo.nsga3 import NSGA3
from pymoo.util.ref_dirs import get_reference_directions
from pymoo.optimize import minimize

# ── Shared physical constants, reconciled to the C# source of truth ───────────
# Each entry mirrors a field in Engine/AeroSpec.cs or a literal in
# Engine/DesignSweep.ComputeSilent / Engine/HeatTransfer. If you change the C#
# side, change it here too — the banner self-check prints these so a human
# watching stderr can eyeball-compare against the C# log.
_G0          = 9.80665    # C#: Thermochemistry.g0  (was 9.81)
_R0          = 8314.0     # C#: Thermochemistry.R0  (unused downstream, kept for parity)
_PA_SL       = 101325.0   # C#: Thermochemistry.Pa_SL
_F_THRUST    = 5000.0     # C#: OuterSweep.Config.FixedThrust (5 kN, pinned — see banner)

_TC_REF      = 3492.0     # C#: ComputeSilent TcRef for mu_gas scaling
_MU_REF      = 8.5e-5     # C#: AeroSpec.mu_gas base @ TcRef
_CP_TRANSPORT= 2200.0     # C#: AeroSpec.Cp_transport (TRANSPORT, NOT 6795 equilibrium!)
_PR_GAS      = 0.55       # C#: AeroSpec.Pr_gas

_THROAT_GAP_RATIO = 0.20  # C#: AeroSpec.throatGapRatio
_CONV_HALF_ANGLE  = 42.0  # C#: AeroSpec.convergentHalfAngle (deg)

_SIGMA_YIELD = 250e6      # C#: AeroSpec.sigma_yield (Pa)
_MIN_PRINT_WALL = 0.5     # C#: AeroSpec.minPrintWall (mm)
_K_WALL      = 320.0      # C#: AeroSpec.k_wall  W/(m·K)
_E_MOD       = 120e9      # C#: AeroSpec.E_mod   Pa
_ALPHA_CTE   = 17e-6      # C#: AeroSpec.alpha_CTE 1/K
_NU_POISSON  = 0.33       # C#: AeroSpec.nu_poisson
_RHO_METAL   = 8900.0     # C#: AeroSpec.rho (CuCrZr) kg/m³
_T_MAX_SERVICE = 800.0    # C#: AeroSpec.T_max_service (T_wg) K

_RHO_COOL_SHROUD = 200.0  # C#: AeroSpec.rho_coolant_shroud (supercritical CH4)
_V_COOL_MAX  = 10.0       # C#: AeroSpec.v_cool_max (m/s)
_V_COOL_MIN  = 3.0        # C#: AeroSpec.v_cool_min (m/s)
_MIN_CHANNEL = 1.0        # C#: AeroSpec.minChannel (mm)
_FILM_FRAC   = 0.08       # C#: AeroSpec.filmCoolFraction
_FILM_EFF    = 0.40       # C#: AeroSpec.filmCoolEffectiveness
_MASS_FILL   = 0.35       # C#: EngineEvaluation.MassEstimateKg fillFactor

# Surfaced in the stderr banner so a human can compare against the C# log values
# at a glance and catch silent drift between this port and the C# constants.
_ASSUMED_CONSTANTS = (
    f"g0={_G0} Cp_t={_CP_TRANSPORT:.0f} Pr={_PR_GAS} σ_yield={_SIGMA_YIELD/1e6:.0f}MPa "
    f"k={_K_WALL:.0f} E={_E_MOD/1e9:.0f}GPa α={_ALPHA_CTE*1e6:.0f}e-6 ν={_NU_POISSON} "
    f"gap={_THROAT_GAP_RATIO} F={_F_THRUST/1000:.0f}kN film={_FILM_FRAC}×{_FILM_EFF}"
)

# ── NASA CEA interpolation table: LOX/CH4 at ~50 bar ──────────────────────────
# Copied VERBATIM from Engine/Thermochemistry.cs (_ceaTable).
# Columns: O/F, Tc [K], gamma [-], MW [g/mol], cStar [m/s].
# Keep in sync with the C# table if it ever changes.
_CEA_TABLE = np.array([
    (2.0, 2850.0, 1.180, 17.5, 1680.0),
    (2.2, 3050.0, 1.165, 18.5, 1740.0),
    (2.4, 3200.0, 1.155, 19.3, 1780.0),
    (2.6, 3320.0, 1.145, 20.0, 1810.0),
    (2.8, 3400.0, 1.138, 20.6, 1830.0),
    (3.0, 3450.0, 1.134, 21.0, 1843.0),
    (3.2, 3492.0, 1.131, 21.3, 1850.0),  # verified CEA point
    (3.4, 3480.0, 1.128, 21.8, 1845.0),  # past stoich peak
    (3.6, 3450.0, 1.125, 22.2, 1835.0),
    (3.8, 3400.0, 1.122, 22.6, 1820.0),
    (4.0, 3340.0, 1.120, 23.0, 1800.0),
    (4.2, 3270.0, 1.118, 23.3, 1775.0),
])
_CEA_OF    = _CEA_TABLE[:, 0]
_CEA_TC    = _CEA_TABLE[:, 1]
_CEA_GAMMA = _CEA_TABLE[:, 2]
_CEA_MW    = _CEA_TABLE[:, 3]
_CEA_CSTAR = _CEA_TABLE[:, 4]


def interpolate_cea(of):
    """Vectorized linear interpolation of CEA data by O/F.
    np.interp clamps to the table end-points, matching the C# InterpolateCEA
    clamp behavior in Engine/Thermochemistry.cs.

    NOTE: like Engine/DesignSweep.ComputeSilent, NO ApplyPressureCorrection is
    applied — the raw 50-bar table is used. (Only the viewer/headless path via
    Thermochemistry.Compute applies the Pc correction; the OuterSweep this screen
    mirrors does not.)"""
    Tc    = np.interp(of, _CEA_OF, _CEA_TC)
    gamma = np.interp(of, _CEA_OF, _CEA_GAMMA)
    MW    = np.interp(of, _CEA_OF, _CEA_MW)
    cStar = np.interp(of, _CEA_OF, _CEA_CSTAR)
    return Tc, gamma, MW, cStar


class RocketProblem(Problem):
    def __init__(self):
        super().__init__(
            n_var=6, n_obj=3, n_constr=2,
            xl=np.array([50e5, 2.5, 3.0, 0.3, 1.3, 1.0]),
            xu=np.array([150e5, 4.0, 6.0, 1.2, 2.0, 4.0]),
        )

    def _evaluate(self, X, out, *args, **kwargs):
        # Vectorized PORT of Engine/DesignSweep.ComputeSilent (the exact physics
        # the C# OuterSweep runs). Variable names and the order of operations
        # follow ComputeSilent so the two are line-comparable.
        Pc = X[:, 0]; OF = X[:, 1]; CR = X[:, 2]
        Lstar = X[:, 3]; SF = X[:, 4]; Twist = X[:, 5]   # Twist unused in physics, carried through

        F  = _F_THRUST                       # C#: pinned thrust (see banner)
        g0 = _G0

        # ── Thermochemistry (ComputeSilent) ──────────────────────────────────
        Tc, gamma, MW, cStar = interpolate_cea(OF)        # C#: InterpolateCEA, raw table
        mu_gas = _MU_REF * (Tc / _TC_REF) ** 0.7          # C#: mu_gas = 8.5e-5·(Tc/3492)^0.7
        g  = gamma
        pressureRatio = _PA_SL / Pc
        exponent = (g - 1.0) / g
        Cf = np.sqrt(
            (2.0 * g * g / (g - 1.0))
            * (2.0 / (g + 1.0)) ** ((g + 1.0) / (g - 1.0))
            * (1.0 - pressureRatio ** exponent)
        )                                                  # C#: isentropic Cf (SL)
        Isp = cStar * Cf / g0                              # C#: Isp_SL

        # ── Chamber sizing (ComputeSilent) ───────────────────────────────────
        At = F / (Cf * Pc)                                 # C#: At = F/(Cf·Pc)  [m²]
        Dt = 2.0 * np.sqrt(At / np.pi)                     # C#: equivalent throat diameter [m]

        # Annular throat radii from throatGapRatio (replaces old 0.36 guess).
        gapFactor  = 1.0 - (1.0 - _THROAT_GAP_RATIO) ** 2  # C#: gapFactor
        rShroud_m  = np.sqrt(At / (np.pi * gapFactor))     # C#: rShroud_m
        rSpike_m   = rShroud_m * (1.0 - _THROAT_GAP_RATIO) # C#: rSpike_m
        rShroudThroat = rShroud_m * 1000.0                 # mm
        rSpikeThroat  = rSpike_m  * 1000.0                 # mm

        rSpikeChamber = rSpikeThroat * 1.2                 # C#: gentle outward taper
        AcNeeded      = CR * At                            # C#: Ac = CR·At  [m²]
        rSpikeCh_m    = rSpikeChamber / 1000.0
        rShroudChamber = np.sqrt(AcNeeded / np.pi + rSpikeCh_m ** 2) * 1000.0  # C#: rShroudCh [mm]

        # Lengths / z-stations (only what mass + sizing need).
        Lc = Lstar * At / AcNeeded * 1000.0                # C#: Lc = L*·At/Ac  [mm]
        deltaR_shroud = rShroudChamber - rShroudThroat
        tanAngle  = np.tan(np.radians(_CONV_HALF_ANGLE))
        convergentDz = deltaR_shroud / tanAngle            # C#: convergentDz
        domeDz    = rSpikeChamber                          # C#: 45° cone closure
        throatGap = rShroudThroat - rSpikeThroat
        spikeBelowThroat = throatGap * 8.0                 # C#: spike length below throat
        zThroat   = spikeBelowThroat
        zChBot    = zThroat + convergentDz
        zChTop    = zChBot + Lc
        zInjector = zChTop + domeDz
        zTotal    = zInjector + 4.0                        # C#: zTotal (+4 mm margin)

        # ── Heat transfer (ComputeSilent / HeatTransfer.BaseBartz) ───────────
        Rc = 0.382 * Dt / 2.0                              # C#: throat radius of curvature
        recoveryFactor = _PR_GAS ** 0.33                   # C#: ≈0.82
        T_aw = recoveryFactor * Tc                         # C#: adiabatic wall temp
        T_wg = _T_MAX_SERVICE                              # C#: design to material limit (800 K)

        # FULL Bartz base coefficient — note the explicit Cp_transport, the
        # Pr^0.6 divisor, AND the throat-curvature (Dt/Rc)^0.1 term that the old
        # reduced form dropped.
        baseBartz = (
            0.026 / Dt ** 0.2
            * mu_gas ** 0.2 * _CP_TRANSPORT / _PR_GAS ** 0.6
            * (Pc / cStar) ** 0.8
            * (Dt / Rc) ** 0.1
        )                                                  # C#: BaseBartz(S)
        hg_throat = baseBartz                              # C#: At/A=1 at throat → (·)^0.9 = 1
        q_raw = hg_throat * (T_aw - T_wg)

        # Film cooling — exact C# FilmReduction (was entirely absent before).
        filmReduction = 1.0 - _FILM_FRAC * _FILM_EFF * (T_aw - 300.0) / (T_aw - T_wg)
        filmReduction = np.clip(filmReduction, 0.3, 1.0)
        qThroat = q_raw * filmReduction                    # C#: S.qThroat  [W/m²]

        # Wall thickness at throat — pressure hoop on the larger radius, σ_yield.
        rLocal_throat = np.maximum(rShroudThroat, rSpikeThroat) / 1000.0   # m
        t_pressure = Pc * rLocal_throat / (_SIGMA_YIELD / SF) * 1000.0     # mm
        wallThroat = np.maximum(t_pressure, _MIN_PRINT_WALL)               # C#: S.wallThroat

        # Thermal stress at throat (EngineEvaluation.ThermalStressMPa).
        deltaT   = qThroat * (wallThroat / 1000.0) / _K_WALL              # C#: ΔT
        sigma_th = _E_MOD * _ALPHA_CTE * deltaT / (1.0 - _NU_POISSON)     # Pa
        sigma_th_MPa = sigma_th / 1e6

        # ── Mass estimate (EngineEvaluation.MassEstimateKg) ──────────────────
        rOuter   = rShroudChamber + 3.0                    # mm
        vol_mm3  = np.pi * rOuter * rOuter * zTotal
        mass     = vol_mm3 * 1e-9 * _RHO_METAL * _MASS_FILL  # C#: 35% fill factor

        out["F"] = np.column_stack([-Isp, mass, sigma_th_MPa])
        # Same wall-thickness validity band the pymoo screen always used: ≥0.5 mm
        # printable (matches _MIN_PRINT_WALL) and ≤3 mm to keep mass sane.
        out["G"] = np.column_stack([wallThroat - 3.0, _MIN_PRINT_WALL - wallThroat])


n_gen = int(sys.argv[1]) if len(sys.argv) > 1 else 200
pop_size = int(sys.argv[2]) if len(sys.argv) > 2 else 500
out_path = sys.argv[3] if len(sys.argv) > 3 else None

# Loud runtime banner so a human watching stderr cannot miss the disclaimer.
print("=" * 70, file=sys.stderr)
print("  pymoo NSGA-III sweep  —  NON-AUTHORITATIVE physics (numpy PORT of C#)", file=sys.stderr)
print("  Reconciled to DesignSweep.ComputeSilent: full Bartz (Cp=2200, Pr^0.6,", file=sys.stderr)
print("  throat-curvature), film cooling, annular sizing, mass = C# formula.", file=sys.stderr)
print(f"  Constants: {_ASSUMED_CONSTANTS}", file=sys.stderr)
print("  STILL a PORT not a CALL → can drift; F=5kN pinned; no Pc-correction,", file=sys.stderr)
print("  no voxel mass, no SpatialValidator. Validate winners in the C# 'Quick'", file=sys.stderr)
print("  sweep + Apply Best for authoritative values.", file=sys.stderr)
print("=" * 70, file=sys.stderr)

t0 = time.time()
ref_dirs = get_reference_directions("das-dennis", 3, n_partitions=12)
algo = NSGA3(pop_size=pop_size, ref_dirs=ref_dirs)
res = minimize(RocketProblem(), algo, ("n_gen", n_gen), seed=42, verbose=False)
elapsed = time.time() - t0

results = []
for i in range(len(res.F)):
    Pc,OF,CR,Lstar,SF,Twist = res.X[i]
    results.append(dict(
        Isp=float(-res.F[i,0]), mass=float(res.F[i,1]), stress_MPa=float(res.F[i,2]),
        Pc=float(Pc), OF=float(OF), CR=float(CR),
        Lstar=float(Lstar), SF=float(SF), Twist=float(Twist)))
results.sort(key=lambda r: -r["Isp"])

output = json.dumps(dict(
    # "simplified" flag is additive metadata; SweepPanel ignores unknown keys.
    # Kept True: this is a numpy PORT, not a call into the C# engine, so it is
    # still non-authoritative even though the constants/formulas are reconciled.
    simplified=True,
    physics_note="non-authoritative: numpy PORT of C# ComputeSilent (reconciled "
                 "constants/Bartz/film/mass), F=5kN pinned, no Pc-correction; "
                 "validate in C#",
    n_eval=int(res.algorithm.evaluator.n_eval),
    elapsed_s=round(elapsed, 2),
    pareto_size=len(results),
    solutions=results
), indent=2)

if out_path:
    with open(out_path, "w") as f: f.write(output)
    print(f"pymoo: {len(results)} Pareto solutions in {elapsed:.1f}s -> {out_path} "
          f"[NON-AUTHORITATIVE — numpy port of C#]", file=sys.stderr)
else:
    print(output)
