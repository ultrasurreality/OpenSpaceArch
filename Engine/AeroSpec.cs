// AeroSpec.cs — Single source of truth for all engine parameters
// "Geometry is the last thing you output" — every field here is computed from physics
//
// INPUTS: F_thrust, Pc, propellant pair, material
// COMPUTED: everything else (thermochem → sizing → heat transfer → channels)

using PicoGK;
using Leap71.ShapeKernel;

namespace OpenSpaceArch.Engine;

public enum ChannelMode { MeshBased_v4, Implicit_v5, Routed_v5b }

public class AeroSpec
{
    // Channel generation mode
    public ChannelMode channelMode = ChannelMode.Routed_v5b;

    // ============================================================
    // INPUTS (the ONLY manually specified values)
    // ============================================================

    // Performance requirements
    public float F_thrust   = 5000f;        // N — target thrust
    public float Pc         = 110e5f;       // Pa — chamber pressure (110 bar) — sweep v2 Pareto

    // Propellant selection
    public float OF_ratio   = 3.2f;         // O/F ratio — sweep v2 optimal (stoich peak Isp)

    // Design parameters (physics-informed choices)
    public float Lstar      = 0.4f;         // m — shorter chamber for visible aerospike proportions
    public float CR         = 4.0f;         // contraction ratio — sweep optimal
    public float throatGapRatio = 0.2f;     // (rShroud-rSpike)/rShroud at throat — smaller gap = fatter spike
    public float convergentHalfAngle = 42f; // degrees (max ~45° for stability)
    public float channelTwistTurns = 2.0f;  // full helical turns over channel length
    public float SF = 1.5f;                 // safety factor — sweep optimal (thinner wall = less thermal stress)

    // Symmetry-breaking parameters (biological look — breaks 2π/N fold in Routed_v5b)
    // Routing uses analytical helical paths on revolution surface — no turtle walk artifacts.
    // L3 (θ-mod) applied as spatial function of voxel position, not per-channel — keeps SDF continuous.
    public int   channelSeed                   = 42;    // master seed — same seed → identical STL, reproducibility
    public float azimuthalJitterFraction       = 0.35f; // how far phi_i can drift from its nominal position, in units of 2π/N
    public float twistJitterFraction           = 0.25f; // per-channel twistRate_i = global · (1 ± jitter)
    public float perChannelWidthJitterFraction = 0f;    // kept for future (smooth-blending spine version); disabled because discontinuous
    public float heatFluxAngularAmplitude      = 0.20f; // θ-modulation amplitude on halfH (hot zone around fuel inlet port, spatial)
    public int   heatFluxAngularHarmonic       = 2;     // number of "lobes" for θ-modulation; 2 = bipolar (fuel 0° vs LOX 180°)
    public float phaseDistributionExponent     = 1f;    // 1.0 = full golden ratio phyllotaxis, 0.0 = uniform + jitter

    // Manufacturing constraints (LPBF)
    public float voxelSize   = 0.4f;        // mm — voxel resolution
    public float minPrintWall = 0.5f;       // mm — LPBF minimum wall
    public float minChannel   = 1.0f;       // mm — minimum channel diameter for powder removal
    public float minRibWall  = 1.2f;       // mm — min wall between channels (≥3 voxels at 0.4mm)
    public float maxOverhang  = 45f;        // degrees — LPBF self-supporting angle

    // Turbulator ribs (inside channels, increase heat transfer 2-3×)
    public float ribPitch  = 2.5f;         // mm — rib spacing along channel
    public float ribHeight = 0.4f;         // mm — rib protrusion (reduces channel height)

    // Manifolds, ports & structural
    public float manifoldRadius   = 3.0f;  // mm — collector/inlet manifold cross-section
    public float feedPortRadius   = 1.5f;  // mm — external feed line bore
    public float feedPortLength   = 6.0f;  // mm — how far ports extend outward
    public int   nInjectorHoles   = 8;     // fuel injection holes around LOX tube
    public float mountFlangeExtent = 12f;  // mm — flange radial extent beyond shroud

    // ============================================================
    // MATERIAL PROPERTIES — CuCrZr at operating temperature
    // ============================================================

    public float k_wall      = 320f;        // W/(m·K) — thermal conductivity
    public float sigma_yield = 250e6f;      // Pa — yield strength at 600K
    public float E_mod       = 120e9f;      // Pa — Young's modulus
    public float alpha_CTE   = 17e-6f;      // 1/K — coefficient of thermal expansion
    public float nu_poisson  = 0.33f;       // Poisson's ratio
    public float rho         = 8900f;       // kg/m³ — density
    public float T_melt      = 1350f;       // K — melting point
    public float T_max_service = 800f;      // K — max continuous service temperature

    // ============================================================
    // COOLANT PROPERTIES — CH4 shroud, LOX spike
    // ============================================================

    public float rho_coolant_shroud = 200f;    // kg/m³ — supercritical CH4 avg
    public float Cp_coolant_shroud  = 3500f;   // J/(kg·K)
    public float rho_coolant_spike  = 1000f;   // kg/m³ — LOX
    public float Cp_coolant_spike   = 1700f;   // J/(kg·K)
    public float v_cool_max  = 10f;            // m/s — max coolant velocity (throat)
    public float v_cool_min  = 3f;             // m/s — min coolant velocity (chamber)
    public float spikeCoolFraction = 0.3f;     // fraction of LOX for spike cooling
    public float filmCoolFraction = 0.08f;     // fraction of fuel for film cooling (reduces q on wall)
    public float filmCoolEffectiveness = 0.4f; // η: how effectively film reduces heat flux (0=none, 1=perfect)

    // ============================================================
    // COMPUTED — Step 1: Thermochemistry (from CEA lookup)
    // ============================================================

    public float Tc;            // K — combustion temperature
    public float gamma;         // — specific heat ratio of products
    public float molWeight;     // kg/kmol — molecular weight of exhaust
    public float cStar;         // m/s — characteristic velocity
    public float Cf;            // — thrust coefficient (computed from gamma + Pc/Pa)
    public float Isp_SL;        // s — specific impulse at sea level
    public float Isp_vac;       // s — specific impulse in vacuum

    // Transport properties (for Bartz — CRITICAL: use transport Cp, NOT equilibrium)
    public float mu_gas;        // Pa·s — dynamic viscosity
    public float Cp_transport;  // J/(kg·K) — specific heat (TRANSPORT, ~2200, NOT 6795 equilibrium!)
    public float Pr_gas;        // — Prandtl number
    public float R_gas;         // J/(kg·K) — specific gas constant
    public float a_sound;       // m/s — speed of sound in chamber

    // ============================================================
    // COMPUTED — Step 2: Chamber Sizing
    // ============================================================

    public float At;            // m² — throat area
    public float Dt;            // m — throat diameter (equivalent)
    public float mDot;          // kg/s — total mass flow rate

    // Radii (mm) — computed from At and ratios
    public float rSpikeThroat;  // mm — spike radius at throat
    public float rShroudThroat; // mm — shroud inner radius at throat
    public float rSpikeChamber; // mm — spike radius in chamber
    public float rShroudChamber;// mm — shroud inner radius in chamber
    public float rSpikeTip;     // mm — spike tip radius (min printable)

    // Lengths (mm) — computed from L*, CR, angles
    public float Lc;            // mm — chamber cylindrical length
    public float convergentDz;  // mm — convergent section axial length
    public float domeDz;        // mm — dome closure height (45° LPBF constraint)

    // Z-stations (mm) — computed from lengths
    public float zTip;          // spike tip (z = 0)
    public float zCowl;         // cowl lip (shroud start)
    public float zThroat;       // throat location
    public float zChBot;        // chamber bottom (convergent end)
    public float zChTop;        // chamber top (cylindrical end)
    public float zInjector;     // injector faceplate
    public float zTotal;        // total engine length

    // ============================================================
    // COMPUTED — Step 3: Heat Transfer
    // ============================================================

    // These are stored as functions of z (mm), not arrays
    // Use SpikeProfile(z), ShroudProfile(z) etc. from ChamberSizing

    public float qThroat;       // W/m² — peak heat flux at throat
    public float wallThroat;    // mm — wall thickness at throat
    public float chRadiusMin;   // mm — channel radius at throat (narrowest)
    public float chRadiusMax;   // mm — channel radius in chamber (widest)

    // ============================================================
    // COMPUTED — Step 3b: Coolant Flow & Channel Count
    // ============================================================

    public float mDot_fuel;        // kg/s — fuel mass flow (= shroud coolant)
    public float mDot_ox;          // kg/s — oxidizer mass flow
    public float mDot_cool_spike;  // kg/s — LOX through spike cooling channels
    public int   nChannelsShroud;  // N = 2πr / (2·r_ch + wall) at throat
    public int   nChannelsSpike;   // computed from spike throat geometry

    // ============================================================
    // METHODS
    // ============================================================

    public void LogSummary()
    {
        Library.Log("╔══════════════════════════════════════════╗");
        Library.Log("║   OpenSpaceArch v4 — Engine Parameters   ║");
        Library.Log("╚══════════════════════════════════════════╝");
        Library.Log("");
        Library.Log("── INPUTS ──────────────────────────────────");
        Library.Log($"  Thrust:     {F_thrust:F0} N ({F_thrust/1000:F1} kN)");
        Library.Log($"  Pc:         {Pc/1e5:F0} bar");
        Library.Log($"  Propellant: LOX/CH4, O/F={OF_ratio:F1}");
        Library.Log($"  Material:   CuCrZr (k={k_wall}, σ={sigma_yield/1e6:F0} MPa)");
        Library.Log($"  Voxel:      {voxelSize:F1} mm");
        Library.Log("");
        Library.Log("── THERMOCHEMISTRY ─────────────────────────");
        Library.Log($"  Tc:         {Tc:F0} K");
        Library.Log($"  gamma:      {gamma:F3}");
        Library.Log($"  c*:         {cStar:F0} m/s");
        Library.Log($"  Cf:         {Cf:F3}");
        Library.Log($"  Isp (SL):   {Isp_SL:F1} s");
        Library.Log($"  Isp (vac):  {Isp_vac:F1} s");
        Library.Log($"  ṁ:          {mDot:F3} kg/s");
        Library.Log("");
        Library.Log("── SIZING ──────────────────────────────────");
        Library.Log($"  At:         {At*1e6:F1} mm² ({Dt*1000:F1} mm eq. diameter)");
        Library.Log($"  Spike @thr: {rSpikeThroat:F1} mm");
        Library.Log($"  Shroud @thr:{rShroudThroat:F1} mm");
        Library.Log($"  Gap @thr:   {rShroudThroat-rSpikeThroat:F1} mm");
        Library.Log($"  Spike @ch:  {rSpikeChamber:F1} mm");
        Library.Log($"  Shroud @ch: {rShroudChamber:F1} mm");
        Library.Log($"  Lc:         {Lc:F1} mm");
        Library.Log($"  Total L:    {zTotal:F1} mm");
        Library.Log("");
        Library.Log("── Z-STATIONS ──────────────────────────────");
        Library.Log($"  zTip:       {zTip:F1} mm");
        Library.Log($"  zCowl:      {zCowl:F1} mm");
        Library.Log($"  zThroat:    {zThroat:F1} mm");
        Library.Log($"  zChBot:     {zChBot:F1} mm");
        Library.Log($"  zChTop:     {zChTop:F1} mm");
        Library.Log($"  zInjector:  {zInjector:F1} mm");
        Library.Log($"  zTotal:     {zTotal:F1} mm");
        Library.Log("");
        Library.Log("── HEAT TRANSFER ───────────────────────────");
        Library.Log($"  q(throat):  {qThroat/1e6:F1} MW/m²");
        Library.Log($"  wall @thr:  {wallThroat:F2} mm");
        Library.Log($"  ch.R @thr:  {chRadiusMin:F2} mm");
        Library.Log($"  ch.R @ch:   {chRadiusMax:F2} mm");
        Library.Log("");
        Library.Log("── COOLANT FLOW ────────────────────────────");
        Library.Log($"  ṁ_fuel:     {mDot_fuel:F3} kg/s (shroud coolant)");
        Library.Log($"  ṁ_ox:       {mDot_ox:F3} kg/s");
        Library.Log($"  ṁ_cool_sp:  {mDot_cool_spike:F3} kg/s ({spikeCoolFraction*100:F0}% LOX)");
        Library.Log($"  N shroud:   {nChannelsShroud} (from mass flow + geometry)");
        Library.Log($"  N spike:    {nChannelsSpike}");
        Library.Log("");
    }
}
