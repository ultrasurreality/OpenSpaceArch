// EngineState.cs — drives shader selection and simulation subsystems.
//
// Materializing: hologram dissolve reveal (current Phase 2 behavior)
// Running:       walls show heat map, plume particles spawning,
//                chamber glow, coolant flow, shock diamonds, HUD tickers live

namespace OpenSpaceArch.Viewer.Simulation;

public enum EngineState
{
    Materializing,
    Running,
}
