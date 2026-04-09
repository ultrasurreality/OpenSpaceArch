// EngineAssembly.cs — Just calls FluidFirst.Build()

using PicoGK;

namespace OpenSpaceArch.Engine;

public static class EngineAssembly
{
    public static Voxels Build(AeroSpec S)
    {
        return FluidFirst.Build(S);
    }
}
