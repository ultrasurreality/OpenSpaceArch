// BoundedFloat.cs — Phase 1 of two-layer CSP architecture
//
// Placeholder primitive for Phase 2 bounded random sweep. Wraps a scalar
// parameter with [Min, Max] bounds and a deterministic sampling method
// via Leap71.ShapeKernel.Uf random helpers.
//
// NOT used in Phase 1 — this file exists so that Phase 2 can start
// immediately without file-creation churn. Phase 1 only adds live
// constraint visualization on top of existing EngineValidator.
//
// See ~/LEAP71_Knowledge/Инсайты/Архитектурный синтез CSP.md for rationale.

using Leap71.ShapeKernel;

namespace OpenSpaceArch.Engine.CSP;

/// <summary>
/// A scalar parameter bounded to [Min, Max] with a current value. Solver
/// samples from the range via seeded Random for reproducibility.
/// </summary>
public readonly record struct BoundedFloat(float Min, float Max, float Current)
{
    /// <summary>Uniform sample from [Min, Max].</summary>
    public float Sample(Random rng) => Uf.fGetRandomLinear(Min, Max, rng);

    /// <summary>
    /// Gaussian sample centered within the range at <paramref name="centerFrac"/>
    /// of the span, with <paramref name="stdFrac"/> of the span as stddev.
    /// Clamped to [Min, Max] to stay inside bounds.
    /// </summary>
    public float SampleGaussian(Random rng, float centerFrac = 0.5f, float stdFrac = 0.25f)
    {
        float center = Min + (Max - Min) * centerFrac;
        float std = (Max - Min) * stdFrac;
        float value = Uf.fGetRandomGaussian(center, std, rng);
        return Math.Clamp(value, Min, Max);
    }

    /// <summary>[0..1] fraction of <see cref="Current"/> within the range.</summary>
    public float Normalized => (Max > Min) ? (Current - Min) / (Max - Min) : 0.5f;

    /// <summary>Returns a copy with a new current value (clamped to bounds).</summary>
    public BoundedFloat With(float current) =>
        this with { Current = Math.Clamp(current, Min, Max) };

    public override string ToString() => $"{Current:F3} ∈ [{Min:F3}, {Max:F3}]";
}
