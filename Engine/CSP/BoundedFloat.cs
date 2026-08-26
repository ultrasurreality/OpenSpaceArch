// SPDX-License-Identifier: AGPL-3.0-or-later
//
// OpenSpaceArch — Open Computational Architecture for Aerospace Hardware
// Copyright (C) 2025-2026 ultrasurreality
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// BoundedFloat.cs — core sampling primitive of the two-layer CSP architecture
//
// Wraps a scalar parameter with [Min, Max] bounds plus deterministic, seeded
// sampling via Leap71.ShapeKernel.Uf random helpers. This is the building
// block of the Phase 2 outer sweep: SweepVariables holds one BoundedFloat per
// search dimension, and OuterSweep draws an AeroSpec by sampling each one.
//
// Two sampling modes:
//   • Sample()         — uniform over [Min, Max]; the default exploratory sweep.
//   • SampleGaussian()  — clamped Gaussian centered in the range; the optional
//                         center-biased refinement mode (OuterSweep.Config.CenterBiased).
//
// See ARCHITECTURE.md, "SEARCH (CSP layer)", for the rationale.

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
