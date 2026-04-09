// SpatialValidator.cs — Геометрическая валидация без вокселей
//
// Проверяет пространственные конфликты между всеми элементами двигателя
// чистой математикой. Каждый элемент — функция положения и размера от z.

using System.Numerics;

namespace OpenSpaceArch.Engine;

public static class SpatialValidator
{
    public record Conflict(string ElementA, string ElementB, float Z, float Distance, float MinRequired);

    public static List<Conflict> Validate(AeroSpec S)
    {
        var conflicts = new List<Conflict>();
        float minWall = S.minPrintWall; // 0.5mm — реальный LPBF минимум
        float zStep = 1f; // проверяем каждый мм

        // ── Каналы shroud vs камера (gas path) ──
        for (float z = S.zCowl + 2f; z <= S.zInjector - 2f; z += zStep)
        {
            float rShroud = ChamberSizing.ShroudProfile(S, z);
            if (rShroud < 2f) continue;
            float wall = HeatTransfer.WallThickness(S, z);
            var (cw, ch) = HeatTransfer.ChannelRect(S, z);

            // Внутренний край канала = rShroud + wall
            // Должен быть > rShroud (поверхность камеры) + minWall
            float channelInner = rShroud + wall;
            float gap = wall; // wall IS the gap
            if (gap < minWall)
                conflicts.Add(new("shroud_channel", "chamber", z, gap, minWall));

            // Каналы влезают по окружности?
            // Используем ту же формулу rCenter что в HeatTransfer.ChannelRect
            float rCenter = rShroud + wall + 2f;
            float circ = 2f * MathF.PI * rCenter;
            float needed = S.nChannelsShroud * (cw + S.minRibWall);
            if (needed > circ)
                conflicts.Add(new("shroud_channels", "circumference", z, circ, needed));
        }

        // ── Каналы spike vs камера ──
        for (float z = S.zCowl + 2f; z <= S.zInjector - 2f; z += zStep)
        {
            float rSpike = ChamberSizing.SpikeProfile(S, z);
            if (rSpike < 3f) continue;
            float wall = HeatTransfer.WallThickness(S, z);
            var (cw, ch) = HeatTransfer.ChannelRectSpike(S, z);

            float channelOuter = rSpike - wall;
            if (wall < minWall)
                conflicts.Add(new("spike_channel", "chamber", z, wall, minWall));

            // Каналы spike влезают?
            float rCenter = rSpike - wall - ch / 2f;
            if (rCenter < 2f) continue;
            float circ = 2f * MathF.PI * rCenter;
            float needed = S.nChannelsSpike * (cw + S.minRibWall);
            if (needed > circ)
                conflicts.Add(new("spike_channels", "circumference", z, circ, needed));
        }

        // Spike channels vs axial manifold — handled by mutual exclusion (voxOffset subtract)
        // Not a hard conflict — manifold carves through spike channels by design

        // ── Коллектор vs каналы shroud на zChTop ──
        {
            float z = S.zChTop;
            float rShroud = ChamberSizing.ShroudProfile(S, z);
            float wall = HeatTransfer.WallThickness(S, z);
            var (cw, ch) = HeatTransfer.ChannelRect(S, z);
            float chCenter = rShroud + wall + ch / 2f;
            float collectorR = S.manifoldRadius;

            // Коллектор на том же радиусе — должен перекрываться с каналами (это нормально)
            // Но проверяем: коллектор не вылезает за пределы shroud outer wall
            float outerEdge = chCenter + collectorR;
            float maxOuter = rShroud + wall + ch + 3f; // разумный предел
            if (outerEdge > maxOuter + 5f)
                conflicts.Add(new("collector", "outer_bound", z, maxOuter, outerEdge));
        }

        // Feed ports и вейны пересекают каналы BY DESIGN
        // Mutual exclusion (voxOffset subtract) вырезает каналы вокруг них
        // Это не ошибка — проверяем только что порт не слишком огромный
        {
            float zFuel = S.zCowl + 3f;
            float rShroud = ChamberSizing.ShroudProfile(S, zFuel);
            if (rShroud < 2f) rShroud = S.rShroudThroat;
            var (cw, ch) = HeatTransfer.ChannelRect(S, zFuel);
            float chCenter = rShroud + HeatTransfer.WallThickness(S, zFuel) + ch / 2f;
            float angularSpanPort = 2f * MathF.Atan2(S.feedPortRadius, chCenter);
            float angularSpanChannel = (cw + S.minRibWall) / chCenter;
            int affectedChannels = (int)MathF.Ceiling(angularSpanPort / angularSpanChannel);
            // Порт убивает >25% каналов — это проблема
            if (affectedChannels > S.nChannelsShroud / 4)
                conflicts.Add(new("fuel_port", "too_many_channels", zFuel,
                    affectedChannels, S.nChannelsShroud / 4f));
        }

        // ── Инжекторные отверстия vs каналы ──
        // Film cooling holes are INSIDE the chamber (rShroudChamber*0.85 < rShroud)
        // They don't conflict with shroud channels which are OUTSIDE the chamber wall
        // Only check: bolt holes through flange vs top of channels
        {
            float z = S.zInjector;
            float rShroud = ChamberSizing.ShroudProfile(S, z);
            float wall = HeatTransfer.WallThickness(S, z);
            var (cw, ch) = HeatTransfer.ChannelRect(S, z);
            float chOuterEdge = rShroud + wall + ch;

            float rBoltCircle = S.rShroudChamber + S.mountFlangeExtent * 0.6f;
            float rBolt = 2.0f;
            // Болты на фланце — снаружи каналов? Проверяем inner edge bolt vs outer edge channel
            float gapBolt = rBoltCircle - rBolt - chOuterEdge;
            if (gapBolt < minWall)
                conflicts.Add(new("bolt_hole", "shroud_channel_top", z, gapBolt, minWall));
        }

        return conflicts;
    }

    public static void PrintReport(AeroSpec S)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var conflicts = Validate(S);
        sw.Stop();

        Console.WriteLine($"\nSpatial validation: {sw.ElapsedTicks * 1000000 / System.Diagnostics.Stopwatch.Frequency} µs");
        Console.WriteLine($"Conflicts found: {conflicts.Count}");

        if (conflicts.Count == 0)
        {
            Console.WriteLine("  All clear — no spatial conflicts detected.");
            return;
        }

        // Группируем по типу конфликта
        var groups = conflicts.GroupBy(c => $"{c.ElementA} vs {c.ElementB}");
        foreach (var g in groups)
        {
            var worst = g.OrderBy(c => c.Distance).First();
            Console.WriteLine($"  {g.Key}: {g.Count()} conflicts, worst at z={worst.Z:F1}mm (gap={worst.Distance:F2}, need={worst.MinRequired:F2})");
        }
    }
}
