// Phase 0 smoke test — verifies PicoGK Core works headless (no Viewer).
//
// Tests:
//  1. Library.InitHeadless() loads native picogk.1.7.dll
//  2. Lattice sphere → Voxels conversion
//  3. Voxels.CalculateProperties returns volume + bbox
//  4. Voxels → Mesh extraction has positive vertex/triangle count
//  5. Library.Shutdown() cleans up
//
// Success criteria: zero exceptions, non-zero volume, non-zero triangle count.

using System.Numerics;
using PicoGK;

Console.WriteLine("╔══════════════════════════════════════════╗");
Console.WriteLine("║  Phase 0 smoke test: PicoGK Core headless ║");
Console.WriteLine("╚══════════════════════════════════════════╝");

try
{
    Console.WriteLine("[1/5] Initializing PicoGK Core headless (voxel = 0.5 mm)...");
    Library.InitHeadless(voxelSizeMM: 0.5f);
    Console.WriteLine($"      Library: {Library.strName()} v{Library.strVersion()}");

    Console.WriteLine("[2/5] Creating sphere lattice (r = 10 mm)...");
    var sphere = new Lattice();
    sphere.AddSphere(Vector3.Zero, 10f);

    Console.WriteLine("[3/5] Voxelizing lattice...");
    var vox = new Voxels(sphere);

    Console.WriteLine("[4/5] Calculating properties...");
    vox.CalculateProperties(out float volumeMM3, out BBox3 bbox);
    float expectedVol = (4f / 3f) * MathF.PI * MathF.Pow(10f, 3f);
    Console.WriteLine($"      Volume:   {volumeMM3:F1} mm³");
    Console.WriteLine($"      Expected: {expectedVol:F1} mm³ (analytical 4/3πr³)");
    Console.WriteLine($"      Ratio:    {volumeMM3 / expectedVol:F3} (should be ~1.0)");
    Console.WriteLine($"      BBox:     ({bbox.vecMin.X:F2}, {bbox.vecMin.Y:F2}, {bbox.vecMin.Z:F2})");
    Console.WriteLine($"                ({bbox.vecMax.X:F2}, {bbox.vecMax.Y:F2}, {bbox.vecMax.Z:F2})");

    Console.WriteLine("[5/5] Extracting mesh...");
    var mesh = new Mesh(vox);
    int nVerts = mesh.nVertexCount();
    int nTris = mesh.nTriangleCount();
    Console.WriteLine($"      Vertices:  {nVerts}");
    Console.WriteLine($"      Triangles: {nTris}");

    // Sanity checks
    if (volumeMM3 <= 0)
        throw new Exception("Volume is zero or negative — voxelization failed");
    if (nVerts <= 0 || nTris <= 0)
        throw new Exception("Mesh extraction returned empty geometry");
    if (volumeMM3 / expectedVol < 0.9f || volumeMM3 / expectedVol > 1.1f)
        throw new Exception($"Volume deviates by >10% from analytical expected");

    Console.WriteLine();
    Console.WriteLine("✓ All checks passed. PicoGK Core works headless.");

    Library.Shutdown();
    Console.WriteLine("✓ Library.Shutdown() completed cleanly.");
}
catch (Exception ex)
{
    Console.WriteLine();
    Console.WriteLine($"✗ SMOKE TEST FAILED: {ex.GetType().Name}");
    Console.WriteLine($"  {ex.Message}");
    Console.WriteLine();
    Console.WriteLine(ex.StackTrace);
    Environment.Exit(1);
}
