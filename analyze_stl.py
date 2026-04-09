#!/usr/bin/env python3
"""
analyze_stl.py — STL geometry analyzer for aerospike rocket engines.
Produces a structured text report for automated iteration.

Usage:
    python analyze_stl.py AerospikeV3.stl
    python analyze_stl.py AerospikeV3.stl --json spec.json
    python analyze_stl.py AerospikeV3.stl --plot
"""

import argparse
import json
import sys
import os
from datetime import datetime
from pathlib import Path

import numpy as np
import trimesh
from scipy import ndimage
from matplotlib.path import Path as MplPath


# ─────────────────────────────────────────────
# 1. GLOBAL METRICS
# ─────────────────────────────────────────────

def global_metrics(mesh, density=8900.0):
    bb = mesh.bounds  # (2,3) min/max
    dims = mesh.extents
    verts_r = np.sqrt(mesh.vertices[:, 0]**2 + mesh.vertices[:, 1]**2)
    vol = mesh.volume if mesh.is_volume else float('nan')
    mass_g = vol * density / 1e9 * 1000 if not np.isnan(vol) else float('nan')

    lines = []
    lines.append("== 1. GLOBAL METRICS ==")
    lines.append(f"Bounding box: X[{bb[0][0]:.1f}, {bb[1][0]:.1f}] "
                 f"Y[{bb[0][1]:.1f}, {bb[1][1]:.1f}] "
                 f"Z[{bb[0][2]:.1f}, {bb[1][2]:.1f}]")
    lines.append(f"Dimensions:   {dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm")
    lines.append(f"Total height: {dims[2]:.1f} mm (Z range)")
    lines.append(f"Max radius:   {verts_r.max():.1f} mm")
    lines.append(f"Volume:       {vol:.0f} mm3 ({vol/1e3:.1f} cm3)")
    lines.append(f"Surface area: {mesh.area:.0f} mm2")
    lines.append(f"Est. mass:    {mass_g:.0f} g (density {density:.0f} kg/m3)")
    lines.append(f"Watertight:   {'YES' if mesh.is_watertight else 'NO'}")
    lines.append(f"Triangles:    {len(mesh.faces)}")
    return lines


# ─────────────────────────────────────────────
# 2. CROSS-SECTION SLICING
# ─────────────────────────────────────────────

def slice_at_z(mesh, z_values):
    """Batch-slice mesh at given Z heights. Returns list of Path2D or None."""
    z_arr = np.array(z_values, dtype=float)
    z_min = mesh.bounds[0][2]
    origin = np.array([0, 0, z_min])
    normal = np.array([0, 0, 1])
    heights = z_arr - z_min
    paths = mesh.section_multiplane(origin, normal, heights)
    return paths


def analyze_cross_section(path2d, z, grid_res=0.1):
    """Analyze a single cross-section. Returns dict of measurements."""
    result = {
        "z": z, "has_material": False,
        "outer_r": 0, "inner_r": 0, "gap": 0,
        "wall_min": 0, "wall_max": 0, "wall_mean": 0,
        "material_area": 0, "n_loops": 0, "n_voids": 0,
    }

    if path2d is None:
        return result

    try:
        loops = path2d.discrete
    except Exception:
        return result

    if len(loops) == 0:
        return result

    result["has_material"] = True
    result["n_loops"] = len(loops)

    # Collect all points for global radii
    all_pts = np.vstack(loops)
    r_all = np.sqrt(all_pts[:, 0]**2 + all_pts[:, 1]**2)
    result["outer_r"] = float(r_all.max())
    result["inner_r"] = float(r_all.min())
    result["gap"] = result["outer_r"] - result["inner_r"]

    # ── Polar wall thickness analysis ──
    # Bin all contour points by angle, find radial extent per bin
    n_bins = 360
    theta_all = np.arctan2(all_pts[:, 1], all_pts[:, 0])
    bin_edges = np.linspace(-np.pi, np.pi, n_bins + 1)
    bin_idx = np.digitize(theta_all, bin_edges) - 1
    bin_idx = np.clip(bin_idx, 0, n_bins - 1)

    wall_thick = []
    for b in range(n_bins):
        mask = bin_idx == b
        if mask.sum() < 2:
            continue
        r_bin = r_all[mask]
        wall_thick.append(r_bin.max() - r_bin.min())

    if wall_thick:
        wt = np.array(wall_thick)
        result["wall_min"] = float(wt.min())
        result["wall_max"] = float(wt.max())
        result["wall_mean"] = float(wt.mean())

    # ── Rasterize for void counting ──
    try:
        bb_min = all_pts.min(axis=0)
        bb_max = all_pts.max(axis=0)
        margin = 1.0  # mm
        nx = int((bb_max[0] - bb_min[0] + 2 * margin) / grid_res)
        ny = int((bb_max[1] - bb_min[1] + 2 * margin) / grid_res)

        # Limit grid size for memory safety
        if nx > 2000 or ny > 2000:
            grid_res_adj = max((bb_max[0] - bb_min[0] + 2 * margin) / 2000,
                               (bb_max[1] - bb_min[1] + 2 * margin) / 2000)
            nx = int((bb_max[0] - bb_min[0] + 2 * margin) / grid_res_adj)
            ny = int((bb_max[1] - bb_min[1] + 2 * margin) / grid_res_adj)
            grid_res = grid_res_adj

        x_lin = np.linspace(bb_min[0] - margin, bb_max[0] + margin, nx)
        y_lin = np.linspace(bb_min[1] - margin, bb_max[1] + margin, ny)
        xx, yy = np.meshgrid(x_lin, y_lin)
        grid_pts = np.column_stack([xx.ravel(), yy.ravel()])

        # Check which grid points are inside any contour
        inside = np.zeros(len(grid_pts), dtype=bool)
        for loop in loops:
            if len(loop) < 3:
                continue
            mpl_path = MplPath(loop)
            inside ^= mpl_path.contains_points(grid_pts)  # XOR for nested contours

        material_grid = inside.reshape(ny, nx)
        result["material_area"] = float(material_grid.sum() * grid_res * grid_res)

        # Flood fill exterior from corners
        exterior = np.zeros_like(material_grid, dtype=bool)
        exterior[0, :] = True
        exterior[-1, :] = True
        exterior[:, 0] = True
        exterior[:, -1] = True
        exterior = exterior & ~material_grid
        exterior_labeled, _ = ndimage.label(exterior)
        # All connected to border = exterior
        border_labels = set(exterior_labeled[0, :]) | set(exterior_labeled[-1, :]) | \
                        set(exterior_labeled[:, 0]) | set(exterior_labeled[:, -1])
        border_labels.discard(0)
        is_exterior = np.isin(exterior_labeled, list(border_labels))

        # Voids = not material AND not exterior
        voids = ~material_grid & ~is_exterior
        labeled_voids, n_voids = ndimage.label(voids)

        # Filter tiny voids (< 4 pixels = noise)
        real_voids = 0
        for v in range(1, n_voids + 1):
            if (labeled_voids == v).sum() >= 4:
                real_voids += 1
        result["n_voids"] = real_voids

    except Exception:
        pass  # rasterization failed, report what we have

    return result


def cross_section_report(mesh, z_stations):
    """Analyze cross-sections at named Z-stations."""
    names = list(z_stations.keys())
    z_vals = [z_stations[n] for n in names]
    paths = slice_at_z(mesh, z_vals)

    lines = []
    lines.append("== 2. CROSS-SECTIONS AT KEY Z-STATIONS ==")
    header = f"{'Station':<14} {'Z(mm)':>7} {'OutR':>7} {'InR':>7} {'Gap':>6} " \
             f"{'WMin':>6} {'WMax':>6} {'Area':>8} {'Loops':>5} {'Voids':>5}"
    lines.append(header)
    lines.append("-" * len(header))

    results = []
    for name, z, path in zip(names, z_vals, paths):
        cs = analyze_cross_section(path, z)
        results.append(cs)

        if not cs["has_material"]:
            lines.append(f"{name:<14} {z:>7.1f} {'---':>7} {'---':>7} {'---':>6} "
                         f"{'---':>6} {'---':>6} {'---':>8} {0:>5} {0:>5}")
        else:
            lines.append(f"{name:<14} {z:>7.1f} {cs['outer_r']:>7.1f} {cs['inner_r']:>7.1f} "
                         f"{cs['gap']:>6.1f} {cs['wall_min']:>6.2f} {cs['wall_max']:>6.2f} "
                         f"{cs['material_area']:>8.0f} {cs['n_loops']:>5} {cs['n_voids']:>5}")

    return lines, results


# ─────────────────────────────────────────────
# 3. WALL THICKNESS SCAN
# ─────────────────────────────────────────────

def wall_thickness_scan(mesh, z_min, z_max, n_steps=20, min_wall=0.5):
    """Sweep Z and measure wall thickness at each step."""
    z_vals = np.linspace(z_min, z_max, n_steps)
    paths = slice_at_z(mesh, z_vals)

    lines = []
    lines.append(f"== 3. WALL THICKNESS SCAN ({n_steps} steps, min threshold: {min_wall}mm) ==")
    lines.append(f"{'Z(mm)':>7}  {'WallMin(mm)':>11}  {'WallMax(mm)':>11}  Status")
    lines.append("-" * 50)

    problems = []
    for z, path in zip(z_vals, paths):
        cs = analyze_cross_section(path, z, grid_res=0.15)  # coarser for speed

        if not cs["has_material"]:
            status = "[NO MATERIAL]"
            # Only flag as problem if not near tip/top
            if z > z_min + 3 and z < z_max - 3:
                status = "[GAP!] <<<"
                problems.append(z)
            lines.append(f"{z:>7.1f}  {'---':>11}  {'---':>11}  {status}")
        elif cs["wall_min"] < min_wall:
            lines.append(f"{z:>7.1f}  {cs['wall_min']:>11.2f}  {cs['wall_max']:>11.2f}  "
                         f"[THIN] <<<")
            problems.append(z)
        else:
            lines.append(f"{z:>7.1f}  {cs['wall_min']:>11.2f}  {cs['wall_max']:>11.2f}  [OK]")

    if problems:
        lines.append(f"PROBLEMS: {len(problems)} issues at Z={[f'{z:.1f}' for z in problems]}")
    else:
        lines.append("All wall thicknesses OK.")

    return lines


# ─────────────────────────────────────────────
# 4. CHANNEL DETECTION
# ─────────────────────────────────────────────

def channel_detection(mesh, z_stations, expected_channels=None):
    """Detect cooling channel voids at key stations."""
    # Analyze at throat and mid-chamber
    targets = {}
    if "throat" in z_stations:
        targets["throat"] = z_stations["throat"]
    if "chBot" in z_stations and "chTop" in z_stations:
        targets["mid_chamber"] = (z_stations["chBot"] + z_stations["chTop"]) / 2
    elif "chTop" in z_stations:
        targets["chamber"] = z_stations["chTop"] - 5

    if not targets:
        # Fallback: use middle of Z range
        z_mid = (mesh.bounds[0][2] + mesh.bounds[1][2]) / 2
        targets["mid_height"] = z_mid

    lines = []
    lines.append("== 4. CHANNEL/VOID DETECTION ==")

    z_vals = list(targets.values())
    z_names = list(targets.keys())
    paths = slice_at_z(mesh, z_vals)

    for name, z, path in zip(z_names, z_vals, paths):
        cs = analyze_cross_section(path, z, grid_res=0.08)  # finer for channels

        lines.append(f"--- Z={z:.1f}mm ({name}) ---")
        if not cs["has_material"]:
            lines.append("  No material at this Z")
            continue

        lines.append(f"  Voids found: {cs['n_voids']}")
        lines.append(f"  Outer R: {cs['outer_r']:.1f}mm, Inner R: {cs['inner_r']:.1f}mm")
        lines.append(f"  Loops: {cs['n_loops']}")

        if expected_channels:
            n_expected = expected_channels.get("nShroud", 0) + expected_channels.get("nSpike", 0)
            if n_expected > 0:
                lines.append(f"  Expected: {n_expected} total channels")
                if cs["n_voids"] == 0:
                    lines.append(f"  Status: [FAIL] No channels resolved!")
                elif abs(cs["n_voids"] - n_expected) <= n_expected * 0.2:
                    lines.append(f"  Status: [CLOSE] {cs['n_voids']}/{n_expected}")
                else:
                    lines.append(f"  Status: [MISMATCH] {cs['n_voids']} vs expected {n_expected}")
        lines.append("")

    return lines


# ─────────────────────────────────────────────
# 5. SYMMETRY CHECK
# ─────────────────────────────────────────────

def symmetry_check(mesh, z_stations):
    """Check rotational symmetry at key stations."""
    # Pick 3 stations
    check_keys = []
    for key in ["throat", "chBot", "chTop"]:
        if key in z_stations:
            check_keys.append(key)
    if len(check_keys) == 0:
        # Fallback
        z_mid = (mesh.bounds[0][2] + mesh.bounds[1][2]) / 2
        z_stations = {"mid": z_mid}
        check_keys = ["mid"]

    z_vals = [z_stations[k] for k in check_keys]
    paths = slice_at_z(mesh, z_vals)

    lines = []
    lines.append("== 5. SYMMETRY CHECK ==")

    for name, z, path in zip(check_keys, z_vals, paths):
        if path is None:
            lines.append(f"Z={z:.1f}mm ({name}): no material")
            continue

        try:
            all_pts = np.vstack(path.discrete)
        except Exception:
            lines.append(f"Z={z:.1f}mm ({name}): could not extract points")
            continue

        r = np.sqrt(all_pts[:, 0]**2 + all_pts[:, 1]**2)
        theta = np.arctan2(all_pts[:, 1], all_pts[:, 0])

        # 4 quadrants
        q_means = []
        for q in range(4):
            lo = -np.pi + q * np.pi / 2
            hi = lo + np.pi / 2
            mask = (theta >= lo) & (theta < hi)
            if mask.sum() > 0:
                q_means.append(r[mask].mean())
            else:
                q_means.append(0)

        overall_mean = np.mean(q_means) if np.mean(q_means) > 0 else 1
        max_dev = max(abs(qm - overall_mean) / overall_mean * 100 for qm in q_means)
        centroid = np.sqrt(all_pts[:, 0].mean()**2 + all_pts[:, 1].mean()**2)

        status = "[OK]" if max_dev < 10 else "[ASYM]"
        lines.append(f"Z={z:.1f}mm ({name}):  max deviation {max_dev:.1f}%  "
                     f"centroid offset {centroid:.2f}mm  {status}")

    return lines


# ─────────────────────────────────────────────
# 6. PROFILE COMPARISON
# ─────────────────────────────────────────────

def profile_comparison(mesh, spec):
    """Compare actual radii vs expected from spec."""
    radii = spec.get("radii", {})
    z_st = spec.get("z_stations", {})

    if not radii or not z_st:
        return ["== 6. PROFILE COMPARISON ==", "  Skipped (no expected radii in spec)"]

    # Build expected profile points
    expected_pts = []
    if "throat" in z_st:
        expected_pts.append(("throat", z_st["throat"],
                             radii.get("spikeThroat", 0), radii.get("shroudThroat", 0)))
    if "chBot" in z_st:
        expected_pts.append(("chBot", z_st["chBot"],
                             radii.get("spikeChamber", 0), radii.get("shroudChamber", 0)))
    if "chTop" in z_st:
        expected_pts.append(("chTop", z_st["chTop"],
                             radii.get("spikeChamber", 0), radii.get("shroudChamber", 0)))

    if not expected_pts:
        return ["== 6. PROFILE COMPARISON ==", "  Skipped (insufficient data)"]

    z_vals = [p[1] for p in expected_pts]
    paths = slice_at_z(mesh, z_vals)

    lines = []
    lines.append("== 6. PROFILE COMPARISON ==")
    lines.append(f"{'Station':<12} {'Z':>6} {'ExpSpkR':>8} {'ActSpkR':>8} {'Dev':>6} "
                 f"{'ExpShrR':>8} {'ActShrR':>8} {'Dev':>6}")
    lines.append("-" * 68)

    for (name, z, exp_spike, exp_shroud), path in zip(expected_pts, paths):
        if path is None:
            lines.append(f"{name:<12} {z:>6.1f}  no material")
            continue

        cs = analyze_cross_section(path, z)
        act_inner = cs["inner_r"]
        act_outer = cs["outer_r"]

        # Inner radius ≈ spike surface, outer ≈ shroud outer (includes wall+channels)
        dev_spike = act_inner - exp_spike if exp_spike > 0 else float('nan')
        dev_shroud = act_outer - exp_shroud if exp_shroud > 0 else float('nan')

        lines.append(f"{name:<12} {z:>6.1f} {exp_spike:>8.1f} {act_inner:>8.1f} "
                     f"{dev_spike:>+6.1f} {exp_shroud:>8.1f} {act_outer:>8.1f} "
                     f"{dev_shroud:>+6.1f}")

    return lines


# ─────────────────────────────────────────────
# 7. PLOT (optional)
# ─────────────────────────────────────────────

def save_cross_section_plots(mesh, z_stations, out_dir):
    """Save cross-section PNG plots."""
    import matplotlib.pyplot as plt

    z_names = list(z_stations.keys())
    z_vals = [z_stations[n] for n in z_names]
    paths = slice_at_z(mesh, z_vals)

    n = len(z_vals)
    cols = min(3, n)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 5 * rows))
    if n == 1:
        axes = np.array([axes])
    axes = np.atleast_2d(axes)

    for idx, (name, z, path) in enumerate(zip(z_names, z_vals, paths)):
        r, c = divmod(idx, cols)
        ax = axes[r][c]
        ax.set_aspect('equal')
        ax.set_title(f"{name} (Z={z:.1f}mm)")
        ax.grid(True, alpha=0.3)

        if path is None:
            ax.text(0.5, 0.5, "No material", transform=ax.transAxes, ha='center')
            continue

        for loop in path.discrete:
            loop_closed = np.vstack([loop, loop[0]])
            ax.plot(loop_closed[:, 0], loop_closed[:, 1], 'b-', linewidth=0.5)

    # Hide empty subplots
    for idx in range(n, rows * cols):
        r, c = divmod(idx, cols)
        axes[r][c].set_visible(False)

    plt.tight_layout()
    out_path = os.path.join(out_dir, "cross_sections.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Analyze aerospike engine STL geometry")
    parser.add_argument("stl_path", help="Path to STL file")
    parser.add_argument("--json", dest="json_path", help="Path to spec JSON sidecar")
    parser.add_argument("--density", type=float, default=8900.0,
                        help="Material density kg/m3 (default: 8900 CuCrZr)")
    parser.add_argument("--min-wall", type=float, default=0.5,
                        help="Min wall thickness warning threshold mm (default: 0.5)")
    parser.add_argument("--plot", action="store_true", help="Save PNG cross-section plots")
    args = parser.parse_args()

    # ── Load STL ──
    stl_path = os.path.abspath(args.stl_path)
    if not os.path.exists(stl_path):
        print(f"ERROR: File not found: {stl_path}")
        sys.exit(1)

    print(f"Loading {stl_path}...")
    mesh = trimesh.load(stl_path)
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)

    # ── Load spec ──
    spec = {}
    json_path = args.json_path
    if not json_path:
        # Auto-discover *_spec.json next to STL
        base = stl_path.rsplit('.', 1)[0]
        candidates = [base + "_spec.json", base.replace("_Cutaway", "") + "_spec.json"]
        for c in candidates:
            if os.path.exists(c):
                json_path = c
                break

    if json_path and os.path.exists(json_path):
        with open(json_path) as f:
            spec = json.load(f)
        print(f"Loaded spec: {json_path}")

    # ── Determine Z-stations ──
    z_stations = spec.get("z_stations", {})
    if not z_stations:
        # Auto: 6 evenly spaced + tip/top
        z_lo, z_hi = mesh.bounds[0][2], mesh.bounds[1][2]
        z_stations = {
            f"z{i}": z_lo + i * (z_hi - z_lo) / 7
            for i in range(8)
        }

    density = spec.get("material", {}).get("density", args.density)

    # ── Generate report ──
    report = []
    report.append("=" * 50)
    report.append(f"STL ANALYSIS: {os.path.basename(stl_path)}")
    report.append(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("=" * 50)
    report.append("")

    # Section 1
    report.extend(global_metrics(mesh, density))
    report.append("")

    # Section 2
    cs_lines, cs_results = cross_section_report(mesh, z_stations)
    report.extend(cs_lines)
    report.append("")

    # Section 3
    z_lo, z_hi = mesh.bounds[0][2], mesh.bounds[1][2]
    report.extend(wall_thickness_scan(mesh, z_lo, z_hi, n_steps=20,
                                       min_wall=args.min_wall))
    report.append("")

    # Section 4
    expected_ch = spec.get("channels", None)
    report.extend(channel_detection(mesh, z_stations, expected_ch))
    report.append("")

    # Section 5
    report.extend(symmetry_check(mesh, z_stations))
    report.append("")

    # Section 6 (only if spec has radii)
    if spec.get("radii"):
        report.extend(profile_comparison(mesh, spec))
        report.append("")

    # Section 7: Summary
    report.append("== SUMMARY ==")
    # Collect verdicts
    has_thin = any("[THIN]" in l for l in report)
    has_gap = any("[GAP!]" in l for l in report)
    has_asym = any("[ASYM]" in l for l in report)
    has_ch_fail = any("[FAIL]" in l for l in report)
    watertight = mesh.is_watertight

    report.append(f"{'[OK]' if watertight else '[WARN]'}  Watertight: {watertight}")
    report.append(f"{'[FAIL]' if has_gap else '[OK]'}  Geometry gaps: "
                  f"{'FOUND' if has_gap else 'none'}")
    report.append(f"{'[WARN]' if has_thin else '[OK]'}  Thin walls: "
                  f"{'FOUND' if has_thin else 'none below threshold'}")
    report.append(f"{'[WARN]' if has_asym else '[OK]'}  Symmetry: "
                  f"{'asymmetric' if has_asym else 'OK'}")
    report.append(f"{'[FAIL]' if has_ch_fail else '[OK]'}  Channels: "
                  f"{'NOT resolved' if has_ch_fail else 'detected'}")
    report.append("=" * 50)

    # ── Output ──
    report_text = "\n".join(report)
    print(report_text)

    # Save report next to STL
    report_path = stl_path.rsplit('.', 1)[0] + "_analysis.txt"
    with open(report_path, 'w') as f:
        f.write(report_text)
    print(f"\nReport saved: {report_path}")

    # ── Optional plots ──
    if args.plot:
        out_dir = os.path.dirname(stl_path)
        plot_path = save_cross_section_plots(mesh, z_stations, out_dir)
        print(f"Plots saved: {plot_path}")


if __name__ == "__main__":
    main()
