
"""
zn_comprehensive_analysis.py — Complete graphical analysis of Ku sweep

Creates a multi-panel comparison figure showing:
  1. All oscillation curves overlaid
  2. Tu vs Ku plot
  3. Z-N gains (Kp, Ki, Kd) vs Ku
  4. Quality score heatmap
  5. Individual annotated plots for each Ku

Usage:
    python3 zn_comprehensive_analysis.py /path/to/csv/folder

Requires:
    pip3 install matplotlib scipy pandas numpy
"""

import matplotlib
matplotlib.use("TkAgg")

import os
import sys
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from scipy.signal import find_peaks
from pathlib import Path


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════════════════

FOLDER = "/Users/admin/Downloads/aaa/"

# Peak detection thresholds
MIN_PROMINENCE_FACTOR = 0.4
MIN_DISTANCE_FACTOR = 0.03

# Z-N tuning rules to display
ZN_RULES = {
    "Classic PID": {"kp": 0.600, "ki_factor": 1.200, "kd_factor": 0.075},
    "Some Overshoot": {"kp": 0.333, "ki_factor": 0.667, "kd_factor": 0.111},
    "No Overshoot": {"kp": 0.200, "ki_factor": 0.400, "kd_factor": 0.067},
}

# ══════════════════════════════════════════════════════════════════════════════


def analyze_single_csv(filepath):
    """Extract oscillation data and compute Tu + quality metrics"""
    filename = os.path.basename(filepath)
    try:
        ku = float(filename.replace("zn_ku_", "").replace(".csv", ""))
    except:
        ku = None

    df = pd.read_csv(filepath)
    df.columns = df.columns.str.strip()
    t = df["time_ms"].values / 1000.0
    e = df["error"].values

    # Peak detection
    min_prominence = max(2, np.std(e) * MIN_PROMINENCE_FACTOR)
    min_distance = max(5, int(len(t) * MIN_DISTANCE_FACTOR))
    pos_idx, _ = find_peaks(e, prominence=min_prominence, distance=min_distance)
    neg_idx, _ = find_peaks(-e, prominence=min_prominence, distance=min_distance)

    # Tu calculation
    Tu = None
    Tu_std = None
    if len(pos_idx) >= 2:
        periods = np.diff(t[pos_idx])
        Tu = float(np.median(periods))
        Tu_std = float(np.std(periods))
    elif len(neg_idx) >= 2:
        periods = np.diff(t[neg_idx])
        Tu = float(np.median(periods))
        Tu_std = float(np.std(periods))

    # Quality metrics
    n_peaks = len(pos_idx) + len(neg_idx)
    
    # Amplitude consistency
    amp_variance = 1.0
    if len(pos_idx) >= 2:
        peak_amps = np.abs(e[pos_idx])
        amp_variance = np.std(peak_amps) / np.mean(peak_amps) if np.mean(peak_amps) > 0 else 1.0

    # Oscillation duration
    osc_duration = 0
    if len(pos_idx) >= 2:
        osc_duration = t[pos_idx[-1]] - t[pos_idx[0]]

    # Overall quality score (0-100)
    quality = 0
    if n_peaks >= 3 and Tu is not None:
        peak_score = min(40, n_peaks * 8)
        consistency_score = max(0, 30 * (1 - amp_variance / 0.35))
        duration_score = min(30, osc_duration / 2.0 * 30)
        quality = peak_score + consistency_score + duration_score

    return {
        "ku": ku,
        "filepath": filepath,
        "t": t,
        "e": e,
        "pos_idx": pos_idx,
        "neg_idx": neg_idx,
        "Tu": Tu,
        "Tu_std": Tu_std,
        "n_peaks": n_peaks,
        "amp_variance": amp_variance,
        "quality": quality,
    }


def compute_zn_gains(ku, Tu, rule_params):
    """Compute Kp, Ki, Kd for a given rule"""
    if Tu is None or Tu == 0:
        return None, None, None
    
    kp = rule_params["kp"] * ku
    ki = rule_params["ki_factor"] * ku / Tu
    kd = rule_params["kd_factor"] * ku * Tu
    return kp, ki, kd


def create_comprehensive_plot(results):
    """
    Generate the main comparison figure with 6 panels:
      Row 1: [All curves overlay] [Tu vs Ku]
      Row 2: [Kp vs Ku] [Ki vs Ku] 
      Row 3: [Kd vs Ku] [Quality heatmap]
    """
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.25)

    # Color map for different Ku values
    n_results = len(results)
    colors = plt.cm.viridis(np.linspace(0, 1, n_results))

    # ═══════════════════════════════════════════════════════════════════════
    # Panel 1: All oscillation curves overlaid
    # ═══════════════════════════════════════════════════════════════════════
    ax1 = fig.add_subplot(gs[0, 0])
    for i, res in enumerate(results):
        t, e, ku = res["t"], res["e"], res["ku"]
        ax1.plot(t, e, color=colors[i], linewidth=1.2, alpha=0.7, label=f"Ku={ku:.2f}")
    
    ax1.axhline(0, color="gray", linestyle="--", linewidth=0.8)
    ax1.set_xlabel("Time (s)", fontsize=10)
    ax1.set_ylabel("Error (degrees)", fontsize=10)
    ax1.set_title("All Oscillation Curves Overlay", fontsize=11, fontweight="bold")
    ax1.legend(loc="upper right", fontsize=7, ncol=2)
    ax1.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════════════════════
    # Panel 2: Tu vs Ku
    # ═══════════════════════════════════════════════════════════════════════
    ax2 = fig.add_subplot(gs[0, 1])
    ku_vals = [r["ku"] for r in results if r["Tu"] is not None]
    tu_vals = [r["Tu"] for r in results if r["Tu"] is not None]
    tu_errs = [r["Tu_std"] if r["Tu_std"] else 0 for r in results if r["Tu"] is not None]
    
    if ku_vals:
        ax2.errorbar(ku_vals, tu_vals, yerr=tu_errs, marker="o", markersize=8, 
                     linewidth=2, capsize=5, color="#e74c3c", label="Tu (oscillation period)")
        ax2.set_xlabel("Ku (Ultimate Gain)", fontsize=10)
        ax2.set_ylabel("Tu (seconds)", fontsize=10)
        ax2.set_title("Oscillation Period vs Ultimate Gain", fontsize=11, fontweight="bold")
        ax2.grid(True, alpha=0.3)
        ax2.legend()

    # ═══════════════════════════════════════════════════════════════════════
    # Panels 3-5: Z-N Gains (Kp, Ki, Kd) vs Ku for all rules
    # ═══════════════════════════════════════════════════════════════════════
    gain_axes = [
        (fig.add_subplot(gs[1, 0]), "Kp", "Proportional Gain"),
        (fig.add_subplot(gs[1, 1]), "Ki", "Integral Gain"),
        (fig.add_subplot(gs[2, 0]), "Kd", "Derivative Gain"),
    ]

    for ax, gain_type, title in gain_axes:
        for rule_name, rule_params in ZN_RULES.items():
            ku_list, gain_list = [], []
            
            for res in results:
                if res["Tu"] is not None:
                    kp, ki, kd = compute_zn_gains(res["ku"], res["Tu"], rule_params)
                    if kp is not None:
                        ku_list.append(res["ku"])
                        if gain_type == "Kp":
                            gain_list.append(kp)
                        elif gain_type == "Ki":
                            gain_list.append(ki)
                        else:  # Kd
                            gain_list.append(kd)
            
            if ku_list:
                ax.plot(ku_list, gain_list, marker="o", linewidth=2, 
                       markersize=6, label=rule_name)
        
        ax.set_xlabel("Ku (Ultimate Gain)", fontsize=10)
        ax.set_ylabel(f"{gain_type} value", fontsize=10)
        ax.set_title(f"{title} vs Ku (Z-N Rules)", fontsize=11, fontweight="bold")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    # ═══════════════════════════════════════════════════════════════════════
    # Panel 6: Quality Score Heatmap
    # ═══════════════════════════════════════════════════════════════════════
    ax6 = fig.add_subplot(gs[2, 1])
    ku_vals_all = [r["ku"] for r in results]
    quality_vals = [r["quality"] for r in results]
    
    # Color-coded bar chart
    bar_colors = ["#27ae60" if q > 70 else "#f39c12" if q > 40 else "#e74c3c" 
                  for q in quality_vals]
    bars = ax6.barh(range(len(ku_vals_all)), quality_vals, color=bar_colors)
    ax6.set_yticks(range(len(ku_vals_all)))
    ax6.set_yticklabels([f"Ku={k:.2f}" for k in ku_vals_all], fontsize=8)
    ax6.set_xlabel("Quality Score (0-100)", fontsize=10)
    ax6.set_title("Oscillation Quality by Ku", fontsize=11, fontweight="bold")
    ax6.set_xlim(0, 100)
    ax6.grid(True, axis="x", alpha=0.3)
    
    # Add score labels on bars
    for i, (bar, score) in enumerate(zip(bars, quality_vals)):
        ax6.text(score + 2, i, f"{score:.0f}", va="center", fontsize=8)

    # Overall title
    fig.suptitle("Ziegler-Nichols Comprehensive Analysis", 
                 fontsize=14, fontweight="bold", y=0.995)

    # Save
    out_path = os.path.join(FOLDER, "zn_comprehensive_analysis.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\n✓ Comprehensive analysis saved → {out_path}")
    plt.show()


def create_individual_plots(results):
    """Generate detailed annotated plots for each Ku value"""
    output_folder = os.path.join(FOLDER, "individual_plots")
    os.makedirs(output_folder, exist_ok=True)

    for res in results:
        ku = res["ku"]
        t, e = res["t"], res["e"]
        pos_idx, neg_idx = res["pos_idx"], res["neg_idx"]
        Tu = res["Tu"]
        quality = res["quality"]

        fig, ax = plt.subplots(figsize=(12, 6))
        
        # Plot signal
        ax.plot(t, e, color="#3498db", linewidth=1.5, label="Error")
        ax.axhline(0, color="gray", linestyle="--", linewidth=0.8)
        
        # Mark peaks
        if len(pos_idx):
            ax.scatter(t[pos_idx], e[pos_idx], marker="v", s=100, 
                      color="#e74c3c", zorder=5, label=f"Peaks (+) ×{len(pos_idx)}")
            for i, idx in enumerate(pos_idx):
                ax.annotate(f"P{i+1}", xy=(t[idx], e[idx]), 
                           xytext=(5, 8), textcoords="offset points",
                           fontsize=8, color="#c0392b")
        
        if len(neg_idx):
            ax.scatter(t[neg_idx], e[neg_idx], marker="^", s=100,
                      color="#27ae60", zorder=5, label=f"Troughs (−) ×{len(neg_idx)}")
            for i, idx in enumerate(neg_idx):
                ax.annotate(f"T{i+1}", xy=(t[idx], e[idx]),
                           xytext=(5, -15), textcoords="offset points",
                           fontsize=8, color="#229954")
        
        # Annotate Tu
        if Tu is not None and len(pos_idx) >= 2:
            p1_t, p2_t = t[pos_idx[0]], t[pos_idx[1]]
            p1_e, p2_e = e[pos_idx[0]], e[pos_idx[1]]
            y_ann = max(p1_e, p2_e) * 1.15
            ax.annotate("", xy=(p2_t, y_ann), xytext=(p1_t, y_ann),
                       arrowprops=dict(arrowstyle="<->", color="#f39c12", lw=2))
            ax.text((p1_t + p2_t) / 2, y_ann * 1.05, f"Tu = {Tu:.3f}s",
                   ha="center", color="#f39c12", fontsize=10, fontweight="bold")
        
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Error (degrees)", fontsize=10)
        
        status = "GOOD" if quality > 70 else ("MARGINAL" if quality > 40 else "POOR")
        color = "#27ae60" if quality > 70 else ("#f39c12" if quality > 40 else "#e74c3c")
        ax.set_title(f"Ku = {ku:.2f}  |  Tu = {Tu:.4f}s  |  Quality = {quality:.0f}/100  [{status}]",
                    fontsize=12, fontweight="bold", color=color)
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(True, alpha=0.3)
        
        out_file = os.path.join(output_folder, f"zn_ku_{ku:.2f}.png")
        plt.savefig(out_file, dpi=120, bbox_inches="tight")
        plt.close()
    
    print(f"✓ {len(results)} individual plots saved → {output_folder}/")


def print_summary_table(results):
    """Print comprehensive summary with Z-N gains for top candidates"""
    print("\n" + "="*100)
    print("  COMPREHENSIVE SUMMARY — Ranked by Quality")
    print("="*100)
    print(f"{'Rank':<6} {'Ku':<8} {'Tu (s)':<10} {'Quality':<10} {'Peaks':<8} {'Amp Var':<10} {'Status':<12}")
    print("-"*100)

    ranked = sorted(results, key=lambda r: r["quality"], reverse=True)

    for i, res in enumerate(ranked, start=1):
        ku = res["ku"]
        Tu = res["Tu"]
        Tu_str = f"{Tu:.4f}" if Tu else "---"
        quality = res["quality"]
        n_peaks = res["n_peaks"]
        amp_var = res["amp_variance"]
        
        status = "✓ GOOD" if quality > 70 else ("~ Marginal" if quality > 40 else "✗ Poor")
        
        print(f"{i:<6} {ku:<8.2f} {Tu_str:<10} {quality:<10.0f} {n_peaks:<8} {amp_var:<10.3f} {status:<12}")

    print("="*100)

    # Show Z-N gains for top 3 candidates
    top_candidates = [r for r in ranked if r["quality"] > 40 and r["Tu"] is not None][:3]
    
    if top_candidates:
        print("\n" + "="*100)
        print("  RECOMMENDED Z-N GAINS (Top Candidates)")
        print("="*100)
        
        for res in top_candidates:
            ku, Tu = res["ku"], res["Tu"]
            print(f"\n  Ku = {ku:.2f}  |  Tu = {Tu:.4f} s  |  Quality = {res['quality']:.0f}/100")
            print("  " + "-"*90)
            
            for rule_name, rule_params in ZN_RULES.items():
                kp, ki, kd = compute_zn_gains(ku, Tu, rule_params)
                print(f"  {rule_name:<20}  Kp={kp:>8.4f}   Ki={ki:>8.4f}   Kd={kd:>8.4f}")
        
        print("="*100)
    else:
        print("\n⚠ No Ku values produced clean oscillation. Adjust sweep range or test parameters.")
    
    print("")


def main():
    # Find all CSVs
    pattern = os.path.join(FOLDER, "zn_ku_*.csv")
    csv_files = sorted(glob.glob(pattern))

    if not csv_files:
        print(f"No files matching {pattern}")
        sys.exit(1)

    print(f"Found {len(csv_files)} CSV files in {FOLDER}")

    # Analyze all files
    print("Analyzing data...")
    results = [analyze_single_csv(f) for f in csv_files]
    results.sort(key=lambda r: r["ku"] if r["ku"] else 0)

    # Generate outputs
    print_summary_table(results)
    create_comprehensive_plot(results)
    create_individual_plots(results)

    print("\n✓ Analysis complete!")


if __name__ == "__main__":
    main()
