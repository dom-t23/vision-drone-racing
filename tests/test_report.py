"""Write-up figures: every listed results file exists and the numbers match the files."""

import os

from vdr import report


def test_report_files_exist_and_figures_render(tmp_path):
    names = [f for f, _, _ in report.HISTORY]
    names += [f for _, pairs in report.ABLATIONS + report.REJECTED + report.CAMONLY for pair in pairs for f in pair]
    names += [f for _, f in report.CLUTTER]
    missing = [n for n in names if not os.path.exists(os.path.join(report.RESULTS, n))]
    assert not missing, missing
    rows = report.plot_history(str(tmp_path / "h.png"))
    assert abs(rows[0][2] - 100 * report.load(report.HISTORY[0][0])["pass_rate"]) < 1e-9
    report.plot_ablations(str(tmp_path / "a.png"))
    clutter = report.plot_clutter(str(tmp_path / "c.png"))
    assert [r[0] for r in clutter] == sorted(r[0] for r in clutter)
    pngs = ["h.png", "a.png", "c.png"]
    for i in range(len(report.PAIRED)):
        stats = report.plot_paired(str(tmp_path / f"p{i}.png"), which=i)
        assert stats["n"] > 0
        pngs.append(f"p{i}.png")
    assert all((tmp_path / n).stat().st_size > 1000 for n in pngs)
