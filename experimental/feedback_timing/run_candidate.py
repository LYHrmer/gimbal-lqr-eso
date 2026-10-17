#!/usr/bin/env python3
"""Build a patched temporary copy; never replace the public library or headers."""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from sim.run_sensitivity import file_hash


def assess(baseline, candidate):
    failures, comparisons = [], []
    if baseline["clocks"] != candidate["clocks"] or set(baseline["profiles"]) != set(candidate["profiles"]):
        return {"passed": False, "failures": ["clock/profile coverage changed"], "comparisons": []}
    for name, entry in baseline["profiles"].items():
        current = candidate["profiles"][name]
        expected = [(r["clock"], r["seed"]) for r in entry["cases"]]
        if expected != [(r["clock"], r["seed"]) for r in current["cases"]]:
            failures.append(name+": seed/case coverage changed")
            continue
        changes = {}
        for old, new in zip(entry["cases"], current["cases"]):
            clock = old["clock"]
            if set(old["variants"]) != set(new["variants"]):
                failures.append(f"{name}/{clock}: variants changed")
                continue
            for label, a in old["variants"].items():
                b = new["variants"][label]
                if a["actual_c_config"] != b["actual_c_config"]:
                    failures.append(f"{name}/{clock}: configuration changed")
                if old["expected_stale_fault"]:
                    if (b["first_latched_fault"] != a["first_latched_fault"] or
                            not a["first_latched_fault"] or a["first_latched_fault"]["status"] != 6 or
                            a["completed"] != b["completed"] or a.get("failure") != b.get("failure") or
                            bool(a.get("first_physical_limit_crossing")) != bool(b.get("first_physical_limit_crossing"))):
                        failures.append(f"{name}/{clock}: timeout or subsequent failure behavior changed")
                    continue
                if any(not m["completed"] or m["first_latched_fault"] or m.get("first_physical_limit_crossing")
                       for m in (a, b)):
                    failures.append(f"{name}/{clock}/{label}: baseline/candidate incomplete or faulted")
                    continue
                if any(type(m.get(key)) not in (int, float) or not math.isfinite(m[key]) or
                       m[key] < 0 or (m is a and m[key] == 0)
                       for m in (a, b) for key in ("position_rmse_deg", "position_max_abs_error_deg")):
                    failures.append(f"{name}/{clock}/{label}: invalid metric")
                    continue
                if label == "observe_only" or clock == "sync_1_0":
                    if abs(b["position_rmse_deg"]-a["position_rmse_deg"]) > 1e-12:
                        failures.append(f"{name}/{clock}/{label}: unchanged path differs")
                if label == "compensated":
                    changes.setdefault(clock, []).append([
                        100*(b[key]/a[key]-1) for key in ("position_rmse_deg", "position_max_abs_error_deg")])
        for clock, rows in changes.items():
            rmse, peak = (sum(r[i] for r in rows)/len(rows) for i in (0, 1))
            comparisons.append({"profile": name, "clock": clock, "mean_paired_rmse_change_percent": rmse,
                                "mean_paired_peak_change_percent": peak})
            if len(rows) != len(entry["seeds"]) or rmse > 5. or peak > 5.:
                failures.append(f"{name}/{clock}: regression >5% or incomplete pairs")
            if name.startswith("gm6020") and clock in ("sync_2_0", "angle500_gyro1000") and rmse > -10.:
                failures.append(f"{name}/{clock}: improvement <10%")
    return {"passed": not failures, "failures": failures, "comparisons": comparisons,
            "scope": "development screening only, not full acceptance or hardware validation"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    patch = Path(__file__).with_name("candidate.patch")
    patch_digest = file_hash(patch)
    baseline = json.loads(args.baseline.read_text())
    # A candidate must not be evaluated against measurements of another source.
    for name, digest in baseline["provenance"]["source_sha256"].items():
        if file_hash(ROOT/name) != digest:
            raise SystemExit("baseline source mismatch: "+name)
    with tempfile.TemporaryDirectory(prefix="feedback-candidate-", dir=ROOT/"build") as tmp:
        work = Path(tmp)
        for directory in ("src", "include", "sim", "profiles", "tools"):
            shutil.copytree(ROOT/directory, work/directory, ignore=shutil.ignore_patterns("__pycache__"))
        shutil.copyfile(ROOT/"CMakeLists.txt", work/"CMakeLists.txt")
        subprocess.run(["git", "apply", str(patch)], cwd=work, check=True)
        subprocess.run(["cmake", "-S", str(work), "-B", str(work/"build"),
                        "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"], check=True)
        subprocess.run(["cmake", "--build", str(work/"build"), "-j", "2"], check=True)
        result = subprocess.run([sys.executable, str(work/"sim/run_multirate.py"),
                                 "--output", str(output)], cwd=work)
        candidate = json.loads((output/"metrics.json").read_text())
        verdict = assess(baseline, candidate)
        if file_hash(patch) != patch_digest:
            raise RuntimeError("candidate patch changed during experiment")
        verdict.update(patch_sha256=patch_digest, evaluator_sha256=file_hash(Path(__file__)), baseline_sha256=file_hash(args.baseline),
                       candidate_sha256=file_hash(output/"metrics.json"), experiment_exit_code=result.returncode)
        if result.returncode:
            verdict["passed"] = False
            verdict["failures"].append("candidate experiment integrity check failed")
        (output/"assessment.json").write_text(json.dumps(verdict, indent=2, allow_nan=False)+"\n")
        print(json.dumps(verdict, indent=2), flush=True)
        if not verdict["passed"]:
            raise SystemExit("candidate rejected; all results retained")


if __name__ == "__main__":
    main()
