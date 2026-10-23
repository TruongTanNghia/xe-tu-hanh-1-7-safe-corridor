import csv
from datetime import datetime, timezone
import json
from pathlib import Path

from rc_validation.velocity_metrics import (
    calculate_metrics,
    evaluate_filtered_metrics,
)


SOURCES = ("raw", "conditioned", "filtered")


def write_velocity_results(
    *,
    samples,
    output_dir,
    expected_speed_mps,
    spike_threshold_mps,
    duration_sec,
    started_utc=None,
    source_kind="live",
    source_path="",
):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics = {
        source: calculate_metrics(
            samples[source],
            expected_speed_mps,
            spike_threshold_mps,
        )
        for source in SOURCES
    }
    passed, failures = evaluate_filtered_metrics(
        metrics["filtered"],
        expected_speed_mps,
    )
    result = {
        "started_utc": (
            started_utc or datetime.now(timezone.utc)
        ).isoformat(),
        "source_kind": source_kind,
        "source_path": str(source_path),
        "duration_sec": float(duration_sec),
        "expected_speed_mps": float(expected_speed_mps),
        "spike_threshold_mps": float(spike_threshold_mps),
        "passed": passed,
        "failures": failures,
        "metrics": metrics,
    }

    with (output_dir / "velocity_samples.csv").open(
        "w",
        encoding="utf-8",
        newline="",
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["source", "elapsed_s", "stamp_sec", "stamp_nanosec", "vx_mps"]
        )
        for source in SOURCES:
            for sample in samples[source]:
                writer.writerow(
                    [
                        source,
                        f"{sample['elapsed_s']:.6f}",
                        sample["stamp_sec"],
                        sample["stamp_nanosec"],
                        f"{sample['value_mps']:.9f}",
                    ]
                )

    (output_dir / "metrics.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    lines = [
        "# Báo cáo đánh giá vận tốc",
        "",
        f"- Kết quả: **{'PASS' if passed else 'FAIL'}**",
        f"- Nguồn: `{source_kind}`",
        f"- Expected speed: `{expected_speed_mps:.3f} m/s`",
        f"- Duration: `{duration_sec:.3f} s`",
        f"- Failures: `{', '.join(failures) if failures else 'none'}`",
        "",
        "| Source | Count | Mean | Stddev | P95 abs error | Spikes | Rate |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for source in SOURCES:
        item = metrics[source]

        def format_value(value):
            return "n/a" if value is None else f"{value:.4f}"

        lines.append(
            f"| {source} | {item['count']} | "
            f"{format_value(item['mean_mps'])} | "
            f"{format_value(item['stddev_mps'])} | "
            f"{format_value(item['p95_abs_error_mps'])} | "
            f"{item['spike_count']} | "
            f"{item['sample_rate_hz']:.2f} Hz |"
        )
    (output_dir / "report.md").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )
    return passed, result

