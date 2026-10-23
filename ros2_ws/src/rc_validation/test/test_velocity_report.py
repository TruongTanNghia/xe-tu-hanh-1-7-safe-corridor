import json

from rc_validation.velocity_report import SOURCES, write_velocity_results


def test_write_velocity_results_creates_machine_and_human_reports(tmp_path):
    samples = {}
    for source in SOURCES:
        samples[source] = [
            {
                "elapsed_s": index * 0.1,
                "stamp_sec": 10,
                "stamp_nanosec": index * 100_000_000,
                "value_mps": 0.0,
            }
            for index in range(20)
        ]

    passed, result = write_velocity_results(
        samples=samples,
        output_dir=tmp_path,
        expected_speed_mps=0.0,
        spike_threshold_mps=0.5,
        duration_sec=1.9,
        source_kind="rosbag",
        source_path="/tmp/example_bag",
    )

    assert passed
    assert result["source_kind"] == "rosbag"
    assert (tmp_path / "velocity_samples.csv").is_file()
    assert (tmp_path / "report.md").is_file()
    saved = json.loads((tmp_path / "metrics.json").read_text())
    assert saved["passed"] is True
    assert saved["metrics"]["filtered"]["count"] == 20
