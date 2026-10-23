import numpy as np

from rc_safe_corridor.scan_clustering import ClusterShape
from rc_safe_corridor.tracking import MultiObjectTracker, TrackerParams


def _det(x, y):
    return ClusterShape(
        center=np.array([x, y]),
        radius=0.05,
        length=0.1,
        width=0.1,
        yaw=0.0,
        shape="circle",
        point_count=8,
        min_range=float(np.hypot(x, y)),
    )


def test_static_object_is_confirmed_and_stays_static():
    tracker = MultiObjectTracker(TrackerParams(confirm_hits=3))
    rng = np.random.default_rng(1)
    for _ in range(10):
        noisy = _det(1.0 + rng.normal(0, 0.01), 0.2 + rng.normal(0, 0.01))
        confirmed = tracker.update([noisy], dt=0.15)
    assert len(confirmed) == 1
    track = confirmed[0]
    assert track.track_id == 1
    assert not track.is_dynamic
    assert np.allclose(track.position, [1.0, 0.2], atol=0.03)


def test_moving_object_becomes_dynamic_with_velocity_estimate():
    tracker = MultiObjectTracker(TrackerParams())
    x = 1.0
    for _ in range(15):
        x += 0.4 * 0.1
        confirmed = tracker.update([_det(x, 0.0)], dt=0.1)
    assert len(confirmed) == 1
    track = confirmed[0]
    assert track.is_dynamic
    assert abs(track.velocity[0] - 0.4) < 0.1
    assert abs(track.velocity[1]) < 0.05


def test_ghost_detection_is_not_confirmed_and_is_deleted():
    tracker = MultiObjectTracker(TrackerParams(confirm_hits=3, max_misses=2))
    tracker.update([_det(1.0, 0.0)], dt=0.1)
    assert tracker.confirmed_tracks() == []
    for _ in range(3):
        tracker.update([], dt=0.1)
    assert tracker.tracks == []


def test_ids_are_stable_with_two_objects_swapping_order():
    tracker = MultiObjectTracker(TrackerParams())
    a, b = (1.0, 0.3), (1.5, -0.3)
    for i in range(6):
        dets = [_det(*a), _det(*b)] if i % 2 == 0 else [_det(*b), _det(*a)]
        tracker.update(dets, dt=0.1)
    tracks = {t.track_id: t for t in tracker.confirmed_tracks()}
    assert set(tracks) == {1, 2}
    assert np.allclose(tracks[1].position, a, atol=0.02)
    assert np.allclose(tracks[2].position, b, atol=0.02)


def test_track_survives_short_occlusion():
    tracker = MultiObjectTracker(TrackerParams(max_misses=3, max_coast_s=0.6))
    for _ in range(4):
        tracker.update([_det(1.0, 0.0)], dt=0.1)
    tracker.update([], dt=0.1)
    tracker.update([], dt=0.1)
    confirmed = tracker.update([_det(1.0, 0.0)], dt=0.1)
    assert len(confirmed) == 1 and confirmed[0].track_id == 1
