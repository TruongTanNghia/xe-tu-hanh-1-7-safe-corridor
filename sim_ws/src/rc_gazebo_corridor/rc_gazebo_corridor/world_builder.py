"""Generate a Gazebo Classic world: two-lane oval track + scenario obstacles.

The geometry is the same OvalTrack used by rc_safe_corridor's lightweight
simulator (straights 3.5 m, centre-line radius 0.925 m, drivable half width
0.45 m = two 0.45 m lanes), so every scenario behaves the same in both.

    python3 -m rc_gazebo_corridor.world_builder --scenario tour --output /tmp/w.sdf
"""

import argparse
import math

from rc_safe_corridor.sim_track import SCENARIOS, OvalTrack, initially_active, is_triggered


ROAD_HALF_WIDTH = 0.45
BORDER_WIDTH = 0.04
SEGMENT_M = 0.10
OBSTACLE_HEIGHT = 0.40   # taller than the LiDAR scan plane (0.239 m above ground)
PARK_X = 40.0            # where inactive (timed) obstacles wait


def _pose(x, y, z, yaw):
    return f"{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}"


def _box_visual(name, x, y, z, yaw, sx, sy, sz, rgba):
    return f"""
        <visual name="{name}">
          <pose>{_pose(x, y, z, yaw)}</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
        </visual>"""


def track_model(track: OvalTrack):
    visuals = []
    n = int(math.ceil(track.length / SEGMENT_M))
    seg = track.length / n
    for i in range(n):
        s_mid = (i + 0.5) * seg
        x, y, h, k = track.pose(s_mid)
        # Segments on the curve are wedges; overlap a bit to hide the seams.
        length = seg * (1.0 + abs(k) * ROAD_HALF_WIDTH) + 0.01
        visuals.append(_box_visual(f"road_{i}", x, y, 0.002, h, length, 2 * ROAD_HALF_WIDTH,
                                   0.004, "0.12 0.13 0.14 1"))
        for side, sign in (("l", 1.0), ("r", -1.0)):
            d = sign * (ROAD_HALF_WIDTH + BORDER_WIDTH / 2)
            bx, by, _h = track.frenet_to_xy(s_mid, d)
            blen = seg * (1.0 + abs(k) * abs(d)) + 0.01
            visuals.append(_box_visual(f"border_{side}_{i}", bx, by, 0.004, h, blen, BORDER_WIDTH,
                                       0.006, "0.95 0.77 0.10 1"))
        if i % 2 == 0:
            visuals.append(_box_visual(f"dash_{i}", x, y, 0.005, h, seg * 0.6, 0.015, 0.006,
                                       "0.9 0.9 0.9 1"))
    return f"""
    <model name="oval_track_2_lane">
      <static>true</static>
      <link name="track_link">{''.join(visuals)}
      </link>
    </model>"""


def obstacle_model(index, obs, track: OvalTrack):
    x, y, h = track.frenet_to_xy(obs["s"], obs["d"])
    if not initially_active(obs):
        x, y = PARK_X + index, PARK_X
    dynamic = bool(obs.get("speed") or obs.get("lat_speed") or "appear_t" in obs
                   or "vanish_t" in obs or is_triggered(obs))
    if obs.get("length", 0.0) > 0.0:
        geom = (f"<box><size>{obs['length']:.3f} {obs['width']:.3f} "
                f"{OBSTACLE_HEIGHT:.3f}</size></box>")
        color = "0.85 0.35 0.10 1" if obs.get("speed") else "0.80 0.20 0.20 1"
    else:
        geom = (f"<cylinder><radius>{obs.get('radius', 0.05):.3f}</radius>"
                f"<length>{OBSTACLE_HEIGHT:.3f}</length></cylinder>")
        color = "0.95 0.55 0.10 1" if obs.get("lat_speed") else "0.95 0.45 0.05 1"
    return f"""
    <model name="obstacle_{index}">
      <static>{'false' if dynamic else 'true'}</static>
      <pose>{_pose(x, y, OBSTACLE_HEIGHT / 2, h)}</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def build_world(scenario="tour", track=None):
    track = track or OvalTrack()
    obstacles = SCENARIOS[scenario]
    obstacle_models = "".join(obstacle_model(i, o, track) for i, o in enumerate(obstacles))
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="safe_corridor_{scenario}">
    <physics name="default_physics" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.7 0.7 0.75 1</ambient>
      <background>0.55 0.7 0.85 1</background>
      <shadows>false</shadows>
    </scene>
    <gui>
      <camera name="user_camera">
        <pose>1.75 -3.2 4.2 0 0.95 1.5708</pose>
      </camera>
    </gui>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.85 0.86 0.84 1</ambient><diffuse>0.85 0.86 0.84 1</diffuse></material>
        </visual>
      </link>
    </model>
    {track_model(track)}
    {obstacle_models}
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>
  </world>
</sdf>
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="tour", choices=sorted(SCENARIOS))
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    with open(args.output, "w", encoding="utf-8") as stream:
        stream.write(build_world(args.scenario))
    print(args.output)


if __name__ == "__main__":
    main()
