# Rover Persistent Traversability Map

ROS2 package that accumulates local traversability grids into a persistent `OccupancyGrid` map for rover navigation.

This package is designed to keep a memory of previously observed terrain while still allowing the map to be corrected when an area is observed again.

## Purpose

The goal of this package is to convert a local traversability grid into a persistent map.

A local traversability grid usually represents only the area currently seen by the robot. When the robot moves away, obstacles may disappear from the local map even if they are still present in the environment.

This package keeps those observations in a larger persistent map.

## Mapping Rule

The persistent map follows this logic:

```text
new obstacle measurement  -> store obstacle
new free measurement      -> store free
unknown measurement       -> keep previous value
not observed anymore      -> keep previous value
```

This means:

```text
obstacle seen once        -> remains in the map
obstacle no longer seen   -> remains in the map
same area seen as free    -> obstacle is removed
unknown area              -> does not overwrite previous data
```

This provides a permanent map that can still be corrected if the robot observes that an obstacle has moved or disappeared.

## Architecture

```text
/traversability_grid              nav_msgs/msg/OccupancyGrid
        │
        ▼
rover_persistent_traversability_map
        │
        ▼
/persistent_traversability_grid   nav_msgs/msg/OccupancyGrid
```

Typical complete navigation pipeline:

```text
/scan + /front_cloud
        │
        ▼
/merged_cloud
        │
        ▼
elevation_mapping
        │
        ▼
/traversability_grid
        │
        ▼
persistent_traversability_map_node
        │
        ▼
/persistent_traversability_grid
        │
        ▼
Nav2 costmap
```

## Subscribed Topics

| Topic                  | Type                         | Description               |
| ---------------------- | ---------------------------- | ------------------------- |
| `/traversability_grid` | `nav_msgs/msg/OccupancyGrid` | Local traversability grid |

## Published Topics

| Topic                             | Type                         | Description                   |
| --------------------------------- | ---------------------------- | ----------------------------- |
| `/persistent_traversability_grid` | `nav_msgs/msg/OccupancyGrid` | Persistent traversability map |

## Parameters

| Parameter          |                           Default | Description                        |
| ------------------ | --------------------------------: | ---------------------------------- |
| `input_map_topic`  |            `/traversability_grid` | Input local traversability grid    |
| `output_map_topic` | `/persistent_traversability_grid` | Output persistent grid             |
| `map_frame`        |                             `map` | Frame of the persistent map        |
| `resolution`       |                            `0.10` | Map resolution in meters per cell  |
| `width_m`          |                            `60.0` | Persistent map width in meters     |
| `height_m`         |                            `60.0` | Persistent map height in meters    |
| `origin_x`         |                           `-30.0` | Map origin x position              |
| `origin_y`         |                           `-30.0` | Map origin y position              |
| `publish_rate`     |                             `2.0` | Output publication rate in Hz      |
| `lethal_threshold` |                              `70` | Input value considered as obstacle |
| `free_threshold`   |                              `20` | Input value considered as free     |
| `occupied_value`   |                             `100` | Output value for occupied cells    |
| `free_value`       |                               `0` | Output value for free cells        |
| `unknown_value`    |                              `-1` | Output value for unknown cells     |

## Configuration

Default configuration file:

```text
config/persistent_traversability_map.yaml
```

Example:

```yaml
persistent_traversability_map_node:
  ros__parameters:
    input_map_topic: "/traversability_grid"
    output_map_topic: "/persistent_traversability_grid"

    map_frame: "map"

    resolution: 0.10
    width_m: 60.0
    height_m: 60.0
    origin_x: -30.0
    origin_y: -30.0

    publish_rate: 2.0

    lethal_threshold: 70
    free_threshold: 20

    occupied_value: 100
    free_value: 0
    unknown_value: -1
```

## Frame Requirements

The input map frame must be transformable into the configured `map_frame`.

For example, if the persistent map uses:

```yaml
map_frame: "map"
```

then the system must provide a valid TF from the input grid frame to `map`.

Check the input frame:

```bash
ros2 topic echo /traversability_grid --once | grep frame_id
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo map odom
```

or, depending on the input frame:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

For a persistent map, using a fixed frame such as `map` or `odom` is recommended.

Avoid using `base_link` for the persistent map, because `base_link` moves with the robot.

## Build

From the ROS2 workspace:

```bash
cd ~/ros2_ws

source /opt/ros/lyrical/setup.bash

colcon build --packages-select rover_persistent_traversability_map

source install/setup.bash
```

## Run

```bash
ros2 launch rover_persistent_traversability_map persistent_traversability_map.launch.py
```

## Check Output

```bash
ros2 topic list | grep persistent
ros2 topic echo /persistent_traversability_grid --once
ros2 topic hz /persistent_traversability_grid
```

## RViz Visualization

In RViz:

```text
Fixed Frame: map
Add -> Map -> /persistent_traversability_grid
```

If the package is configured with:

```yaml
map_frame: "odom"
```

then use:

```text
Fixed Frame: odom
```

## Nav2 Integration

The output map can be used as a `StaticLayer` in Nav2.

Example:

```yaml
traversability_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  enabled: true
  map_topic: /persistent_traversability_grid
  subscribe_to_updates: true
  map_subscribe_transient_local: true
  lethal_cost_threshold: 70
  trinary_costmap: true
  track_unknown_space: false
  use_maximum: true
```

This allows Nav2 to use the persistent traversability map instead of only the current local traversability grid.

## Save the Persistent Map

The map can be saved with `nav2_map_server`:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f rover_persistent_traversability_map \
  --ros-args \
  -r map:=/persistent_traversability_grid
```

This creates:

```text
rover_persistent_traversability_map.yaml
rover_persistent_traversability_map.pgm
```

## Notes

The node does not erase obstacles simply because they are no longer observed.

An obstacle is removed only when the same area is observed again as free.

This behavior is useful for outdoor rover navigation where sensor coverage changes as the robot moves.

## License

MIT License.
