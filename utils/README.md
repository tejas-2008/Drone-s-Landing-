# Utility Scripts

This directory contains helper utilities for processing and visualizing data in the Drone IBVS project.

## Contents

### `rosbag_plotter.py`
A comprehensive Python utility for extracting and visualizing time series data from ROSBAG files.

**Features:**
- ✓ Extract time series data from ROSBAG topics
- ✓ Single topic plotting with customizable styling
- ✓ Multi-topic overlay plotting
- ✓ Subplot organization for complex comparisons
- ✓ Grid enabled by default for better readability
- ✓ Save plots to high-resolution files
- ✓ Topic discovery and data caching

**Installation Requirements:**
```bash
pip install rosbag matplotlib numpy
```

**Quick Start:**

```python
from rosbag_plotter import ROSBagPlotter

# Load a ROSBAG file
plotter = ROSBagPlotter("path/to/bagfile.bag")

# Plot a single topic
plotter.plot_topic("/topic_name", y_label="Metric Name")
plotter.show()

# Or plot multiple topics
plotter.plot_multiple_topics(["/topic1", "/topic2"])
plotter.show()

# Save to file
plotter.save("output.png", dpi=300)
```

### `example_usage.py`
Demonstrates how to use the ROSBagPlotter utility with your ROSBAG files.

**Run it:**
```bash
python3 example_usage.py
```

This script will:
1. List all available topics in a ROSBAG file
2. Show commented examples of different plotting approaches

## Available ROSBAG Files

You can find ROSBAG files in `/home/tejas/dev_ws/output/`:
- `static_landing_update.bag`
- `moving-2-landing.bag`
- `moving-3-landing.bag`
- `moving-4-landing.bag`
- `stationary-2-landing.bag`
- `stationary-3-landing.bag`
- `stationary-4-landing.bag`

## Examples

### Plot single topic
```python
plotter = ROSBagPlotter("output/static_landing_update.bag")
plotter.plot_topic("/drone/pose", title="Drone Position", y_label="Position (m)")
plotter.show()
```

### Plot multiple topics on same axis
```python
plotter = ROSBagPlotter("output/static_landing_update.bag")
plotter.plot_multiple_topics(
    ["/drone/pose", "/camera/pose"],
    title="Position Comparison",
    y_label="Position (m)"
)
plotter.show()
```

### Plot organized subplots
```python
plotter = ROSBagPlotter("output/static_landing_update.bag", figsize=(14, 10))
plotter.plot_subplots({
    "Position": ["/drone/pose/x", "/drone/pose/y", "/drone/pose/z"],
    "Velocity": ["/drone/vel/x", "/drone/vel/y", "/drone/vel/z"],
    "Attitude": ["/drone/roll", "/drone/pitch", "/drone/yaw"]
})
plotter.show()
```

## Features in Detail

### Grid
Grid is **enabled by default** on all plots for better readability. Disable with `grid=False`:
```python
plotter.plot_topic("/topic", grid=False)
```

### Customization
```python
plotter.plot_topic(
    "/topic_name",
    title="Custom Title",
    x_label="Time (seconds)",
    y_label="Measurement",
    color='r',           # Red line
    linewidth=2.0,       # Thicker line
    alpha=0.8,           # Slight transparency
    grid=True            # Grid enabled
)
```

### Saving Plots
```python
plotter.plot_topic("/drone/pose")
plotter.save("figures/drone_pose.png", dpi=300)  # High resolution
```

## API Reference

### ROSBagPlotter Class

**Methods:**

- `get_topics()` - List all topics in the ROSBAG file
- `extract_topic_data(topic)` - Get raw timestamps and data values as numpy arrays
- `plot_topic(topic, ...)` - Plot single topic
- `plot_multiple_topics(topics, ...)` - Plot multiple topics on same axis
- `plot_subplots(topic_groups, ...)` - Organize topics in subplots
- `show()` - Display the plot
- `save(path, dpi=300)` - Save plot to file
- `close()` - Clean up resources

## Notes

- Timestamps are automatically normalized to start from 0 seconds
- Data values are extracted automatically from message `.data`, `.x` fields, or first numeric field
- Colors are automatically assigned for multi-topic plots
- All plots include legends for easy identification
