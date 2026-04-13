#!/usr/bin/env python3
"""
ROSBAG Time Series Plotter Utility
Provides helper functions to extract and visualize time series data from ROSBAG files.

Usage:
    from rosbag_plotter import ROSBagPlotter
    
    plotter = ROSBagPlotter("path/to/bagfile.bag")
    plotter.plot_topic("/topic_name", y_label="Metric")
    plotter.show()
"""

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from typing import List, Dict, Tuple, Optional
import os


class ROSBagPlotter:
    """Helper class for plotting time series data from ROSBAG files."""
    
    def __init__(self, bagfile_path: str, figsize: Tuple[int, int] = (12, 6), sample_rate: Optional[float] = None,
                 label_fontsize: int = 17, tick_fontsize: int = 11):
        """
        Initialize the ROSBagPlotter.
        
        Args:
            bagfile_path: Path to the ROSBAG file
            figsize: Figure size (width, height) in inches
            sample_rate: Desired output sample rate in Hz (e.g., 10 for 10 samples/sec).
                         None keeps all messages (no downsampling).
            label_fontsize: Font size for axis labels and title
            tick_fontsize: Font size for tick mark numbers on axes
        """
        if not os.path.exists(bagfile_path):
            raise FileNotFoundError(f"ROSBAG file not found: {bagfile_path}")
        
        self.bagfile_path = bagfile_path
        self.figsize = figsize
        self.sample_rate = sample_rate
        self.label_fontsize = label_fontsize
        self.tick_fontsize = tick_fontsize
        self.bag = rosbag.Bag(bagfile_path)
        self._data_cache = {}
        self.fig = None
        self.ax = None
        
    def get_topics(self) -> List[str]:
        """Get list of all topics in the ROSBAG file."""
        topics = set()
        for topic, msg, t in self.bag.read_messages():
            topics.add(topic)
        return list(topics)
    
    def get_topic_fields(self, topic: str) -> List[str]:
        """
        Get available numeric fields in a topic's messages.
        
        Args:
            topic: Topic name
            
        Returns:
            List of numeric field names in the topic's messages
        """
        for msg_topic, msg, t in self.bag.read_messages(topics=[topic], limit=1):
            fields = []
            if hasattr(msg, '__slots__'):
                for field in msg.__slots__:
                    field_value = getattr(msg, field, None)
                    if isinstance(field_value, (int, float)):
                        fields.append(field)
            return fields
        return []

    def extract_topic_data(self, topic: str, field: Optional[str] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract time series data from a specific topic and optional field.
        
        Args:
            topic: Topic name to extract
            field: Optional specific field to extract (e.g., 'x', 'y', 'z', 'data')
            
        Returns:
            Tuple of (timestamps, data) as numpy arrays
        """
        cache_key = f"{topic}:{field}" if field else topic
        if cache_key in self._data_cache:
            return self._data_cache[cache_key]
        
        timestamps = []
        data_values = []
        
        for msg_topic, msg, t in self.bag.read_messages(topics=[topic]):
            timestamps.append(t.to_sec())
            
            if field:
                # Extract specific field
                if hasattr(msg, field):
                    field_value = getattr(msg, field, None)
                    if isinstance(field_value, (int, float)):
                        data_values.append(float(field_value))
                else:
                    raise ValueError(f"Field '{field}' not found in topic '{topic}'")
            else:
                # Auto-detect field
                if hasattr(msg, 'data'):
                    data_values.append(msg.data)
                elif hasattr(msg, 'x'):
                    data_values.append(msg.x)
                else:
                    # Try to extract first numeric field
                    found = False
                    if hasattr(msg, '__slots__'):
                        for field_name in msg.__slots__:
                            field_value = getattr(msg, field_name, None)
                            if isinstance(field_value, (int, float)):
                                data_values.append(float(field_value))
                                found = True
                                break
                    if not found:
                        raise ValueError(f"No numeric fields found in topic '{topic}'")
        
        if not timestamps:
            raise ValueError(f"No data found for topic: {topic}")
        
        # Normalize timestamps to start from 0
        timestamps = np.array(timestamps)
        timestamps = timestamps - timestamps[0]
        data_values = np.array(data_values)
        
        # Downsample to desired sample_rate if specified
        if self.sample_rate is not None and len(timestamps) > 1:
            dt = 1.0 / self.sample_rate
            indices = [0]
            last_t = timestamps[0]
            for i in range(1, len(timestamps)):
                if timestamps[i] - last_t >= dt:
                    indices.append(i)
                    last_t = timestamps[i]
            timestamps = timestamps[indices]
            data_values = data_values[indices]
        
        self._data_cache[cache_key] = (timestamps, data_values)
        return timestamps, data_values
    
    def plot_topic(
        self, 
        topic: str,
        field: Optional[str] = None,
        title: Optional[str] = None,
        x_label: str = "Time (s)",
        y_label: str = "Value",
        color: str = 'b',
        linewidth: float = 1.5,
        grid: bool = True,
        grid_linewidth: float = 1.0,
        xticks: Optional[int] = None,
        yticks: Optional[int] = None,
        tick_rotation: int = 0,
        alpha: float = 1.0
    ) -> None:
        """
        Plot a single topic as a time series.
        
        Args:
            topic: Topic name to plot
            field: Optional specific field to extract (e.g., 'x', 'y', 'z')
            title: Plot title (defaults to topic name)
            x_label: X-axis label
            y_label: Y-axis label
            color: Line color
            linewidth: Line width
            grid: Enable grid (default: True)
            grid_linewidth: Grid line width (default: 0.5)
            xticks: Number of x-axis ticks (None for auto)
            yticks: Number of y-axis ticks (None for auto)
            tick_rotation: Rotation angle for tick labels (default: 0)
            alpha: Line transparency
        """
        if self.fig is None:
            self.fig, self.ax = plt.subplots(figsize=self.figsize)
        
        timestamps, data = self.extract_topic_data(topic, field=field)
        
        label = f"{topic}:{field}" if field else topic
        self.ax.plot(timestamps, data, color=color, linewidth=linewidth, alpha=alpha, label=label)
        self.ax.set_xlabel(x_label, fontsize=self.label_fontsize)
        self.ax.set_ylabel(y_label, fontsize=self.label_fontsize)
        
        if title is None:
            title = f"Time Series: {topic}"
        self.ax.set_title(title, fontsize=self.label_fontsize + 2, fontweight='bold')
        
        if grid:
            self.ax.grid(True, linestyle='--', alpha=0.7, linewidth=grid_linewidth)
        
        # Control ticks
        if xticks is not None:
            self.ax.locator_params(axis='x', nbins=xticks)
        if yticks is not None:
            self.ax.locator_params(axis='y', nbins=yticks)
        
        # Set tick label font sizes and rotation
        self.ax.tick_params(axis='both', labelsize=self.tick_fontsize)
        if tick_rotation != 0:
            self.ax.tick_params(axis='x', rotation=tick_rotation)
            self.ax.tick_params(axis='y', rotation=tick_rotation)
        
        self.ax.legend(loc='best', fontsize=self.tick_fontsize)
    
    def plot_multiple_topics(
        self,
        topics: List[str],
        fields: Optional[List[str]] = None,
        title: Optional[str] = None,
        x_label: str = "Time (s)",
        y_label: str = "Value",
        colors: Optional[List[str]] = None,
        grid: bool = True,
        grid_linewidth: float = 0.5,
        xticks: Optional[int] = None,
        yticks: Optional[int] = None,
        tick_rotation: int = 0
    ) -> None:
        """
        Plot multiple topics on the same axis.
        
        Args:
            topics: List of topic names to plot
            fields: Optional list of fields (one per topic) or None
                   Example: [None, 'x', 'y'] to extract default field, x field, y field
            title: Plot title
            x_label: X-axis label
            y_label: Y-axis label
            colors: List of colors for each topic
            grid: Enable grid (default: True)
            grid_linewidth: Grid line width (default: 0.5)
            xticks: Number of x-axis ticks (None for auto)
            yticks: Number of y-axis ticks (None for auto)
            tick_rotation: Rotation angle for tick labels (default: 0)
        """
        if self.fig is None:
            self.fig, self.ax = plt.subplots(figsize=self.figsize)
        
        if colors is None:
            colors = plt.cm.tab10(np.linspace(0, 1, len(topics)))
        
        for idx, topic in enumerate(topics):
            field = fields[idx] if fields and idx < len(fields) else None
            timestamps, data = self.extract_topic_data(topic, field=field)
            color = colors[idx] if idx < len(colors) else 'b'
            label = f"{topic}:{field}" if field else topic
            self.ax.plot(timestamps, data, linewidth=1.5, alpha=0.8, label=label, color=color)
        
        self.ax.set_xlabel(x_label, fontsize=self.label_fontsize)
        self.ax.set_ylabel(y_label, fontsize=self.label_fontsize)
        
        if title is None:
            title = f"Time Series Comparison ({len(topics)} topics)"
        self.ax.set_title(title, fontsize=self.label_fontsize + 2, fontweight='bold')
        
        if grid:
            self.ax.grid(True, linestyle='--', alpha=0.7, linewidth=grid_linewidth)
        
        # Control ticks
        if xticks is not None:
            self.ax.locator_params(axis='x', nbins=xticks)
        if yticks is not None:
            self.ax.locator_params(axis='y', nbins=yticks)
        
        # Set tick label font sizes and rotation
        self.ax.tick_params(axis='both', labelsize=self.tick_fontsize)
        if tick_rotation != 0:
            self.ax.tick_params(axis='x', rotation=tick_rotation)
            self.ax.tick_params(axis='y', rotation=tick_rotation)
        
        self.ax.legend(loc='best', fontsize=self.tick_fontsize)
    
    def plot_subplots(
        self,
        topic_groups: Dict[str, List[str]],
        field_groups: Optional[Dict[str, List[Optional[str]]]] = None,
        figsize: Optional[Tuple[int, int]] = None,
        grid: bool = True,
        grid_linewidth: float = 0.5,
        xticks: Optional[int] = None,
        yticks: Optional[int] = None,
        tick_rotation: int = 0
    ) -> None:
        """
        Plot multiple topics in separate subplots.
        
        Args:
            topic_groups: Dictionary mapping subplot titles to lists of topics
            field_groups: Optional dictionary mapping subplot titles to lists of fields
                         (one field per topic in the same group)
            figsize: Figure size (uses default if not specified)
            grid: Enable grid for all subplots (default: True)
            grid_linewidth: Grid line width (default: 0.5)
            xticks: Number of x-axis ticks (None for auto)
            yticks: Number of y-axis ticks (None for auto)
            tick_rotation: Rotation angle for tick labels (default: 0)
        """
        if figsize is None:
            figsize = self.figsize
        
        num_plots = len(topic_groups)
        rows = int(np.ceil(np.sqrt(num_plots)))
        cols = int(np.ceil(num_plots / rows))
        
        self.fig, axes = plt.subplots(rows, cols, figsize=figsize)
        
        if num_plots == 1:
            axes = [axes]
        else:
            axes = axes.flatten()
        
        for idx, (title, topics) in enumerate(topic_groups.items()):
            ax = axes[idx]
            
            colors = plt.cm.tab10(np.linspace(0, 1, len(topics)))
            fields = field_groups.get(title) if field_groups else None
            
            for jdx, topic in enumerate(topics):
                field = fields[jdx] if fields and jdx < len(fields) else None
                timestamps, data = self.extract_topic_data(topic, field=field)
                label = f"{topic}:{field}" if field else topic
                ax.plot(timestamps, data, linewidth=1.5, alpha=0.8, label=label, color=colors[jdx])
            
            ax.set_xlabel("Time (s)", fontsize=self.label_fontsize)
            ax.set_ylabel("Value", fontsize=self.label_fontsize)
            ax.set_title(title, fontsize=self.label_fontsize + 1, fontweight='bold')
            
            if grid:
                ax.grid(True, linestyle='--', alpha=0.7, linewidth=grid_linewidth)
            
            # Control ticks
            if xticks is not None:
                ax.locator_params(axis='x', nbins=xticks)
            if yticks is not None:
                ax.locator_params(axis='y', nbins=yticks)
            
            # Set tick label font sizes and rotation
            ax.tick_params(axis='both', labelsize=self.tick_fontsize)
            if tick_rotation != 0:
                ax.tick_params(axis='x', rotation=tick_rotation)
                ax.tick_params(axis='y', rotation=tick_rotation)
            
            ax.legend(loc='best', fontsize=self.tick_fontsize - 2)
        
        # Hide extra subplots
        for idx in range(len(topic_groups), len(axes)):
            axes[idx].set_visible(False)
        
        self.fig.tight_layout()
    
    def show(self) -> None:
        """Display the plot."""
        if self.fig is not None:
            plt.show()
    
    def save(self, output_path: str, dpi: int = 300) -> None:
        """
        Save the current plot to a file.
        
        Args:
            output_path: Path to save the figure
            dpi: Resolution in dots per inch
        """
        if self.fig is not None:
            self.fig.savefig(output_path, dpi=dpi, bbox_inches='tight')
            print(f"Plot saved to: {output_path}")
    
    def close(self) -> None:
        """Close the plot and cleanup."""
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
            self.ax = None
    
    def __del__(self):
        """Cleanup resources."""
        self.bag.close()


# Example usage function
def example_usage():
    """
    Example demonstrating how to use the ROSBagPlotter utility.
    """
    # Find a ROSBAG file in the output directory
    bagfile = "/home/tejas/dev_ws/output/static_landing_update.bag"
    
    try:
        plotter = ROSBagPlotter(bagfile)
        
        # Print available topics
        print("Available topics:")
        for topic in plotter.get_topics():
            print(f"  {topic}")
        
        # Example 1: Plot single topic
        # plotter.plot_topic("/camera/pose", title="Camera Pose Over Time", y_label="Pose (m)")
        # plotter.show()
        
        # Example 2: Plot multiple topics
        # plotter.plot_multiple_topics(
        #     ["/topic1", "/topic2"],
        #     title="Comparison of Topics",
        #     y_label="Value"
        # )
        # plotter.show()
        
        # Example 3: Plot subplots
        # topic_groups = {
        #     "Position": ["/camera/pose/x", "/camera/pose/y"],
        #     "Velocity": ["/camera/vel/x", "/camera/vel/y"]
        # }
        # plotter.plot_subplots(topic_groups)
        # plotter.show()
        
        plotter.close()
    
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    example_usage()
