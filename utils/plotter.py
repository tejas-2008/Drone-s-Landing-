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
    
    def __init__(self, bagfile_path: str, figsize: Tuple[int, int] = (14, 7), sample_rate: Optional[float] = None,
                 label_fontsize: int = 42, tick_fontsize: int = 35, legend_fontsize: int = 42,
                 default_linewidth: float = 2.6):
        """
        Initialize the ROSBagPlotter.

        Args:
            bagfile_path: Path to the ROSBAG file
            figsize: Figure size (width, height) in inches
            sample_rate: Desired output sample rate in Hz (e.g., 10 for 10 samples/sec).
                         None keeps all messages (no downsampling).
            label_fontsize: Font size for axis labels and title
            tick_fontsize: Font size for tick mark numbers on axes
            legend_fontsize: Font size for legend text
            default_linewidth: Default line width for all plotted series
        """
        if not os.path.exists(bagfile_path):
            raise FileNotFoundError(f"ROSBAG file not found: {bagfile_path}")

        self.bagfile_path = bagfile_path
        self.figsize = figsize
        self.sample_rate = sample_rate
        self.label_fontsize = label_fontsize
        self.tick_fontsize = tick_fontsize
        self.legend_fontsize = legend_fontsize
        self.default_linewidth = default_linewidth
        self.bag = rosbag.Bag(bagfile_path)
        self._data_cache = {}
        self.fig = None
        self.ax = None

        # Steel-blue + cyan anchor scheme (matches reference figure), extended with
        # harmonious complements so multi-series plots keep distinct, balanced colors.
        self.color_palette = [
            '#3B7BB8',  # steel blue       (anchor — matches reference dark line)
            '#36CEDB',  # bright cyan      (anchor — matches reference light line)
            '#E07A3C',  # warm orange      (warm complement for high contrast)
            '#7E57C2',  # soft violet
            '#2E8B57',  # sea green
            '#D81B60',  # magenta
            '#FFB74D',  # amber
            '#1F3D6B',  # deep navy
        ]

        # Global styling — only aesthetic tweaks, all relative sizes preserved
        plt.rcParams.update({
            'font.family': 'DejaVu Sans',
            'axes.linewidth': 1.4,
            'axes.labelweight': 'bold',
            'axes.titleweight': 'bold',
            'axes.edgecolor': '#2B2B2B',
            'axes.facecolor': '#FBFBFB',
            'axes.axisbelow': True,
            'figure.facecolor': 'white',
            'xtick.major.width': 1.7,
            'ytick.major.width': 1.7,
            'xtick.major.size': 9,
            'ytick.major.size': 9,
            'xtick.color': '#2B2B2B',
            'ytick.color': '#2B2B2B',
            'xtick.minor.visible': False,
            'ytick.minor.visible': False,
            'grid.color': '#9AA0A6',
            'legend.frameon': True,
            'legend.framealpha': 0.95,
            'legend.edgecolor': '#3A3A3A',
            'legend.facecolor': '#FFFFFF',
            'savefig.bbox': 'tight',
            'savefig.dpi': 300,
            'savefig.facecolor': 'white',
            'figure.autolayout': False,
            'lines.antialiased': True,
        })

    @staticmethod
    def _short_label(topic: str, field: Optional[str]) -> str:
        """Build a compact legend label: field if present, else the topic's last segment."""
        if field:
            return field
        return topic.rstrip('/').rsplit('/', 1)[-1] or topic

    @staticmethod
    def _topic_title(topic: str) -> str:
        """Convert a topic's last segment into a human-readable title.

        Examples:
            /platform_ibvs/diagnostics/pixel_error -> 'Pixel Error'
            /feature_error                         -> 'Feature Error'
            /platform_ibvs/diagnostics/error_norms -> 'Error Norms'
        """
        last = topic.rstrip('/').rsplit('/', 1)[-1] or topic
        return last.replace('_', ' ').replace('-', ' ').strip().title()

    @classmethod
    def derive_title(cls, topics: List[str]) -> str:
        """Derive a plot title from one or more topics.

        If all topics share the same last segment, use that. Otherwise, join the
        unique last-segment titles with ' / '.
        """
        if not topics:
            return "Time Series"
        titles = []
        seen = set()
        for t in topics:
            name = cls._topic_title(t)
            if name not in seen:
                seen.add(name)
                titles.append(name)
        return titles[0] if len(titles) == 1 else " / ".join(titles)

    def _style_legend(self, ax):
        """Apply a consistent, readable legend style."""
        leg = ax.legend(loc='best', fontsize=self.legend_fontsize,
                        frameon=True, fancybox=True, framealpha=0.95,
                        borderpad=0.6, labelspacing=0.4, handlelength=2.2,
                        facecolor='#FFFFFF')
        if leg is not None:
            leg.get_frame().set_linewidth(1.0)
            leg.get_frame().set_edgecolor("#3A3A3A")
        return leg
        
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
        color: Optional[str] = None,
        linewidth: Optional[float] = None,
        grid: bool = True,
        grid_linewidth: float = 0.9,
        xticks: Optional[int] = 8,
        yticks: Optional[int] = 8,
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

        if linewidth is None:
            linewidth = self.default_linewidth
        if color is None:
            color = self.color_palette[0]

        label = self._short_label(topic, field)
        self.ax.plot(timestamps, data, color=color, linewidth=linewidth, alpha=alpha, label=label,
                     solid_capstyle='round', solid_joinstyle='round')
        self.ax.set_xlabel(x_label, fontsize=self.label_fontsize, fontweight='bold')
        self.ax.set_ylabel(y_label, fontsize=self.label_fontsize, fontweight='bold')

        if title is None:
            title = f"Time Series: {topic}"
        self.ax.set_title(title, fontsize=self.label_fontsize + 2, fontweight='bold', pad=12)

        if grid:
            self.ax.minorticks_on()
            self.ax.grid(True, which='major', linestyle='--', alpha=0.40, linewidth=grid_linewidth, color="#000000")
            self.ax.grid(True, which='minor', linestyle=':', alpha=0.22, linewidth=grid_linewidth * 0.55, color="#000000")
            # finer sub-grid only — hide minor tick marks so it stays unannotated
            self.ax.tick_params(axis='both', which='minor', length=0)

        # Control ticks — keep counts low so labels stay readable when shrunk
        if xticks is not None:
            self.ax.locator_params(axis='x', nbins=xticks)
        if yticks is not None:
            self.ax.locator_params(axis='y', nbins=yticks)

        # Set tick label font sizes and rotation
        self.ax.tick_params(axis='both', which='major', labelsize=self.tick_fontsize, width=1.7, length=9)
        if tick_rotation != 0:
            self.ax.tick_params(axis='x', rotation=tick_rotation)
            self.ax.tick_params(axis='y', rotation=tick_rotation)

        # Thicker axis spines for clarity when scaled down
        for spine in self.ax.spines.values():
            spine.set_linewidth(1.4)

        # self._style_legend(self.ax)
        self.fig.tight_layout()
    
    def plot_multiple_topics(
        self,
        topics: List[str],
        fields: Optional[List[str]] = None,
        title: Optional[str] = None,
        x_label: str = "Time (s)",
        y_label: str = "Value",
        colors: Optional[List[str]] = None,
        grid: bool = True,
        grid_linewidth: float = 0.9,
        xticks: Optional[int] = 8,
        yticks: Optional[int] = 8,
        tick_rotation: int = 0,
        linewidth: Optional[float] = None
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

        if linewidth is None:
            linewidth = self.default_linewidth

        # Curated colorblind-safe palette (shared with single/subplot paths)
        if colors is None:
            base_palette = self.color_palette
            if len(topics) <= len(base_palette):
                colors = base_palette[:len(topics)]
            else:
                colors = plt.cm.viridis(np.linspace(0.05, 0.85, len(topics)))

        for idx, topic in enumerate(topics):
            field = fields[idx] if fields and idx < len(fields) else None
            timestamps, data = self.extract_topic_data(topic, field=field)
            color = colors[idx] if idx < len(colors) else self.color_palette[0]
            label = self._short_label(topic, field)
            self.ax.plot(timestamps, data, linewidth=linewidth, alpha=0.92, label=label, color=color,
                         solid_capstyle='round', solid_joinstyle='round')

        self.ax.set_xlabel(x_label, fontsize=self.label_fontsize, fontweight='bold')
        self.ax.set_ylabel(y_label, fontsize=self.label_fontsize, fontweight='bold')

        if title is None:
            title = f"Time Series Comparison ({len(topics)} topics)"
        self.ax.set_title(title, fontsize=self.label_fontsize + 2, fontweight='bold', pad=12)

        if grid:
            self.ax.minorticks_on()
            self.ax.grid(True, which='major', linestyle='--', alpha=0.40, linewidth=grid_linewidth, color="#000000")
            self.ax.grid(True, which='minor', linestyle=':', alpha=0.22, linewidth=grid_linewidth * 0.55, color="#000000")
            # finer sub-grid only — hide minor tick marks so it stays unannotated
            self.ax.tick_params(axis='both', which='minor', length=0)

        if xticks is not None:
            self.ax.locator_params(axis='x', nbins=xticks)
        if yticks is not None:
            self.ax.locator_params(axis='y', nbins=yticks)

        self.ax.tick_params(axis='both', which='major', labelsize=self.tick_fontsize, width=1.7, length=9)
        if tick_rotation != 0:
            self.ax.tick_params(axis='x', rotation=tick_rotation)
            self.ax.tick_params(axis='y', rotation=tick_rotation)

        for spine in self.ax.spines.values():
            spine.set_linewidth(1.4)

        self._style_legend(self.ax)
        self.fig.tight_layout()
    
    def plot_subplots(
        self,
        topic_groups: Dict[str, List[str]],
        field_groups: Optional[Dict[str, List[Optional[str]]]] = None,
        figsize: Optional[Tuple[int, int]] = None,
        grid: bool = True,
        grid_linewidth: float = 0.9,
        xticks: Optional[int] = 6,
        yticks: Optional[int] = 6,
        tick_rotation: int = 0,
        linewidth: Optional[float] = None
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
            # Scale figure size with number of subplots so each panel stays large enough
            n = len(topic_groups)
            cols_guess = int(np.ceil(np.sqrt(n)))
            rows_guess = int(np.ceil(n / cols_guess))
            figsize = (max(self.figsize[0], 7 * cols_guess),
                       max(self.figsize[1], 5 * rows_guess))

        if linewidth is None:
            linewidth = self.default_linewidth

        num_plots = len(topic_groups)
        rows = int(np.ceil(np.sqrt(num_plots)))
        cols = int(np.ceil(num_plots / rows))

        self.fig, axes = plt.subplots(rows, cols, figsize=figsize)

        if num_plots == 1:
            axes = [axes]
        else:
            axes = axes.flatten()

        base_palette = self.color_palette

        for idx, (title, topics) in enumerate(topic_groups.items()):
            ax = axes[idx]

            if len(topics) <= len(base_palette):
                colors = base_palette[:len(topics)]
            else:
                colors = plt.cm.viridis(np.linspace(0.05, 0.85, len(topics)))
            fields = field_groups.get(title) if field_groups else None

            for jdx, topic in enumerate(topics):
                field = fields[jdx] if fields and jdx < len(fields) else None
                timestamps, data = self.extract_topic_data(topic, field=field)
                label = self._short_label(topic, field)
                ax.plot(timestamps, data, linewidth=linewidth, alpha=0.92, label=label,
                        color=colors[jdx], solid_capstyle='round', solid_joinstyle='round')

            ax.set_xlabel("Time (s)", fontsize=self.label_fontsize, fontweight='bold')
            ax.set_ylabel("Value", fontsize=self.label_fontsize, fontweight='bold')
            ax.set_title(title, fontsize=self.label_fontsize + 1, fontweight='bold', pad=10)

            if grid:
                ax.minorticks_on()
                ax.grid(True, which='major', linestyle='--', alpha=0.40, linewidth=grid_linewidth, color='#9AA0A6')
                ax.grid(True, which='minor', linestyle=':', alpha=0.22, linewidth=grid_linewidth * 0.55, color='#9AA0A6')
                # finer sub-grid only — hide minor tick marks so it stays unannotated
                ax.tick_params(axis='both', which='minor', length=0)

            if xticks is not None:
                ax.locator_params(axis='x', nbins=xticks)
            if yticks is not None:
                ax.locator_params(axis='y', nbins=yticks)

            ax.tick_params(axis='both', which='major', labelsize=self.tick_fontsize, width=1.7, length=9)
            if tick_rotation != 0:
                ax.tick_params(axis='x', rotation=tick_rotation)
                ax.tick_params(axis='y', rotation=tick_rotation)

            for spine in ax.spines.values():
                spine.set_linewidth(1.4)

            leg = ax.legend(loc='best', fontsize=max(self.legend_fontsize - 2, 22),
                            frameon=True, fancybox=True, framealpha=0.95,
                            borderpad=0.5, labelspacing=0.35, handlelength=2.0,
                            facecolor='#FFFFFF')
            if leg is not None:
                leg.get_frame().set_linewidth(1.0)
                leg.get_frame().set_edgecolor('#3A3A3A')

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
