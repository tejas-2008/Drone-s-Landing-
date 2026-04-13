#!/usr/bin/env python3
"""
ROSBAG Time Series Plotter - Command-line utility for plotting topics from ROSBAG files.

Usage:
    python3 main.py <bagfile> <topic1> [topic2] [topic3] ...
    python3 main.py --list <bagfile>  # List all available topics
    python3 main.py --help             # Show this help message
"""

import sys
import os
import argparse

# Add utils directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.plotter import ROSBagPlotter


def list_topics(bagfile_path):
    """List all available topics in a ROSBAG file."""
    print(f"\n{'='*60}")
    print(f"Topics in ROSBAG file: {bagfile_path}")
    print('='*60)
    
    try:
        plotter = ROSBagPlotter(bagfile_path, sample_rate=1)
        topics = plotter.get_topics()
        
        print(f"\nFound {len(topics)} topics:\n")
        for idx, topic in enumerate(sorted(topics), 1):
            fields = plotter.get_topic_fields(topic)
            fields_str = f" {fields}" if fields else ""
            print(f"  {idx:2d}. {topic}{fields_str}")
        
        plotter.close()
        return topics
    except Exception as e:
        print(f"Error: {e}")
        return []


def plot_topics(bagfile_path, topics_to_plot, fields_to_plot=None, save_path=None, sample_rate=None):
    """Plot one or more topics from a ROSBAG file."""
    if not topics_to_plot:
        print("Error: No topics specified for plotting")
        return
    
    try:
        plotter = ROSBagPlotter(bagfile_path, figsize=(14, 7), sample_rate=sample_rate)
        
        if len(topics_to_plot) == 1:
            # Plot single topic
            topic = topics_to_plot[0]
            field = fields_to_plot[0] if fields_to_plot else None
            field_str = f":{field}" if field else ""
            print(f"\nPlotting single topic: {topic}{field_str}")
            plotter.plot_topic(
                topic,
                field=field,
                title=f"Feature Error ",
                y_label="Value (m)", xticks= 20, yticks=40, grid_linewidth=1.0
            )
        else:
            # Plot multiple topics
            print(f"\nPlotting {len(topics_to_plot)} topics:")
            for idx, topic in enumerate(topics_to_plot):
                field = fields_to_plot[idx] if fields_to_plot and idx < len(fields_to_plot) else None
                field_str = f":{field}" if field else ""
                print(f"  ✓ {topic}{field_str}")
            
            plotter.plot_multiple_topics(
                topics_to_plot,
                fields=fields_to_plot,
                title=f"Pixel Errors",
                y_label="Value (px)", xticks= 20, yticks=40, grid_linewidth=0.8
            )
        
        if save_path:
            plotter.save(save_path, dpi=300)
            print(f"\n✓ Plot saved to: {save_path}")
        
        print("\nShowing plot...")
        plotter.show()
        plotter.close()
    
    except Exception as e:
        print(f"Error: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Plot time series data from ROSBAG files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # List all topics and their fields in a bagfile
  python3 main.py --list output/static_landing_update.bag
  
  # Plot a single topic (auto-detect field)
  python3 main.py output/static_landing_update.bag /drone/pose
  
  # Plot a specific field from a topic
  python3 main.py output/static_landing_update.bag /platform_ibvs/diagnostics/error_norms:x
  
  # Plot multiple topics with different fields
  python3 main.py output/static_landing_update.bag /topic1:x /topic2:y /topic3
  
  # Plot multiple topics and save to file
  python3 main.py output/static_landing_update.bag /topic1:x /topic2:y --save comparison.png
        """
    )
    
    parser.add_argument(
        'bagfile',
        nargs='?',
        help='Path to ROSBAG file'
    )
    
    parser.add_argument(
        'topics',
        nargs='*',
        help='Topics to plot (format: /topic_name or /topic_name:field)'
    )
    
    parser.add_argument(
        '--list',
        action='store_true',
        help='List all available topics and their fields in the bagfile'
    )
    
    parser.add_argument(
        '--save',
        type=str,
        help='Save plot to file (e.g., --save output.png)'
    )
    
    parser.add_argument(
        '--sample-rate',
        type=float,
        default=None,
        help='Sampling rate in Hz for downsampling (e.g., --sample-rate 10)'
    )
    
    args = parser.parse_args()
    
    # Validate bagfile argument
    if not args.bagfile:
        parser.print_help()
        print("\nError: ROSBAG file path is required")
        sys.exit(1)
    
    if not os.path.exists(args.bagfile):
        print(f"Error: ROSBAG file not found: {args.bagfile}")
        print("\nAvailable files in ./output/:")
        output_dir = os.path.join(os.path.dirname(__file__), '..', 'output')
        if os.path.exists(output_dir):
            for f in os.listdir(output_dir):
                if f.endswith('.bag'):
                    print(f"  - {f}")
        sys.exit(1)
    
    # Handle --list flag
    if args.list:
        list_topics(args.bagfile)
        sys.exit(0)
    
    # Parse topics and fields from input
    topics_to_plot = []
    fields_to_plot = []
    
    for topic_spec in args.topics:
        if ':' in topic_spec:
            # Format: /topic_name:field
            topic, field = topic_spec.rsplit(':', 1)
            topics_to_plot.append(topic)
            fields_to_plot.append(field)
        else:
            # Format: /topic_name
            topics_to_plot.append(topic_spec)
            fields_to_plot.append(None)
    
    # If topics provided, plot them
    if topics_to_plot:
        plot_topics(args.bagfile, topics_to_plot, fields_to_plot if any(f for f in fields_to_plot) else None, save_path=args.save, sample_rate=args.sample_rate)
    else:
        # If no topics specified and not using --list, list topics
        print("No topics specified. Use --list to see available topics.\n")
        list_topics(args.bagfile)


if __name__ == "__main__":
    main()
