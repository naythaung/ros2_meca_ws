#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


IMAGE_TOPIC = "/image_raw"


def get_bag_files(input_dir):
    """Find all MCAP files in the input directory."""
    return sorted(Path(input_dir).glob("*.mcap"))


def extract_video(bag_path, output_path):
    print(f"\nProcessing: {bag_path.name}")

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="mcap",
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Check that the image topic exists
    topics = reader.get_all_topics_and_types()

    topic_types = {
        topic.name: topic.type
        for topic in topics
    }

    if IMAGE_TOPIC not in topic_types:
        print(f"  WARNING: {IMAGE_TOPIC} not found")
        return

    if topic_types[IMAGE_TOPIC] != "sensor_msgs/msg/Image":
        print(
            f"  WARNING: {IMAGE_TOPIC} has type "
            f"{topic_types[IMAGE_TOPIC]}"
        )
        return

    bridge = CvBridge()

    writer = None
    frame_count = 0

    first_timestamp = None
    previous_timestamp = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != IMAGE_TOPIC:
            continue

        msg = deserialize_message(
            data,
            Image
        )

        try:
            frame = bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except Exception as e:
            print(f"  WARNING: Could not decode frame: {e}")
            continue

        height, width = frame.shape[:2]

        # Calculate FPS from ROS timestamps
        if previous_timestamp is not None:
            dt = (timestamp - previous_timestamp) / 1e9

            if dt > 0:
                current_fps = 1.0 / dt

                # Initialize video writer using the first valid FPS
                if writer is None:
                    fps = max(1.0, min(current_fps, 120.0))

                    print(
                        f"  Resolution: {width}x{height}"
                    )
                    print(
                        f"  FPS: {fps:.2f}"
                    )

                    fourcc = cv2.VideoWriter_fourcc(
                        *"mp4v"
                    )

                    writer = cv2.VideoWriter(
                        str(output_path),
                        fourcc,
                        fps,
                        (width, height)
                    )

                    if not writer.isOpened():
                        raise RuntimeError(
                            f"Could not open video writer: {output_path}"
                        )

        previous_timestamp = timestamp

        if writer is not None:
            writer.write(frame)
            frame_count += 1

            if frame_count % 500 == 0:
                print(
                    f"  Frames written: {frame_count}"
                )

    if writer is not None:
        writer.release()

    if frame_count == 0:
        print("  No frames extracted.")
        return

    print(
        f"  DONE: {frame_count} frames -> "
        f"{output_path.name}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Extract /image_raw from ROS 2 MCAP bags to MP4"
    )

    parser.add_argument(
        "input_dir",
        help="Directory containing .mcap files"
    )

    parser.add_argument(
        "-o",
        "--output",
        default="videos",
        help="Output directory (default: videos)"
    )

    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output)

    if not input_dir.exists():
        raise FileNotFoundError(
            f"Input directory does not exist: {input_dir}"
        )

    output_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    bags = get_bag_files(input_dir)

    if not bags:
        print(
            f"No .mcap files found in {input_dir}"
        )
        return

    print(
        f"Found {len(bags)} MCAP file(s)"
    )

    for bag_path in bags:

        # Same name as the bag, but .mp4
        output_path = (
            output_dir /
            f"{bag_path.stem}.mp4"
        )

        # Don't redo existing videos
        if output_path.exists():
            print(
                f"\nSkipping {bag_path.name} "
                f"(already exists)"
            )
            continue

        try:
            extract_video(
                bag_path,
                output_path
            )

        except Exception as e:
            print(
                f"\nERROR processing "
                f"{bag_path.name}: {e}"
            )

    print("\nAll bags processed.")


if __name__ == "__main__":
    main()