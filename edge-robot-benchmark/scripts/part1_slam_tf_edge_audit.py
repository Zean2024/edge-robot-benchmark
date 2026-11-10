#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


def format_gid(gid):
    return ".".join(f"{int(value):02x}" for value in gid)


def normalize_frame(frame):
    return frame.lstrip("/")


class TFEdgeAuditor(Node):
    def __init__(self):
        super().__init__("part1_slam_tf_edge_auditor")

        self.edges = {}
        self.parents_by_child = {}
        self.total_messages = {"/tf": 0, "/tf_static": 0}

        dynamic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            TFMessage, "/tf", self.dynamic_callback, dynamic_qos
        )
        self.create_subscription(
            TFMessage, "/tf_static", self.static_callback, static_qos
        )

    def dynamic_callback(self, message, message_info):
        self.record_message(message, message_info, "/tf")

    def static_callback(self, message, message_info):
        self.record_message(message, message_info, "/tf_static")

    def record_message(self, message, message_info, topic):
        gid = format_gid(message_info.publisher_gid)
        self.total_messages[topic] += 1

        for transform in message.transforms:
            parent = normalize_frame(transform.header.frame_id)
            child = normalize_frame(transform.child_frame_id)

            if not parent or not child:
                continue

            key = (parent, child)
            edge = self.edges.setdefault(
                key,
                {"count": 0, "publishers": {}},
            )
            edge["count"] += 1

            publisher = edge["publishers"].setdefault(
                gid,
                {"count": 0, "topics": set()},
            )
            publisher["count"] += 1
            publisher["topics"].add(topic)

            self.parents_by_child.setdefault(child, set()).add(parent)

    def get_endpoint_names(self):
        names = {}

        for topic in ("/tf", "/tf_static"):
            try:
                endpoints = self.get_publishers_info_by_topic(topic)
            except Exception:
                continue

            for endpoint in endpoints:
                gid = format_gid(endpoint.endpoint_gid)
                names[gid] = (
                    f"{endpoint.node_namespace}{endpoint.node_name}"
                    f" on {topic}"
                )

        return names

    def render_report(self):
        names = self.get_endpoint_names()
        lines = [
            "TF EDGE OWNERSHIP AUDIT",
            f"Messages observed on /tf: {self.total_messages['/tf']}",
            f"Messages observed on /tf_static: "
            f"{self.total_messages['/tf_static']}",
            f"Unique parent-child edges: {len(self.edges)}",
            "",
        ]

        duplicate_edges = []
        for (parent, child), data in sorted(self.edges.items()):
            publishers = data["publishers"]
            lines.append(
                f"EDGE {parent} -> {child} | "
                f"messages={data['count']} | "
                f"publisher_endpoints={len(publishers)}"
            )

            for gid, publisher in sorted(publishers.items()):
                name = names.get(gid, "publisher name not resolved")
                topics = ",".join(sorted(publisher["topics"]))
                lines.append(
                    f"  GID={gid} | node={name} | "
                    f"topics={topics} | messages={publisher['count']}"
                )

            if len(publishers) > 1:
                duplicate_edges.append((parent, child, len(publishers)))

        multiple_parent_children = {
            child: sorted(parents)
            for child, parents in self.parents_by_child.items()
            if len(parents) > 1
        }

        lines.append("")
        lines.append("DUPLICATE EDGE PUBLISHERS")

        if duplicate_edges:
            for parent, child, count in duplicate_edges:
                lines.append(
                    f"FAIL: {parent} -> {child} observed from "
                    f"{count} publisher endpoints."
                )
        else:
            lines.append("None observed.")

        lines.append("")
        lines.append("CHILD FRAMES WITH MULTIPLE PARENTS")

        if multiple_parent_children:
            for child, parents in sorted(multiple_parent_children.items()):
                lines.append(
                    f"FAIL: {child} has observed parents: "
                    f"{', '.join(parents)}"
                )
        else:
            lines.append("None observed.")

        has_messages = any(self.total_messages.values())
        failed = bool(
            duplicate_edges or multiple_parent_children or not has_messages
        )

        lines.append("")
        if not has_messages:
            lines.append("AUDIT_RESULT: FAIL (no TF messages observed)")
        elif failed:
            lines.append("AUDIT_RESULT: FAIL")
        else:
            lines.append("AUDIT_RESULT: PASS")
            lines.append(
                "No duplicate edge publishers or multiple-parent frames "
                "were observed during this sampling window."
            )

        return "\n".join(lines), 1 if failed else 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = TFEdgeAuditor()

    deadline = time.monotonic() + max(args.duration, 1.0)

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        report, result = node.render_report()
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(report + "\n", encoding="utf-8")
        print(report)
        return result
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
