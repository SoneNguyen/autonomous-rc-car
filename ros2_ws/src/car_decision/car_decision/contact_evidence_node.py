"""Normalize collision pairs, excluding ground-only contact from collision status."""

from __future__ import annotations

from ros_gz_interfaces.msg import Contacts
from rclpy.qos import qos_profile_sensor_data

from .evidence_common import message_stamp_ns
from .evidence_node_base import EvidenceNode, spin_node


def _is_ground(name: str) -> bool:
    value = name.lower()
    return any(token in value for token in ("ground", "road", "asphalt"))


def normalize_contacts(contacts, obstacle_ids: set[str]) -> dict:
    pairs = []
    for contact in contacts:
        first = str(contact.collision1.name)
        second = str(contact.collision2.name)
        ground = _is_ground(first) or _is_ground(second)
        allowlisted = any(entity in first or entity in second for entity in obstacle_ids)
        pairs.append({
            "collision_a": first,
            "collision_b": second,
            "ground": ground,
            "allowlisted": allowlisted,
        })
    return {
        "contacts": pairs,
        "ground_excluded": all(item["ground"] for item in pairs),
        "non_ground_collision": any(not item["ground"] for item in pairs),
    }


class ContactEvidenceNode(EvidenceNode):
    def __init__(self) -> None:
        super().__init__("contact", "/contacts", "ros_gz_interfaces/msg/Contacts")
        self._obstacle_ids = {
            str(item["entity_id"]) for item in self.scenario.get("obstacles", [])
        }
        self._last_contact_received_ns: int | None = None
        self._subscription = self.create_subscription(
            Contacts, "/contacts", self._on_contacts, qos_profile_sensor_data
        )

    def _empty_payload(self):
        return {"contacts": [], "ground_excluded": True, "non_ground_collision": False}

    def _on_contacts(self, message: Contacts) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.is_faulted(now_ns):
            return
        self._last_contact_received_ns = now_ns
        self.record_sample(
            normalize_contacts(message.contacts, self._obstacle_ids),
            message_stamp_ns(message.header, now_ns),
            message.header.frame_id,
        )

    def _publish_status(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns > 0 and self.get_publishers_info_by_topic("/contacts"):
            # Gazebo's Contact system is edge/event based and is silent when there
            # is no collision. The bridged publisher plus a progressing sim clock
            # is the liveness signal; a recent event remains visible for one
            # configured timeout, then the state becomes a fresh empty contact set.
            if self._last_contact_received_ns is None or (
                now_ns - self._last_contact_received_ns
            ) > int(self._timeout_s * 1_000_000_000):
                self._payload = self._empty_payload()
                self._sample_ns = now_ns
                self.sample_sequence += 1
                self._frame_id = "base_link"
        super()._publish_status()


def main(args=None) -> None:
    spin_node(ContactEvidenceNode, args)
