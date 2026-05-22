#!/usr/bin/env python3
"""mcm09_enumerator.py

One-shot enumerator that scans an LCI Godot scene (.tscn) and emits the
artifacts a ControlLogix programmer needs to mirror every per-tag I/O the
sim subscribes to through the two big-tag buffers used by the oip-comms
"One-Big-Tag" refactor:

    OIP_READ   REAL[N_R]   <- PLC writes; LCI reads
    OIP_WRITE  REAL[N_W]   <- LCI writes; PLC reads

Each REAL buffer has a parallel STRING[N] holding the per-slot logical tag
name (e.g. "NCP1_1_VFD:I.Running"). LCI never sees these arrays -- the DLL
reads the STRING arrays on sim start, builds a name -> slot map, and
silently re-routes every register_tag call through the buffer.

All-REAL routing: integer-typed source tags (DINT/INT/SINT) ride the REAL
buffer with Logix5000's implicit REAL<->DINT coercion on assignment.
Lossless for values within +/-2^24 (16.7M). PowerFlex fault codes
(<= 0x00FFFFFF) and StackLight bitmasks (0..255) both fit comfortably.

Outputs (written to --out, default ./out):

    OIP_READ_NAMES.csv        index,name,datatype,source_node,source_property
    OIP_WRITE_NAMES.csv

    OIP_SETUP.txt             Single Structured-Text blob. Lines 1..N_names are
                              `OIP_*_NAMES[i] := 'tag.name';` assignments — put
                              these in a one-shot init routine gated on S:FS.
                              The remainder is the cyclic mirror block:
                              IF/THEN/ELSE for BOOL <-> REAL buffer slots,
                              `:=` for everything else.

    OIP_SUMMARY.txt           Counts, suggested array sizes, unmapped devices.

Usage:
    python tools/mcm09_enumerator.py \
        --scene C:/Users/ilia.gurielidze/Documents/GitHub/Open-Industry-Project-LCI/CDW5_MCM09.tscn \
        --out tools/out

Conventions for the mirror blob (Logix5000 Structured Text):

    BOOL (lci_read):    IF <tag> THEN OIP_READ[idx] := 1.0; ELSE OIP_READ[idx] := 0.0; END_IF;
    REAL/INT (lci_read): OIP_READ[idx] := <tag>;
    BOOL (lci_write):   <tag> := (OIP_WRITE[idx] > 0.5);
    REAL/INT (lci_write): <tag> := OIP_WRITE[idx];

Bidirectional fields (rare -- Gantry/AGV/SixAxisRobot `vacuum_tag`/`lift_tag`)
are emitted in BOTH the read and write tables so neither direction is lost.

Devices whose .tscn instance lacks an `enable_comms` line are still
included -- Godot only writes property OVERRIDES, so template-default-true
devices (StackLight, PushButton variants) silently have comms enabled.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


# ---------------------------------------------------------------------------
# Device schema
# ---------------------------------------------------------------------------
#
# For each LCI device class we know:
#   - which @export *_tag_name fields exist on it
#   - the PLC datatype for each (drives BUFFER routing: REAL/BOOL -> REAL
#     buffer, DINT/INT/SINT -> DINT buffer)
#   - the direction (lci_read / lci_write / bidir)
#
# Extracted by inspection of the GDScript file backing each device. See
# Open-Industry-Project-LCI/src/<Device>/<device>.gd.
#
# `dir` semantics:
#   lci_write  = sim PUSHES value to PLC (sim calls write_*); slot lives in
#                OIP_WRITE buffer; PLC ladder consumes from buffer.
#   lci_read   = sim PULLS value from PLC (sim calls read_*); slot lives in
#                OIP_READ buffer; PLC ladder writes into buffer.
#   bidir      = sim both reads & writes the same tag at different times.
#                Emit slots in both buffers (different idx).

DeviceField = tuple[str, str, str]  # (field_name, datatype, dir)

DEVICE_SCHEMA: dict[str, list[DeviceField]] = {
    # ---- Allen-Bradley PowerFlex VFD simulator (apf.gd) ----
    "APF": [
        ("connection_faulted_tag_name",  "BOOL", "lci_write"),
        ("keypad_hand_mode_tag_name",    "BOOL", "lci_write"),
        ("disconnect_closed_tag_name",   "BOOL", "lci_write"),
        ("fault_tag_name",               "BOOL", "lci_write"),
        ("running_tag_name",             "BOOL", "lci_write"),
        ("safe_torque_enabled_tag_name", "BOOL", "lci_write"),
        ("output_current_tag_name",      "REAL", "lci_write"),
        ("output_voltage_tag_name",      "REAL", "lci_write"),
        ("velocity_tag_name",            "REAL", "lci_write"),
        ("trip_fault_code_tag_name",     "DINT", "lci_write"),
        ("stop_tag_name",                "BOOL", "lci_read"),
        ("start_tag_name",               "BOOL", "lci_read"),
        ("direction_cmd_0_tag_name",     "BOOL", "lci_read"),
        ("direction_cmd_1_tag_name",     "BOOL", "lci_read"),
        ("commanded_velocity_tag_name",  "REAL", "lci_read"),
        ("clear_fault_tag_name",         "BOOL", "lci_read"),
        ("dynamic_accel_time_tag_name",  "REAL", "lci_read"),
        ("dynamic_decel_time_tag_name",  "REAL", "lci_read"),
    ],
    # ---- Motor Control Module panel (mcm.gd) ----
    "MCM": [
        ("start_pb_tag_name",                          "BOOL", "lci_write"),
        ("stop_pb_tag_name",                           "BOOL", "lci_write"),
        ("motor_fault_reset_pb_tag_name",              "BOOL", "lci_write"),
        ("jam_restart_pb_tag_name",                    "BOOL", "lci_write"),
        ("low_air_pressure_reset_pb_tag_name",         "BOOL", "lci_write"),
        ("power_branch_fault_reset_pb_tag_name",       "BOOL", "lci_write"),
        ("communication_fault_reset_pb_tag_name",      "BOOL", "lci_write"),
        ("estop_ch1_tag_name",                         "BOOL", "lci_write"),
        ("estop_ch2_tag_name",                         "BOOL", "lci_write"),
        ("ups_fault_tag_name",                         "BOOL", "lci_write"),
        ("on_ups_tag_name",                            "BOOL", "lci_write"),
        ("ups_low_tag_name",                           "BOOL", "lci_write"),
        ("nat_switch_fault_tag_name",                  "BOOL", "lci_write"),
        ("fire_relay_tag_name",                        "BOOL", "lci_write"),
        ("start_lt_tag_name",                          "BOOL", "lci_read"),
        ("motor_fault_reset_lt_tag_name",              "BOOL", "lci_read"),
        ("jam_restart_lt_tag_name",                    "BOOL", "lci_read"),
        ("low_air_pressure_reset_lt_tag_name",         "BOOL", "lci_read"),
        ("power_branch_fault_reset_lt_tag_name",       "BOOL", "lci_read"),
        ("communication_fault_reset_lt_tag_name",      "BOOL", "lci_read"),
        ("estop_actuated_lt_tag_name",                 "BOOL", "lci_read"),
    ],
    # ---- Auxiliary disconnect switch (aux_disconnect.gd) ----
    "AuxDisconnect": [
        ("aux_disconnect_closed_tag_name", "BOOL", "lci_write"),
    ],
    # ---- Emergency Pull Cord, dual channel (epc.gd) ----
    "EPC": [
        ("channel_1_tag_name", "BOOL", "lci_write"),
        ("channel_2_tag_name", "BOOL", "lci_write"),
    ],
    # ---- Stack light / beacon (stack_light.gd) ----
    # PLC tag is a DINT bitmask. LCI reads the low byte via read_uint8(),
    # but we route through the DINT buffer to keep the full bit pattern.
    "StackLight": [
        ("tag_name", "DINT", "lci_read"),
    ],
    # ---- Laser distance sensor (laser_sensor.gd) ----
    "LaserSensor": [
        ("tag_name",        "REAL", "lci_write"),
        ("output_tag_name", "BOOL", "lci_write"),
    ],
    # ---- Diffuse photo-eye sensor (diffuse_sensor.gd) ----
    "DiffuseSensor": [
        ("tag_name", "BOOL", "lci_write"),
    ],
    # ---- Push button + optional lamp (push_button.gd) ----
    "PushButton": [
        ("pushbutton_tag_name", "BOOL", "lci_write"),
        ("lamp_tag_name",       "BOOL", "lci_read"),
    ],
    # ---- Belt conveyors (belt_conveyor.gd + variants) ----
    "BeltConveyor": [
        ("speed_tag_name",   "REAL", "lci_read"),
        ("running_tag_name", "BOOL", "lci_write"),
    ],
    "CurvedBeltConveyor": [
        ("speed_tag_name",   "REAL", "lci_read"),
        ("running_tag_name", "BOOL", "lci_write"),
    ],
    "BeltSpurConveyor": [
        ("speed_tag_name",   "REAL", "lci_read"),
        ("running_tag_name", "BOOL", "lci_write"),
    ],
    # ---- Roller conveyors ----
    "RollerConveyor": [
        ("speed_tag_name",   "REAL", "lci_read"),
        ("running_tag_name", "BOOL", "lci_write"),
    ],
    "CurvedRollerConveyor": [
        ("speed_tag_name",   "REAL", "lci_read"),
        ("running_tag_name", "BOOL", "lci_write"),
    ],
    "RollerSpurConveyor": [
        ("speed_tag_name",   "REAL", "lci_read"),
        ("running_tag_name", "BOOL", "lci_write"),
    ],
    # ---- Diverter (diverter.gd) ----
    "Diverter": [
        ("tag_name", "BOOL", "lci_read"),
    ],
    # ---- Blade-stop popup (blade_stop.gd) ----
    "BladeStop": [
        ("tag_name", "BOOL", "bidir"),
    ],
    # ---- Gantry (gantry.gd) ----
    "Gantry": [
        ("command_tag", "INT",  "lci_read"),
        ("execute_tag", "BOOL", "lci_read"),
        ("done_tag",    "BOOL", "lci_write"),
        ("vacuum_tag",  "BOOL", "bidir"),
    ],
    # ---- Chain transfer (chain_transfer.gd) ----
    "ChainTransfer": [
        ("speed_tag_name", "REAL", "lci_read"),
        ("popup_tag_name", "BOOL", "lci_read"),
    ],
    # ---- AGV (agv.gd) ----
    "AGV": [
        ("command_tag", "INT",  "lci_read"),
        ("execute_tag", "BOOL", "lci_read"),
        ("done_tag",    "BOOL", "lci_write"),
        ("lift_tag",    "BOOL", "bidir"),
    ],
    # ---- 6-axis robot (six_axis_robot.gd) ----
    "SixAxisRobot": [
        ("command_tag", "INT",  "lci_read"),
        ("execute_tag", "BOOL", "lci_read"),
        ("done_tag",    "BOOL", "lci_write"),
        ("vacuum_tag",  "BOOL", "bidir"),
    ],
    # ---- Color sensor (color_sensor.gd) ----
    "ColorSensor": [
        ("tag_name",        "DINT", "lci_write"),
        ("output_tag_name", "BOOL", "lci_write"),
    ],
}

# Map .tscn / .gd basename -> device class key above. Filenames here are the
# basename of the `path=` field of an [ext_resource] line.
RESOURCE_TO_DEVICE: dict[str, str] = {
    # Packed scenes
    "APF.tscn":                   "APF",
    "MCM.tscn":                   "MCM",
    "AUX_Disconnect.tscn":        "AuxDisconnect",
    "EPC.tscn":                   "EPC",
    "StackLight.tscn":            "StackLight",
    "LaserSensor.tscn":           "LaserSensor",
    "DiffuseSensor.tscn":         "DiffuseSensor",
    "PushButton.tscn":            "PushButton",
    "BeltConveyor.tscn":          "BeltConveyor",
    "CurvedBeltConveyor.tscn":    "CurvedBeltConveyor",
    "BeltSpurConveyor.tscn":      "BeltSpurConveyor",
    "RollerConveyor.tscn":        "RollerConveyor",
    "CurvedRollerConveyor.tscn":  "CurvedRollerConveyor",
    "RollerSpurConveyor.tscn":    "RollerSpurConveyor",
    "Diverter.tscn":              "Diverter",
    "BladeStop.tscn":             "BladeStop",
    "Gantry.tscn":                "Gantry",
    "ChainTransfer.tscn":         "ChainTransfer",
    "AGV.tscn":                   "AGV",
    "SixAxisRobot.tscn":          "SixAxisRobot",
    "ColorSensor.tscn":           "ColorSensor",
    # Bare scripts (rare)
    "belt_conveyor.gd":           "BeltConveyor",
    "curved_belt_conveyor.gd":    "CurvedBeltConveyor",
    "belt_spur_conveyor.gd":      "BeltSpurConveyor",
    "roller_conveyor.gd":         "RollerConveyor",
    "curved_roller_conveyor.gd":  "CurvedRollerConveyor",
    "roller_spur_conveyor.gd":    "RollerSpurConveyor",
}


# ---------------------------------------------------------------------------
# .tscn parsing
# ---------------------------------------------------------------------------

# Lines we care about in a .tscn:
#
#   [ext_resource type="..." uid="..." path="res://..." id="..."]
#   [node name="X" parent="..." instance=ExtResource("...")]
#   <prop_name> = <value>            # e.g. enable_comms = true
#                                          tag_prefix = "UL20_20_VFD"
#                                          start_tag_name = "UL20_20_VFD:O.Start"
#
# Blank lines and other [...] blocks (sub_resource, etc.) we skip.

_EXT_RESOURCE_RE = re.compile(
    r'^\[ext_resource\s+.*?\bid\s*=\s*"([^"]+)"',
)
_EXT_RESOURCE_PATH_RE = re.compile(r'\bpath\s*=\s*"([^"]+)"')
_NODE_OPEN_RE = re.compile(
    r'^\[node\s+name\s*=\s*"([^"]+)"'
)
_NODE_INSTANCE_RE = re.compile(r'\binstance\s*=\s*ExtResource\(\s*"([^"]+)"\s*\)')
_PROP_RE = re.compile(r'^(\w+)\s*=\s*(.*)$')


@dataclass
class NodeEntry:
    name: str
    resource_id: str | None  # ExtResource("...") if instance, else None
    properties: dict[str, str] = field(default_factory=dict)


def parse_tscn(scene_path: Path) -> tuple[dict[str, str], list[NodeEntry]]:
    """Return (ext_resource_id_to_basename, list of nodes with `instance=` set).

    Sub-resources and [editable]/[connection] blocks are skipped. Properties
    are kept as raw text-after-`=` (we only need the string-typed ones, so
    quoting is left as-is for downstream parsing).
    """
    ext_resources: dict[str, str] = {}  # id -> path basename
    nodes: list[NodeEntry] = []
    current: NodeEntry | None = None

    with scene_path.open(encoding="utf-8") as fh:
        for raw in fh:
            line = raw.rstrip("\n")
            if line.startswith("[ext_resource"):
                m_id = _EXT_RESOURCE_RE.match(line)
                m_path = _EXT_RESOURCE_PATH_RE.search(line)
                if m_id and m_path:
                    ext_resources[m_id.group(1)] = os.path.basename(m_path.group(1))
                current = None
                continue
            if line.startswith("[node"):
                m_name = _NODE_OPEN_RE.match(line)
                if not m_name:
                    current = None
                    continue
                m_inst = _NODE_INSTANCE_RE.search(line)
                current = NodeEntry(
                    name=m_name.group(1),
                    resource_id=(m_inst.group(1) if m_inst else None),
                )
                nodes.append(current)
                continue
            if line.startswith("["):
                # sub_resource, connection, editable, etc.
                current = None
                continue
            if current is None:
                continue
            if not line.strip():
                continue
            m_prop = _PROP_RE.match(line)
            if m_prop:
                current.properties[m_prop.group(1)] = m_prop.group(2).strip()

    return ext_resources, nodes


def unquote(value: str) -> str:
    """Strip surrounding double-quotes from a .tscn string value, if any."""
    value = value.strip()
    if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
        return value[1:-1]
    return value


# ---------------------------------------------------------------------------
# Tag enumeration
# ---------------------------------------------------------------------------

@dataclass
class TagSlot:
    name: str           # full PLC tag name, e.g. "NCP1_1_VFD:I.Running"
    datatype: str       # BOOL | REAL | DINT | INT | SINT
    direction: str      # lci_read | lci_write
    source_node: str    # scene node name, e.g. "NCP1_1_VFD_APF"
    source_prop: str    # field on the node, e.g. "running_tag_name"

    @property
    def buffer(self) -> str:
        """Which big-tag buffer this slot belongs to.

        All-REAL routing: integer types share the REAL buffers with floats
        and bools. ControlLogix STRING/DINT auto-conversion on assignment
        handles the type adaptation. Lossless for values within +/-2^24
        (16.7M), which covers every integer source in this codebase
        (PowerFlex fault codes <= 0x00FFFFFF, StackLight bitmasks 0..255).
        """
        return "OIP_READ" if self.direction == "lci_read" else "OIP_WRITE"


def enumerate_tags(
    ext_resources: dict[str, str],
    nodes: Iterable[NodeEntry],
) -> tuple[list[TagSlot], list[str]]:
    """Walk every node with enable_comms=true, look up its device schema,
    and produce one TagSlot per non-empty *_tag_name field.

    Returns (slots, warnings). `warnings` is a list of human-readable
    strings about nodes that were skipped (unknown device, empty fields,
    etc.).
    """
    slots: list[TagSlot] = []
    warnings: list[str] = []

    for node in nodes:
        if node.resource_id is None:
            continue
        # Godot only writes property OVERRIDES to .tscn files. A device whose
        # PackedScene template defaults enable_comms=true (e.g. StackLight,
        # PushButton variants) doesn't repeat the value on every instance, so
        # filtering for `enable_comms = "true"` would skip them. Default to
        # included; only skip when the .tscn explicitly says false.
        if node.properties.get("enable_comms", "true").strip().lower() == "false":
            continue
        resource_basename = ext_resources.get(node.resource_id, "")
        device_key = RESOURCE_TO_DEVICE.get(resource_basename)
        if device_key is None:
            warnings.append(
                f"[skip] {node.name}: unknown device resource '{resource_basename}'"
                f" (id={node.resource_id})"
            )
            continue
        schema = DEVICE_SCHEMA.get(device_key)
        if not schema:
            warnings.append(
                f"[skip] {node.name}: device {device_key} has no schema entry"
            )
            continue

        for field_name, datatype, direction in schema:
            raw = node.properties.get(field_name)
            if raw is None:
                # Field absent in .tscn: usually means the @export default ""
                # is in effect (no PLC tag wired). Silent skip.
                continue
            tag_name = unquote(raw)
            if not tag_name:
                continue
            if direction == "bidir":
                for sub_dir in ("lci_read", "lci_write"):
                    slots.append(TagSlot(
                        name=tag_name,
                        datatype=datatype,
                        direction=sub_dir,
                        source_node=node.name,
                        source_prop=field_name,
                    ))
            else:
                slots.append(TagSlot(
                    name=tag_name,
                    datatype=datatype,
                    direction=direction,
                    source_node=node.name,
                    source_prop=field_name,
                ))

    return slots, warnings


def assign_indices(
    slots: list[TagSlot],
) -> dict[str, list[tuple[int, TagSlot]]]:
    """Group slots by their target buffer (OIP_READ / OIP_WRITE) and assign
    a stable index to each. Sorting by tag name keeps diffs minimal between
    runs.

    Duplicate tag names within the same buffer are collapsed -- one index
    per name regardless of how many GD nodes share that registration.
    """
    by_buffer: dict[str, list[TagSlot]] = {
        "OIP_READ": [],
        "OIP_WRITE": [],
    }
    for s in slots:
        by_buffer[s.buffer].append(s)

    indexed: dict[str, list[tuple[int, TagSlot]]] = {}
    for buf, items in by_buffer.items():
        # Collapse duplicates: keep the first slot we see for a name. The
        # collapse is important -- two different scene nodes registering the
        # same tag name (e.g. NCP1_4A_VFD:I.Running) must share a slot or the
        # PLC ladder ends up with conflicting destinations.
        seen: dict[str, TagSlot] = {}
        for s in items:
            seen.setdefault(s.name, s)
        sorted_slots = sorted(seen.values(), key=lambda s: s.name)
        indexed[buf] = list(enumerate(sorted_slots))
    return indexed


# ---------------------------------------------------------------------------
# Output emitters
# ---------------------------------------------------------------------------

def write_names_csv(out_dir: Path, buffer: str, rows: list[tuple[int, TagSlot]]) -> None:
    path = out_dir / f"{buffer}_NAMES.csv"
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["index", "name", "datatype", "source_node", "source_property"])
        for idx, slot in rows:
            writer.writerow([idx, slot.name, slot.datatype, slot.source_node, slot.source_prop])


def emit_names_init_st(indexed: dict[str, list[tuple[int, TagSlot]]]) -> str:
    """Pure-ST assignment block populating the two STRING manifest arrays.
    No comments. Designed to live in a first-scan-gated routine (S:FS).
    """
    lines: list[str] = []
    for buf in ("OIP_READ_NAMES", "OIP_WRITE_NAMES"):
        base = buf.replace("_NAMES", "")
        for idx, slot in indexed[base]:
            # Studio 5000 ST: STRING := 'literal'
            # Escape any single quotes in the name (none expected for PLC
            # tag identifiers, but harmless to handle).
            escaped = slot.name.replace("'", "$'")
            lines.append(f"{buf}[{idx}] := '{escaped}';")
    return "\n".join(lines) + ("\n" if lines else "")


def emit_mirror_rungs(
    indexed: dict[str, list[tuple[int, TagSlot]]],
) -> str:
    """Build the Structured-Text blob the programmer pastes into the
    cyclic OIP_MIRROR routine.

    Logix5000 ST -- no comments, section dividers, or blank lines so the
    file can be pasted wholesale into an ST routine without import warnings.

    All-REAL routing: integer-typed source tags (DINT/INT/SINT) assign
    through OIP_READ / OIP_WRITE; Logix5000 auto-converts REAL<->DINT on
    assignment. Lossless for values within +/-2^24.

    Emission order:
      Section 1 (PLC -> Sim):   OIP_READ
      Section 2 (Sim -> PLC):   OIP_WRITE
    """
    lines: list[str] = []
    for idx, slot in indexed["OIP_READ"]:
        tag = slot.name
        if slot.datatype == "BOOL":
            # One-line IF/THEN/ELSE keeps the block readable without
            # bloating the file 3x. Logix5000 ST accepts it.
            lines.append(
                f"IF {tag} THEN OIP_READ[{idx}] := 1.0; "
                f"ELSE OIP_READ[{idx}] := 0.0; END_IF;"
            )
        else:
            lines.append(f"OIP_READ[{idx}] := {tag};")
    for idx, slot in indexed["OIP_WRITE"]:
        tag = slot.name
        if slot.datatype == "BOOL":
            lines.append(f"{tag} := (OIP_WRITE[{idx}] > 0.5);")
        else:
            lines.append(f"{tag} := OIP_WRITE[{idx}];")
    return "\n".join(lines) + "\n"


def write_setup_blob(out_dir: Path, indexed: dict[str, list[tuple[int, TagSlot]]]) -> int:
    """Write the combined names-init + cyclic-mirror ST blob as one .txt file.

    Returns the line number where the mirror section starts (= count of name
    assignment lines), useful for splitting the file into first-scan-gated
    and cyclic routines on the PLC side.
    """
    names_block = emit_names_init_st(indexed)
    mirror_block = emit_mirror_rungs(indexed)
    split_line = names_block.count("\n")
    path = out_dir / "OIP_SETUP.txt"
    with path.open("w", encoding="utf-8") as fh:
        fh.write(names_block)
        fh.write(mirror_block)
    return split_line


def write_summary(
    out_dir: Path,
    scene_path: Path,
    slots: list[TagSlot],
    indexed: dict[str, list[tuple[int, TagSlot]]],
    warnings: list[str],
) -> None:
    """Counts, suggested array sizes, sanity checks. Useful for confirming
    the manifest is what you expect before handing it off."""
    path = out_dir / "OIP_SUMMARY.txt"
    with path.open("w", encoding="utf-8") as fh:
        fh.write(f"OIP manifest summary\n")
        fh.write(f"--------------------\n")
        fh.write(f"Scene: {scene_path}\n")
        fh.write(f"Total slots emitted: {len(slots)}\n")
        fh.write(f"\n")

        for buf in ("OIP_READ", "OIP_WRITE"):
            rows = indexed[buf]
            fh.write(f"  {buf:<16} {len(rows):>4} entries"
                     f"  (recommend STRING[{len(rows)}] + REAL[{len(rows)}])\n")

        fh.write(f"\n")

        # Per-datatype breakdown
        by_type: dict[str, int] = {}
        for s in slots:
            by_type[s.datatype] = by_type.get(s.datatype, 0) + 1
        fh.write(f"Per datatype:\n")
        for k in sorted(by_type):
            fh.write(f"  {k:<6} {by_type[k]}\n")

        fh.write(f"\n")

        # Per-device breakdown
        by_node_type: dict[str, int] = {}
        for s in slots:
            # Source node name often reveals the device class via convention,
            # but the actual count of UNIQUE source nodes is more interesting.
            by_node_type[s.source_node] = by_node_type.get(s.source_node, 0) + 1
        fh.write(f"Per source node ({len(by_node_type)} unique):\n")
        for node_name in sorted(by_node_type):
            fh.write(f"  {node_name:<32} {by_node_type[node_name]}\n")

        if warnings:
            fh.write(f"\nWarnings ({len(warnings)}):\n")
            for w in warnings:
                fh.write(f"  {w}\n")


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--scene", required=True, type=Path,
        help="Path to the .tscn to enumerate (e.g. CDW5_MCM09.tscn).",
    )
    parser.add_argument(
        "--out", default=Path("./out"), type=Path,
        help="Directory to write output files. Created if missing.",
    )
    args = parser.parse_args(argv)

    if not args.scene.is_file():
        print(f"error: scene file not found: {args.scene}", file=sys.stderr)
        return 2
    args.out.mkdir(parents=True, exist_ok=True)

    ext_resources, nodes = parse_tscn(args.scene)
    slots, warnings = enumerate_tags(ext_resources, nodes)
    indexed = assign_indices(slots)

    for buf, rows in indexed.items():
        write_names_csv(args.out, buf, rows)
    split_line = write_setup_blob(args.out, indexed)
    write_summary(args.out, args.scene, slots, indexed, warnings)

    # Print the controller-scope tag allocations the PLC programmer needs.
    # Each STRING[N] holds the manifest names; each parallel REAL[N] / DINT[N]
    # holds the slot values. Dimensions match the actual scene -- bump them up
    # to a fixed ceiling (e.g. 2048) if you want headroom for adding tags
    # without re-running this script + re-downloading the controller.
    counts = {buf: len(indexed[buf]) for buf in indexed}
    print(f"OK: scene={args.scene.name} -> {args.out}")
    print()
    print("PLC controller-scope tag dimensions (all-REAL routing):")
    print(f"  OIP_READ_NAMES   STRING[{counts['OIP_READ']}]      OIP_READ   REAL[{counts['OIP_READ']}]")
    print(f"  OIP_WRITE_NAMES  STRING[{counts['OIP_WRITE']}]      OIP_WRITE  REAL[{counts['OIP_WRITE']}]")
    print()
    print(f"OIP_SETUP.txt: first {split_line} lines = names init (gate on S:FS);")
    print(f"               remaining lines = cyclic mirror block.")
    if warnings:
        print(f"warnings: {len(warnings)} (see OIP_SUMMARY.txt)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
