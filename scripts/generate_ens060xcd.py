#!/usr/bin/env python3
"""Convert the ENS060XCD WBF's inkwave JSON into an EPDiy waveform header.

Usage:
    inkwave -o /tmp/ens060xcd.json ENS060XCD_R5H013_2022112201.wbf
    python3 scripts/generate_ens060xcd.py /tmp/ens060xcd.json \
        src/waveforms/epdiy_ENS060XCD.h \
        --wbf ENS060XCD_R5H013_2022112201.wbf

The WBF stores 32 x 32 transitions. EPDiy's 4-bit framebuffer uses 16 x 16,
so gray level n selects native level 2*n, as waveform_hdrgen.py does.
"""

import argparse
import hashlib
import json
from pathlib import Path


SOURCE_NAME = "ENS060XCD_R5H013_2022112201.wbf"
SOURCE_SHA256 = "747dc80768a7e51701638ec9055d8f26aa837cd339adc5ad88b5b97a06baa74b"
MODE_COUNT = 8
RANGE_COUNT = 14


def pack_phases(phases):
    packed = bytearray()
    if not phases:
        raise ValueError("A waveform range has no phases")
    for phase in phases:
        if len(phase) != 32 or any(len(row) != 32 for row in phase):
            raise ValueError("Expected a 32 x 32 native phase")
        for target in range(0, 32, 2):
            for source in range(0, 32, 8):
                values = [phase[target][source + offset] for offset in (0, 2, 4, 6)]
                if any(not isinstance(value, int) or value < 0 or value > 3 for value in values):
                    raise ValueError("Invalid 2-bit drive value")
                packed.append(
                    (values[0] << 6) | (values[1] << 4) | (values[2] << 2) | values[3]
                )
    return bytes(packed)


def generate(waveforms):
    modes = waveforms["modes"]
    bounds = waveforms["temperature_ranges"]["range_bounds"]
    if [mode["mode"] for mode in modes] != list(range(MODE_COUNT)):
        raise ValueError("Expected source mode slots 0 through 7 in order")
    if len(bounds) != RANGE_COUNT:
        raise ValueError("Expected all 14 source temperature ranges")
    if any(bounds[i]["to"] != bounds[i + 1]["from"] for i in range(RANGE_COUNT - 1)):
        raise ValueError("Temperature intervals are not contiguous")

    output = [
        "/* Generated from " + SOURCE_NAME + " (SHA-256 " + SOURCE_SHA256 + ").",
        " * Source format: 5-bit, 32 x 32 transition tables; EPDiy: 4-bit, 16 x 16.",
        " * Native levels 0, 2, ..., 30 map to EPDiy levels 0, 1, ..., 15.",
        " * Mode numbers are WBF slots. The vendor's mode names are not encoded here.",
        " * Slots 3, 4 and 5 contain identical waveforms in every temperature range.",
        " * Slot 6 is a black/white-only fast mode, despite EPDiy's generic name for 6.",
        " * Rebuild with scripts/generate_ens060xcd.py; do not edit the LUTs by hand.",
        " */",
        "#pragma once",
        "#include <stddef.h>",
        '#include "epd_internals.h"',
        "",
    ]
    unique = {}
    references = []
    for mode in modes:
        if [entry["index"] for entry in mode["ranges"]] != list(range(RANGE_COUNT)):
            raise ValueError("Expected 14 ordered ranges in each mode")
        mode_refs = []
        for entry in mode["ranges"]:
            mode_index = mode["mode"]
            range_index = entry["index"]
            phase_count = len(entry["phases"])
            packed = pack_phases(entry["phases"])
            if packed not in unique:
                name = f"ens060xcd_{mode_index}_{range_index}"
                unique[packed] = name
                output.append(f"/* Mode slot {mode_index}, {bounds[range_index]['from']}..{bounds[range_index]['to']} C: {phase_count} phases. */")
                output.append(f"static const uint8_t {name}_data[{phase_count}][64] = {{")
                for phase_index in range(phase_count):
                    output.append(f"    /* phase {phase_index} */ {{")
                    segment = packed[phase_index * 64 : (phase_index + 1) * 64]
                    for offset in range(0, 64, 16):
                        output.append("        " + ", ".join(f"0x{value:02x}" for value in segment[offset : offset + 16]) + ",")
                    output.append("    },")
                output.extend(
                    [
                        "};",
                        f"static const EpdWaveformPhases {name} = {{",
                        f"    .phases = {phase_count},",
                        f"    .luts = &{name}_data[0][0],",
                        "    .phase_times = NULL,",
                        "};",
                        "",
                    ]
                )
            mode_refs.append(unique[packed])
        references.append(mode_refs)

    output.append("static const EpdWaveformTempInterval ens060xcd_intervals[14] = {")
    output.extend(
        f"    {{ .min = {bound['from']}, .max = {bound['to']} }}," for bound in bounds
    )
    output.extend(["};", ""])
    for mode_index, mode_refs in enumerate(references):
        output.append(f"static const EpdWaveformPhases* ens060xcd_{mode_index}_ranges[14] = {{")
        output.extend(f"    &{name}," for name in mode_refs)
        output.extend(
            [
                "};",
                f"static const EpdWaveformMode ens060xcd_mode_{mode_index} = {{",
                f"    .type = {mode_index},",
                "    .temp_ranges = 14,",
                f"    .range_data = ens060xcd_{mode_index}_ranges,",
                "};",
                "",
            ]
        )
    output.append("static const EpdWaveformMode* ens060xcd_modes[8] = {")
    output.extend(f"    &ens060xcd_mode_{mode_index}," for mode_index in range(MODE_COUNT))
    output.extend(
        [
            "};",
            "const EpdWaveform epdiy_ENS060XCD = {",
            "    .num_modes = 8,",
            "    .num_temp_ranges = 14,",
            "    .mode_data = ens060xcd_modes,",
            "    .temp_intervals = ens060xcd_intervals,",
            "};",
            "",
        ]
    )
    return "\n".join(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("json_file", type=Path, help="JSON exported by vroland/inkwave")
    parser.add_argument("header_file", type=Path)
    parser.add_argument("--wbf", type=Path, required=True)
    args = parser.parse_args()
    digest = hashlib.sha256(args.wbf.read_bytes()).hexdigest()
    if digest != SOURCE_SHA256:
        raise ValueError(f"Unexpected source WBF SHA-256: {digest}")
    waveforms = json.loads(args.json_file.read_text())
    args.header_file.write_text(generate(waveforms))


if __name__ == "__main__":
    main()
