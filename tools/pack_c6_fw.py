#!/usr/bin/env python3
"""
Pack the ESP32-C6 coprocessor images into a single container for the P4's
`c6_fw` partition.

The P4 flashes the C6 over UART using esp-serial-flasher, which needs each
image together with the flash offset it belongs at. This bundles them with a
small index so the P4 can iterate them without hardcoding sizes.

    python tools/pack_c6_fw.py                    # reads c6_firmware/build
    python tools/pack_c6_fw.py -o build/c6_fw.bin

Container layout (all little-endian):

    magic     "C6FW"                    4 bytes
    version   1                         u32
    count     number of images          u32
    reserved  0                         u32
    entries[count]:
        flash_offset  where it goes in C6 flash   u32
        size          image length in bytes       u32
        blob_offset   offset within this file     u32
        reserved      0                           u32
    <image data, each 4-byte aligned>

Run this after building c6_firmware and before building the P4 app; then
flash the result to the c6_fw partition offset (see partitions.csv).
"""

import argparse
import os
import struct
import sys

MAGIC = b"C6FW"
VERSION = 1
HEADER_LEN = 16
ENTRY_LEN = 16

# (flash offset in the C6, path relative to the C6 build dir). Mirrors
# c6_firmware/build/flash_args — keep in sync if the partition table changes.
IMAGES = [
    (0x0000, "bootloader/bootloader.bin"),
    (0x8000, "partition_table/partition-table.bin"),
    (0xD000, "ota_data_initial.bin"),
    (0x10000, "c6_hosted_cp.bin"),
]


def pack(build_dir: str, out_path: str) -> None:
    entries = []
    blobs = []
    blob_offset = HEADER_LEN + ENTRY_LEN * len(IMAGES)

    for flash_offset, rel in IMAGES:
        path = os.path.join(build_dir, rel)
        if not os.path.isfile(path):
            sys.exit(f"missing image: {path}\n"
                     f"Build the C6 firmware first:\n"
                     f"  cd c6_firmware && idf.py set-target esp32c6 && idf.py build")
        with open(path, "rb") as fh:
            data = fh.read()

        entries.append((flash_offset, len(data), blob_offset))
        blobs.append(data)

        padded = (len(data) + 3) & ~3
        blob_offset += padded
        print(f"  0x{flash_offset:06x}  {len(data):>9,} B  {rel}")

    total = blob_offset
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "wb") as fh:
        fh.write(MAGIC)
        fh.write(struct.pack("<III", VERSION, len(entries), 0))
        for flash_offset, size, off in entries:
            fh.write(struct.pack("<IIII", flash_offset, size, off, 0))
        for data in blobs:
            fh.write(data)
            pad = ((len(data) + 3) & ~3) - len(data)
            if pad:
                fh.write(b"\xff" * pad)

    print(f"\nwrote {out_path}  ({total:,} bytes, {len(entries)} images)")

    # The c6_fw partition is 2 MB in partitions.csv; fail loudly rather than
    # letting a silent overflow show up as a corrupt flash much later.
    limit = 2 * 1024 * 1024
    if total > limit:
        sys.exit(f"ERROR: container is {total:,} B, larger than the "
                 f"{limit:,} B c6_fw partition. Enlarge it in partitions.csv.")


def main() -> None:
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-b", "--build-dir",
                    default=os.path.join(here, "c6_firmware", "build"),
                    help="C6 firmware build directory")
    ap.add_argument("-o", "--output",
                    default=os.path.join(here, "build", "c6_fw.bin"),
                    help="output container path")
    args = ap.parse_args()

    print(f"packing from {args.build_dir}")
    pack(args.build_dir, args.output)


if __name__ == "__main__":
    main()
