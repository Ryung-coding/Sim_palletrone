
#!/usr/bin/env python3
"""
STLchanger.py — Scale STL from millimeters to meters (×0.001) with no external deps.
Supports Binary and ASCII STL. Leaves normals unchanged (only vertex coordinates are scaled).

Usage:
  python3 STLchanger.py model.stl                # -> model_m.stl (binary/ascii preserved)
  python3 STLchanger.py model.stl --inplace      # overwrite the source file
  python3 STLchanger.py model.stl -o out.stl     # write to a chosen file
  python3 STLchanger.py /path/to/dir --dir       # batch convert all .stl in a directory
  python3 STLchanger.py /path/to/dir --dir --recursive
  python3 STLchanger.py --help

Notes:
  - ASCII detection uses a simple heuristic; if in doubt, pass --force-ascii or --force-binary.
  - For ASCII STL, only lines beginning with "vertex " are scaled; facet normals remain as-is.
"""

import argparse
import sys
import os
from pathlib import Path
import struct
import re
from typing import Tuple, Optional

SCALE = 1.0/1000.0  # mm -> m

def is_ascii_stl(raw_head: bytes, file_size: int) -> bool:
    # If header starts with 'solid' and doesn't fit binary triangle count math, treat as ASCII.
    head = raw_head.lstrip()
    if head.lower().startswith(b'solid'):
        return True
    # Otherwise likely binary
    return False

def read_uint32_le(b: bytes, off: int) -> int:
    return struct.unpack_from('<I', b, off)[0]

def scale_binary_stl(data: bytes) -> bytes:
    if len(data) < 84:
        raise ValueError("File too small for binary STL.")
    # header (80), count (4), then n * (50)
    ntri = read_uint32_le(data, 80)
    expected = 84 + ntri * 50
    if expected != len(data):
        # Sometimes files have padding; still try if plausible
        if 84 + ntri * 50 > len(data):
            raise ValueError(f"Binary STL length mismatch (ntri={ntri}, size={len(data)}).")
    out = bytearray(data)  # copy
    # Iterate facets
    off = 84
    for i in range(ntri):
        # normals: 3 floats @ off .. off+11 -> leave unchanged
        # vertices: 9 floats @ off+12 .. off+47
        v_off = off + 12
        for j in range(9):
            f = struct.unpack_from('<f', out, v_off + 4*j)[0]
            f *= SCALE
            struct.pack_into('<f', out, v_off + 4*j, f)
        # attr byte count (2 bytes) left as is
        off += 50
        if off > len(out):
            break
    return bytes(out)

_vertex_re = re.compile(r'(^\s*vertex\s+)([-+0-9.eE]+\s+[-+0-9.eE]+\s+[-+0-9.eE]+)(\s*$)', re.IGNORECASE)

def _scale_triplet_str(trip: str) -> str:
    parts = trip.strip().split()
    if len(parts) != 3:
        return trip
    try:
        x, y, z = (float(parts[0])*SCALE, float(parts[1])*SCALE, float(parts[2])*SCALE)
        # keep a reasonable precision
        return f"{x:.9g} {y:.9g} {z:.9g}"
    except Exception:
        return trip

def scale_ascii_stl(text: str) -> str:
    # Replace only vertex lines; normals remain
    def repl(m):
        prefix, trip, suffix = m.groups()
        return prefix + _scale_triplet_str(trip) + suffix
    return _vertex_re.sub(repl, text, count=0)

def convert_file(in_path: Path, out_path: Path, inplace: bool, force: Optional[str]) -> bool:
    try:
        raw = in_path.read_bytes()
        ascii_mode = None
        if force == 'ascii':
            ascii_mode = True
        elif force == 'binary':
            ascii_mode = False
        else:
            ascii_mode = is_ascii_stl(raw[:256], len(raw))

        if ascii_mode:
            txt = raw.decode('utf-8', errors='ignore')
            out_txt = scale_ascii_stl(txt)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(out_txt, encoding='utf-8', newline='')
            fmt = "ASCII"
        else:
            out_bytes = scale_binary_stl(raw)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_bytes(out_bytes)
            fmt = "Binary"

        tag = "(in-place)" if inplace else ""
        print(f"[OK] {in_path.name} -> {out_path.name} {tag} [{fmt}]")
        return True
    except Exception as e:
        print(f"[ERROR] {in_path}: {e}", file=sys.stderr)
        return False

def main():
    ap = argparse.ArgumentParser(description="Scale STL from millimeters to meters (×0.001). No external deps.")
    ap.add_argument("path", type=Path, help="Input STL file or directory")
    ap.add_argument("-o", "--output", type=Path, help="Output file or directory")
    ap.add_argument("--inplace", action="store_true", help="Overwrite input file (single file mode)")
    ap.add_argument("--dir", action="store_true", help="Treat input path as a directory and batch convert")
    ap.add_argument("--recursive", action="store_true", help="Recurse into subdirectories (with --dir)")
    ap.add_argument("--suffix", default="_m", help="Suffix before .stl when not overwriting (default: _m)")
    ap.add_argument("--force-ascii", dest="force_ascii", action="store_true", help="Force ASCII parse/write")
    ap.add_argument("--force-binary", dest="force_binary", action="store_true", help="Force Binary parse/write")
    args = ap.parse_args()

    force = None
    if args.force_ascii and args.force_binary:
        print("[ERROR] --force-ascii and --force-binary cannot be used together.", file=sys.stderr)
        sys.exit(2)
    if args.force_ascii: force = 'ascii'
    if args.force_binary: force = 'binary'

    if args.dir:
        if not args.path.is_dir():
            print("[ERROR] --dir specified but path is not a directory.", file=sys.stderr)
            sys.exit(2)
        root = args.path
        out_root = args.output if args.output else root
        pattern = "**/*.stl" if args.recursive else "*.stl"
        files = list(root.glob(pattern))
        if not files:
            print("[WARN] No .stl files found.", file=sys.stderr)
            sys.exit(1)
        ok = 0
        for fp in files:
            rel = fp.relative_to(root)
            out_dir = (out_root / rel.parent)
            out_name = fp.stem + args.suffix + ".stl"
            out_path = out_dir / out_name
            if args.inplace:
                out_path = fp
            if convert_file(fp, out_path, args.inplace, force):
                ok += 1
        print(f"[DONE] Converted {ok}/{len(files)} files.")
        sys.exit(0 if ok == len(files) else 1)

    # single file mode
    if not args.path.is_file():
        print("[ERROR] Input path must be a file (or use --dir).", file=sys.stderr)
        sys.exit(2)
    in_file = args.path
    if args.inplace and args.output:
        print("[ERROR] --inplace and --output are mutually exclusive.", file=sys.stderr)
        sys.exit(2)
    if args.inplace:
        out_file = in_file
    elif args.output:
        out_file = args.output
        if out_file.is_dir():
            out_file = out_file / (in_file.stem + args.suffix + ".stl")
    else:
        out_file = in_file.with_name(in_file.stem + args.suffix + ".stl")

    ok = convert_file(in_file, out_file, args.inplace, force)
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()