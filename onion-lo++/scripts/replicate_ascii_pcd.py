#!/usr/bin/env python3
"""Create translated copies of an ASCII PCD while preserving every field."""

import argparse
from pathlib import Path


def parse_translation(value):
    try:
        values = tuple(float(item) for item in value.split(","))
    except ValueError as exception:
        raise argparse.ArgumentTypeError(str(exception)) from exception
    if len(values) != 3:
        raise argparse.ArgumentTypeError(
            "translation must be formatted as x,y,z")
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_ascii_pcd")
    parser.add_argument("output_ascii_pcd")
    parser.add_argument(
        "--translation", action="append", type=parse_translation,
        required=True, help="translated copy as x,y,z; repeat as needed")
    args = parser.parse_args()

    input_path = Path(args.input_ascii_pcd)
    with input_path.open("r", encoding="utf-8") as stream:
        header = []
        fields = None
        declared_points = None
        for line in stream:
            stripped = line.strip()
            if stripped.startswith("FIELDS "):
                fields = stripped.split()[1:]
            elif stripped.startswith("POINTS "):
                declared_points = int(stripped.split()[1])
            header.append(line)
            if stripped.lower() == "data ascii":
                break
        else:
            raise ValueError("input PCD does not contain a DATA ascii header")
        points = [line for line in stream if line.strip()]

    if not fields or not {"x", "y", "z"}.issubset(fields):
        raise ValueError("input PCD must contain x/y/z fields")
    if declared_points is not None and declared_points != len(points):
        raise ValueError(
            f"POINTS declares {declared_points}, found {len(points)} rows")

    coordinate_indices = tuple(fields.index(name) for name in ("x", "y", "z"))
    output_count = len(points) * len(args.translation)
    output_path = Path(args.output_ascii_pcd)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="\n") as stream:
        for line in header:
            stripped = line.strip()
            if stripped.startswith("WIDTH "):
                stream.write(f"WIDTH {output_count}\n")
            elif stripped.startswith("HEIGHT "):
                stream.write("HEIGHT 1\n")
            elif stripped.startswith("POINTS "):
                stream.write(f"POINTS {output_count}\n")
            else:
                stream.write(line.rstrip("\r\n") + "\n")

        for translation in args.translation:
            for line in points:
                values = line.split()
                if len(values) != len(fields):
                    raise ValueError(
                        "point row does not match the FIELDS column count")
                for index, offset in zip(coordinate_indices, translation):
                    values[index] = format(float(values[index]) + offset, ".9g")
                stream.write(" ".join(values) + "\n")

    print(
        f"input_points={len(points)} copies={len(args.translation)} "
        f"output_points={output_count} output={output_path}")


if __name__ == "__main__":
    main()
