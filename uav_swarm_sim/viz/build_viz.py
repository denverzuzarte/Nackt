#!/usr/bin/env python3
"""
Build the final self-contained visualization.html by injecting a
previously-exported history JSON into template.html.

Usage:
    python3 export_history.py --seed 1 --out /tmp/history.json
    python3 viz/build_viz.py --data /tmp/history.json --out visualization.html
"""
import argparse
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=str, required=True)
    ap.add_argument("--template", type=str, default=os.path.join(HERE, "template.html"))
    ap.add_argument("--out", type=str, default="visualization.html")
    args = ap.parse_args()

    with open(args.data, encoding="utf-8") as f:
        data_text = f.read()
    # validate it's real JSON before embedding
    json.loads(data_text)

    with open(args.template, encoding="utf-8") as f:
        template = f.read()

    out_html = template.replace("__DATA_JSON__", data_text)

    with open(args.out, "w", encoding="utf-8") as f:
        f.write(out_html)

    size_mb = os.path.getsize(args.out) / (1024 * 1024)
    print(f"Wrote {args.out}: {size_mb:.2f} MB")


if __name__ == "__main__":
    main()
