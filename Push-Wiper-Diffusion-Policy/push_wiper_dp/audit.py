"""Validate every segment and print the portable data identity."""

import argparse
import json
from pathlib import Path

from push_wiper_dp.dataset import PushWiperDataset


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--output", type=Path, help="Also save the JSON audit summary")
    args = parser.parse_args()
    dataset = PushWiperDataset(args.data_root)
    result = json.dumps(dataset.describe(), ensure_ascii=False, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result + "\n", encoding="utf-8")
    print(result)


if __name__ == "__main__":
    main()
