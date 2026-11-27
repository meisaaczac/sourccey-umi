"""Thin launcher for training ACT or Diffusion Policy on a Sourccey UMI LeRobot dataset.

Usage:
  python policy/train_lerobot.py --dataset user/sourccey-demo --policy act --dry-run
  python policy/train_lerobot.py --dataset user/sourccey-demo --policy diffusion --steps 50000
"""
import argparse
import shlex
import subprocess
import sys


def build_train_cmd(dataset: str, policy: str, output_dir: str, steps: int, batch_size: int, device: str) -> list[str]:
    return [
        "lerobot-train",
        f"--dataset.repo_id={dataset}",
        f"--policy.type={policy}",
        f"--output_dir={output_dir}",
        f"--job_name=sourccey_umi_{policy}",
        f"--steps={steps}",
        f"--batch_size={batch_size}",
        f"--policy.device={device}",
    ]


def main():
    ap = argparse.ArgumentParser(description="Train ACT or Diffusion Policy on Sourccey UMI LeRobot datasets")
    ap.add_argument("--dataset", required=True, help="Hugging Face dataset repo_id or local LeRobot path")
    ap.add_argument("--policy", choices=("act", "diffusion"), default="act")
    ap.add_argument("--output-dir", default="outputs/sourccey_policy")
    ap.add_argument("--steps", type=int, default=20000)
    ap.add_argument("--batch-size", type=int, default=16)
    ap.add_argument("--device", default="cuda", help="cuda, mps, or cpu")
    ap.add_argument("--dry-run", action="store_true", help="Print command without executing")
    args = ap.parse_args()

    cmd = build_train_cmd(args.dataset, args.policy, args.output_dir, args.steps, args.batch_size, args.device)
    print("Command:", " ".join(shlex.quote(c) for c in cmd))
    if args.dry_run:
        return
    sys.exit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
