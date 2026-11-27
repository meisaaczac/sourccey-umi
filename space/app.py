"""Sourccey UMI Studio — Fleet, Monocular SLAM & LeRobot Dataset Builder.

Adapted directly from Pollen Robotics' `grabette-fleet` + `grabette-slam` workflow
for Vulcan Robotics' Sourccey UMI (single 20x20 mm wrist camera + STS3215 servo,
no GoPro and no RGB-D depth camera required).

Run locally (with live USB camera + STS3215 recording):
    pip install -r space/requirements.txt
    python space/app.py

Or deploy directly as a Hugging Face Space (uses `space/Dockerfile` + `space/README.md`).
"""
import json
import os
import shutil
import sys
import tempfile
import zipfile
from datetime import datetime
from pathlib import Path

import gradio as gr
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "software"))
sys.path.insert(0, str(ROOT / "tools"))

from build_episode import CFG_EXAMPLE, build_episode, load_pose_csv
from grabette_lerobot import export_grabette_episode, export_lerobot_dataset
from mono_slam import estimate_monocular_pose, save_pose_csv

DATA_DIR = Path(os.environ.get("SOURCCEY_UMI_DATA", ROOT / "umi_sessions"))
DATA_DIR.mkdir(parents=True, exist_ok=True)
TASKS_FILE = DATA_DIR / "tasks.json"


# ---------------------------------------------------------------------------
# Session & Task Helpers (mirrors pollen-robotics/grabette-fleet)
# ---------------------------------------------------------------------------
def load_tasks() -> list[dict]:
    if TASKS_FILE.exists():
        try:
            return json.loads(TASKS_FILE.read_text(encoding="utf-8"))
        except Exception:
            pass
    return [{"name": "pick-red-block", "description": "Pick up the red block and place it on the target pad"}]


def save_tasks(tasks: list[dict]) -> None:
    TASKS_FILE.write_text(json.dumps(tasks, indent=2), encoding="utf-8")


def list_episodes(task_name: str | None = None) -> list[str]:
    base = DATA_DIR / "episodes"
    if not base.exists():
        return []
    eps = []
    for p in sorted(base.iterdir()):
        if not p.is_dir():
            continue
        if task_name:
            meta_p = p / "meta.json"
            if meta_p.exists():
                m = json.loads(meta_p.read_text(encoding="utf-8"))
                if m.get("task") != task_name:
                    continue
        eps.append(p.name)
    return eps


def add_task(name: str, desc: str):
    name = name.strip().replace(" ", "-").lower()
    if not name:
        return gr.update(), "Provide a task name first."
    tasks = load_tasks()
    if any(t["name"] == name for t in tasks):
        return gr.update(choices=[t["name"] for t in tasks], value=name), f"Task `{name}` already exists."
    tasks.append({"name": name, "description": desc.strip() or name})
    save_tasks(tasks)
    names = [t["name"] for t in tasks]
    return gr.update(choices=names, value=name), f"Created task `{name}`."


def record_local_episode(task_name: str, cam_idx: int, port: str, servo_id: int, seconds: float):
    """Record a live episode on local hardware (20x20 mm USB camera + Feetech STS3215)."""
    try:
        import cv2
        import record_episode as rec
        import sts_angle
    except Exception as e:
        return gr.update(), None, f"Hardware modules unavailable in this environment: {e}"

    ep_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    ep_dir = DATA_DIR / "episodes" / ep_id
    ep_dir.mkdir(parents=True, exist_ok=True)
    try:
        s = sts_angle.StsAngle(port, int(servo_id))
        s.torque_off()
        if s.cal is None:
            return gr.update(), None, "Run `python software/sts_angle.py calibrate` first."
        cap = cv2.VideoCapture(int(cam_idx))
        if not cap.isOpened():
            return gr.update(), None, f"Could not open camera index {cam_idx}."
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
        wr = cv2.VideoWriter(str(ep_dir / "video.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (w, h))
        info = rec.record(
            str(ep_dir), cap, wr, s, float(seconds), hz=200,
            meta={"cal": s.cal, "task": task_name, "width": w, "height": h},
            wait_enter=False,
        )
        cap.release()
        eps = list_episodes()
        return gr.update(choices=eps, value=ep_id), str(ep_dir / "video.mp4"), json.dumps(info, indent=2)
    except Exception as e:
        return gr.update(), None, f"Recording failed (expected on cloud Spaces without USB hardware): {e}"


def upload_episode_zip(zip_file, task_name: str):
    """Upload a raw sourccey-umi episode ZIP (video.mp4, frames.csv, angle.csv, meta.json)."""
    if zip_file is None:
        return gr.update(), "Upload a `.zip` containing `video.mp4`, `frames.csv`, `angle.csv`, and `meta.json`."
    ep_id = datetime.now().strftime("upload_%Y%m%d_%H%M%S")
    ep_dir = DATA_DIR / "episodes" / ep_id
    ep_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_file.name, "r") as zf:
        zf.extractall(ep_dir)
    # If zip had a single nested folder, flatten it
    children = list(ep_dir.iterdir())
    if len(children) == 1 and children[0].is_dir() and (children[0] / "meta.json").exists():
        for item in children[0].iterdir():
            shutil.move(str(item), str(ep_dir / item.name))
        children[0].rmdir()
    meta_p = ep_dir / "meta.json"
    if meta_p.exists():
        m = json.loads(meta_p.read_text(encoding="utf-8"))
        m["task"] = task_name
        meta_p.write_text(json.dumps(m, indent=2), encoding="utf-8")
    eps = list_episodes()
    return gr.update(choices=eps, value=ep_id), f"Imported episode `{ep_id}` into task `{task_name}`."


def delete_last_episode():
    eps = list_episodes()
    if not eps:
        return gr.update(choices=[], value=None), "No episodes to delete."
    last = eps[-1]
    shutil.rmtree(DATA_DIR / "episodes" / last, ignore_errors=True)
    rem = list_episodes()
    return gr.update(choices=rem, value=rem[-1] if rem else None), f"Deleted episode `{last}`."


# ---------------------------------------------------------------------------
# Pipeline: Monocular SLAM -> Sourccey 5-DoF IK -> Grabette/LeRobot + HF Hub
# ---------------------------------------------------------------------------
def build_and_publish(
    selected_eps: list[str],
    task_desc: str,
    run_slam_ik: bool,
    table_marker_id: int,
    marker_mm: float,
    repo_id: str,
    push_to_hub: bool,
    hf_token: str,
):
    if not selected_eps:
        return "Select at least one episode.", "", ""

    logs = []
    ep_paths = [DATA_DIR / "episodes" / e for e in selected_eps]

    if run_slam_ik:
        try:
            import common as C
            urdf_path = C.default_urdf()
        except Exception as e:
            urdf_path = None
            logs.append(f"[warn] Could not fetch Sourccey URDF ({e}); exporting wrist video + STS3215 gripper only.")

        for ep in ep_paths:
            pose_csv = ep / "pose.csv"
            if not pose_csv.exists():
                t, pos, quat, lost, summary = estimate_monocular_pose(
                    ep / "video.mp4",
                    ep_dir=ep,
                    table_marker_id=int(table_marker_id),
                    marker_size_m=float(marker_mm) / 1000.0,
                )
                save_pose_csv(pose_csv, t, pos, quat, lost)
                logs.append(f"[{ep.name}] Monocular SLAM ({summary['mode']}): "
                            f"{summary['anchor_frames']} ArUco anchors, {summary['vo_frames']} VO frames")
            if urdf_path:
                cfg = json.loads(json.dumps(CFG_EXAMPLE))
                cfg["urdf"] = urdf_path
                try:
                    data, rep = build_episode(str(ep), load_pose_csv(str(pose_csv)), cfg, urdf=urdf_path)
                    np.savez(ep / "episode.npz", **data)
                    logs.append(f"[{ep.name}] Sourccey 5-DoF IK: valid={rep['valid_pct']}%, "
                                f"p95_pos={rep['err_pos_p95_mm']} mm")
                except Exception as e:
                    logs.append(f"[{ep.name}] IK skipped ({e}); keeping raw gripper joint.")

    out_dir = DATA_DIR / "lerobot_exports" / datetime.now().strftime("ds_%Y%m%d_%H%M%S")
    if hf_token.strip():
        os.environ["HF_TOKEN"] = hf_token.strip()

    try:
        res = export_lerobot_dataset(
            ep_paths,
            out_dir,
            task=task_desc.strip() or "manipulate object",
            fps=30.0,
            repo_id=repo_id.strip() or None,
            push_to_hub=bool(push_to_hub),
        )
        logs.append(f"[LeRobot] Built dataset at `{res['out_dir']}` "
                    f"({res['total_episodes']} eps, {res['total_frames']} frames, format={res['format']})")
    except Exception as e:
        return "\n".join(logs + [f"[error] {e}"]), "", ""

    vis_url = res.get("visualizer_url", "")
    iframe_html = ""
    if vis_url:
        logs.append(f"[Visualizer] {vis_url}")
        iframe_html = (
            f'<iframe src="{vis_url}" width="100%" height="680" '
            f'style="border:1px solid #cbd5e1;border-radius:10px;"></iframe>'
        )
    else:
        iframe_html = (
            "<p style='color:#64748b'>Dataset saved locally. Enable <b>Push to Hugging Face Hub</b> "
            "with a public <code>repo_id</code> to open it in the live "
            "<a href='https://huggingface.co/spaces/lerobot/visualize_dataset' target='_blank'>LeRobot Visualizer</a>.</p>"
        )
    return "\n".join(logs), vis_url, iframe_html


# ---------------------------------------------------------------------------
# Gradio UI (Sourccey UMI Fleet + SLAM -> LeRobot)
# ---------------------------------------------------------------------------
def build_ui() -> gr.Blocks:
    tasks = load_tasks()
    task_names = [t["name"] for t in tasks]
    eps = list_episodes()

    with gr.Blocks(title="Sourccey UMI · Fleet & LeRobot Builder", theme=gr.themes.Soft()) as demo:
        gr.Markdown(
            "# Sourccey UMI — Fleet, Monocular SLAM & LeRobot Builder\n"
            "Adapted from **Pollen Robotics Grabette** (`grabette-fleet` + `grabette-slam`) for "
            "**Vulcan Robotics Sourccey** using **only the 20×20 mm wrist camera + STS3215 servo** (no GoPro)."
        )

        with gr.Tab("1. Record & Manage Sessions"):
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### Tasks")
                    task_dd = gr.Dropdown(label="Active Task", choices=task_names, value=task_names[0])
                    new_task_name = gr.Textbox(label="New Task Name", placeholder="pick-red-block")
                    new_task_desc = gr.Textbox(label="Natural Language Prompt", placeholder="Pick up the red block")
                    btn_add_task = gr.Button("Create Task", variant="secondary")
                    task_status = gr.Markdown()

                    gr.Markdown("### Upload Recorded Episode (.zip)")
                    ep_zip = gr.File(label="Upload sourccey-umi episode ZIP", file_types=[".zip"])
                    btn_upload = gr.Button("Import Episode ZIP")

                with gr.Column(scale=1):
                    gr.Markdown("### Local Hardware Recording (20×20 mm Cam + STS3215)")
                    cam_idx = gr.Number(label="USB Camera Index", value=0, precision=0)
                    servo_port = gr.Textbox(label="STS3215 Serial Port", value="/dev/ttyACM0")
                    servo_id = gr.Number(label="STS3215 ID", value=6, precision=0)
                    rec_sec = gr.Slider(label="Duration (seconds)", minimum=3, maximum=120, value=15, step=1)
                    with gr.Row():
                        btn_record = gr.Button("Record Episode", variant="primary")
                        btn_del = gr.Button("Delete Last Episode", variant="stop")
                    ep_preview = gr.Video(label="Latest Wrist Camera Preview")
                    rec_log = gr.Code(label="Session Status", language="json")

        with gr.Tab("2. Build LeRobot Dataset (SLAM + IK)"):
            ep_select = gr.CheckboxGroup(label="Episodes to Include", choices=eps, value=eps)
            task_prompt = gr.Textbox(label="LeRobot Task Description", value="Pick up the red block")
            with gr.Row():
                run_slam_ik = gr.Checkbox(label="Run Monocular SLAM + Sourccey 5-DoF IK", value=True)
                marker_id = gr.Number(label="Tabletop ArUco ID (metric scale)", value=10, precision=0)
                marker_mm = gr.Number(label="Marker Size (mm)", value=80.0)
            with gr.Row():
                repo_id = gr.Textbox(label="Target HF Dataset repo_id", placeholder="meisaaczac/sourccey-umi-demo")
                push_cb = gr.Checkbox(label="Push to Hugging Face Hub", value=False)
                hf_token = gr.Textbox(label="HF Write Token (optional if logged in)", type="password")
            btn_build = gr.Button("Generate LeRobot Dataset", variant="primary")
            build_log = gr.Textbox(label="Pipeline Log", lines=10)
            vis_link = gr.Textbox(label="LeRobot Visualizer URL")

        with gr.Tab("3. LeRobot Visualizer"):
            gr.Markdown(
                "Inspect your published dataset frame-by-frame using "
                "[**lerobot/visualize_dataset**](https://huggingface.co/spaces/lerobot/visualize_dataset)."
            )
            vis_embed = gr.HTML()

        btn_add_task.click(add_task, [new_task_name, new_task_desc], [task_dd, task_status])
        btn_upload.click(upload_episode_zip, [ep_zip, task_dd], [ep_select, task_status])
        btn_record.click(record_local_episode, [task_dd, cam_idx, servo_port, servo_id, rec_sec], [ep_select, ep_preview, rec_log])
        btn_del.click(delete_last_episode, [], [ep_select, task_status])
        btn_build.click(
            build_and_publish,
            [ep_select, task_prompt, run_slam_ik, marker_id, marker_mm, repo_id, push_cb, hf_token],
            [build_log, vis_link, vis_embed],
        )
    return demo


if __name__ == "__main__":
    demo = build_ui()
    demo.launch(server_name="0.0.0.0", server_port=int(os.environ.get("PORT", 7860)))
