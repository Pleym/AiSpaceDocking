#!/usr/bin/env python3
"""Composes the docking video: full-screen chase view, onboard camera inset, what the CNN saw (with its
detected keypoints), telemetry and live graphs (truth / CNN measurement / Kalman estimate); encodes an MP4
with ffmpeg.

    python compose_video.py --trajectory video_trajectory_cnn.csv --renders OUT_DIR --output docking.mp4 \
        [--cnn CNN_IMAGES_DIR] [--estimates kalman_estimates.csv] [--lang en|fr]

--renders   folder written by render_video.py (onboard/ and exterior/ JPEGs)
--cnn       images seen by the CNN in the loop (image_NNNN.png) + detections.csv (index, u1, v1, ..., uK, vK)
--estimates per control step: time_s, est_x_m, est_vz_m_s, meas_x_m, meas_z_m  -> live graphs
Missing frames (rendered with step > 1) are held, so the video stays real time.
Every on-screen string lives in TEXTS: adding a language means adding one entry, no re-render needed.
"""
import argparse
import subprocess
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT, FPS = 1920, 1080, 30
Z_CONTACT_M, Z_MIN_CAMERA_M = 0.10, 0.5      # same values as simulink/docking_params.m
FONT = "/System/Library/Fonts/Supplemental/Arial.ttf"
TEXTS = {
    "en": {"title": "Autonomous docking guided by a neural network", "onboard": "Onboard camera",
           "ai": "What the AI sees (2 Hz)", "credit": "Orion capsule 3D model: NASA (3D Resources)",
           "phase": "Phase: {}", "contact": "CONTACT", "blind": "Dead reckoning (target too close)",
           "align": "Alignment", "approach": "Approach", "distance": "Distance to port: {:.2f} m",
           "speed": "Closing speed: {:.1f} cm/s", "lateral": "Lateral offset: {:.1f} cm",
           "error": "Vision error: {:.1f} cm", "g_lateral": "Lateral offset x (cm)",
           "g_speed": "Closing speed (cm/s)", "truth": "Truth", "cnn": "CNN measurement",
           "kalman": "Kalman filter", "no_vision": "no vision"},
    "fr": {"title": "Amarrage autonome guidé par un réseau de neurones", "onboard": "Caméra de bord",
           "ai": "Vu par l'IA (2 Hz)", "credit": "Modèle 3D de la capsule Orion : NASA (3D Resources)",
           "phase": "Phase : {}", "contact": "CONTACT", "blind": "Navigation à l'estime (cible trop proche)",
           "align": "Alignement", "approach": "Approche", "distance": "Distance au port : {:.2f} m",
           "speed": "Vitesse d'approche : {:.1f} cm/s", "lateral": "Écart latéral : {:.1f} cm",
           "error": "Erreur de la vision : {:.1f} cm", "g_lateral": "Écart latéral x (cm)",
           "g_speed": "Vitesse d'approche (cm/s)", "truth": "Vérité", "cnn": "Mesure CNN",
           "kalman": "Filtre de Kalman", "no_vision": "sans vision"},
}
GRAPH_BOXES = [(660, 712, 1500, 866), (660, 872, 1500, 1026)]
COLORS = {"truth": (255, 255, 255), "cnn": (255, 90, 90), "kalman": (70, 215, 255)}


def font(size):
    try:
        return ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default()


def inset(canvas, image, position, label, label_font):
    x, y = position
    canvas.paste(image, (x, y))
    draw = ImageDraw.Draw(canvas)
    draw.rectangle([x - 2, y - 2, x + image.width + 1, y + image.height + 1], outline="white", width=2)
    draw.rectangle([x, y + image.height - 34, x + image.width, y + image.height], fill=(0, 0, 0))
    draw.text((x + 10, y + image.height - 30), label, fill="white", font=label_font)


def telemetry(row, end, texts):
    lines = [texts["distance"].format(max(row.z_m - Z_CONTACT_M, 0)),
             texts["speed"].format(100 * row.closing_speed_m_s),
             texts["lateral"].format(100 * np.hypot(row.x_m, row.y_m))]
    measured = np.array([row.meas_x_m, row.meas_y_m, row.meas_z_m])
    if np.isfinite(measured).all():
        lines.append(texts["error"].format(100 * np.linalg.norm(measured - [row.x_m, row.y_m, row.z_m])))
    if row.time_s >= end - 1.0:
        phase = texts["contact"]
    elif row.z_m < Z_MIN_CAMERA_M:
        phase = texts["blind"]
    elif row.closing_speed_m_s < 0.005:
        phase = texts["align"]
    else:
        phase = texts["approach"]
    return [texts["phase"].format(phase)] + lines


def prepare_graphs(trajectory, estimates, texts):
    """Pre-computed curves in pixels. The 'CNN' speed is what raw measurements would give (difference of
    two positions): next to the smooth Kalman speed, it shows what the filter is for."""
    t_max = float(trajectory.time_s.iloc[-1])
    ts = float(estimates.time_s.diff().median())
    graphs = [
        (texts["g_lateral"], [("truth", trajectory.time_s, 100 * trajectory.x_m, "line"),
                              ("cnn", estimates.time_s, 100 * estimates.meas_x_m, "dots"),
                              ("kalman", estimates.time_s, 100 * estimates.est_x_m, "line")]),
        (texts["g_speed"], [("truth", trajectory.time_s, 100 * trajectory.closing_speed_m_s, "line"),
                            ("cnn", estimates.time_s, -100 * estimates.meas_z_m.diff() / ts, "dots"),
                            ("kalman", estimates.time_s, -100 * estimates.est_vz_m_s, "line")]),
    ]
    no_measurement = estimates.time_s[estimates.meas_x_m.isna() & (estimates.time_s > 0)]
    t_blind = float(no_measurement.iloc[0]) if len(no_measurement) else None
    prepared = []
    for (title, series), box in zip(graphs, GRAPH_BOXES):
        truth = np.asarray(series[0][2], float)
        margin = 0.25 * (np.nanmax(truth) - np.nanmin(truth)) + 1.0
        low, high = np.nanmin(truth) - margin, np.nanmax(truth) + margin   # truth scale: CNN outliers are cut
        area = (box[0] + 55, box[1] + 30, box[2] - 12, box[3] - 10)
        curves = []
        for name, t, v, style in series:
            t, v = np.asarray(t, float), np.asarray(v, float)
            keep = np.isfinite(v) & ((style == "line") | ((v >= low) & (v <= high)))
            t, v = t[keep], np.clip(v[keep], low, high)
            px = area[0] + (area[2] - area[0]) * t / t_max
            py = area[3] - (area[3] - area[1]) * (v - low) / (high - low)
            curves.append((name, t, px, py, style))
        prepared.append({"box": box, "area": area, "title": title, "low": low, "high": high,
                         "curves": curves, "t_blind": t_blind, "t_max": t_max})
    return prepared


def draw_graph(draw, graph, t_now, title_font, axis_font, texts, legend):
    box, area, low, high = graph["box"], graph["area"], graph["low"], graph["high"]
    x_of = lambda t: area[0] + (area[2] - area[0]) * t / graph["t_max"]
    draw.rectangle(box, fill=(0, 0, 0, 170))
    if graph["t_blind"] is not None and t_now > graph["t_blind"]:
        xa, xb = x_of(graph["t_blind"]), x_of(min(t_now, graph["t_max"]))
        draw.rectangle([xa, area[1], xb, area[3]], fill=(110, 110, 110, 90))
        draw.text((xa + 4, area[1] + 2), texts["no_vision"], fill=(210, 210, 210), font=axis_font)
    if low < 0 < high:
        y_zero = area[3] - (area[3] - area[1]) * (0 - low) / (high - low)
        draw.line([(area[0], y_zero), (area[2], y_zero)], fill=(90, 90, 90), width=1)
    draw.line([(area[0], area[1]), (area[0], area[3]), (area[2], area[3])], fill=(150, 150, 150), width=1)
    draw.text((box[0] + 8, area[1] - 4), f"{high:.0f}", fill=(180, 180, 180), font=axis_font)
    draw.text((box[0] + 8, area[3] - 18), f"{low:.0f}", fill=(180, 180, 180), font=axis_font)
    draw.text((area[0], box[1] + 5), graph["title"], fill="white", font=title_font)
    if legend:
        x = area[2]
        for name in ("kalman", "cnn", "truth"):         # right to left, each entry gets its own width
            x -= 30 + draw.textlength(texts[name], font=axis_font)
            draw.rectangle([x, box[1] + 11, x + 14, box[1] + 21], fill=COLORS[name])
            draw.text((x + 20, box[1] + 6), texts[name], fill="white", font=axis_font)
    for name, t, px, py, style in graph["curves"]:
        n = int(np.searchsorted(t, t_now, side="right"))
        if style == "line" and n > 1:
            draw.line(list(zip(px[:n].tolist(), py[:n].tolist())), fill=COLORS[name], width=2)
        elif style == "dots":
            for x, y in zip(px[:n].tolist(), py[:n].tolist()):
                draw.ellipse([x - 2.5, y - 2.5, x + 2.5, y + 2.5], fill=COLORS[name])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--renders", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cnn", type=Path)
    parser.add_argument("--estimates", type=Path)
    parser.add_argument("--lang", choices=sorted(TEXTS), default="en")
    args = parser.parse_args()
    texts = TEXTS[args.lang]
    trajectory = pd.read_csv(args.trajectory)
    graphs = prepare_graphs(trajectory, pd.read_csv(args.estimates), texts) if args.estimates else []
    detections = {}
    if args.cnn and (args.cnn / "detections.csv").is_file():
        for line in (args.cnn / "detections.csv").read_text().splitlines():
            values = [float(v) for v in line.split(",")]
            detections[int(values[0])] = np.array(values[1:]).reshape(-1, 2)
    title_font, text_font, small_font = font(34), font(28), font(22)
    graph_font, axis_font = font(20), font(16)
    end = trajectory.time_s.iloc[-1]
    ffmpeg = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                               "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS), "-i", "-", "-c:v", "libx264",
                               "-pix_fmt", "yuv420p", "-crf", "18", str(args.output)], stdin=subprocess.PIPE)
    frame = None
    for i, row in enumerate(trajectory.itertuples()):
        exterior = args.renders / "exterior" / f"{i:05d}.jpg"
        if exterior.is_file():
            frame = Image.open(exterior).convert("RGB")
            onboard = Image.open(args.renders / "onboard" / f"{i:05d}.jpg").convert("RGB").resize((440, 440))
            inset(frame, onboard, (WIDTH - 470, 30), texts["onboard"], small_font)
            seen = args.cnn / f"image_{int(row.cnn_image):04d}.png" if args.cnn else None
            if seen is not None and row.cnn_image >= 0 and seen.is_file():
                view = Image.open(seen).convert("RGB")
                draw = ImageDraw.Draw(view)
                for u, v in detections.get(int(row.cnn_image), []):
                    if 0 <= u < view.width and 0 <= v < view.height:
                        draw.ellipse([u - 4, v - 4, u + 4, v + 4], outline=(255, 60, 60), width=2)
                inset(frame, view.resize((300, 300)), (WIDTH - 330, 500), texts["ai"], small_font)
            draw = ImageDraw.Draw(frame, "RGBA")
            draw.text((30, 25), texts["title"], fill="white", font=title_font)
            lines = telemetry(row, end, texts)
            draw.rectangle([20, HEIGHT - 40 - 40 * len(lines), 640, HEIGHT - 20], fill=(0, 0, 0, 170))
            for k, text in enumerate(lines):
                draw.text((35, HEIGHT - 30 - 40 * (len(lines) - k)), text, fill="white", font=text_font)
            draw.text((30, 70), f"t = {row.time_s:5.1f} s", fill=(200, 200, 200), font=small_font)
            for k, graph in enumerate(graphs):
                draw_graph(draw, graph, row.time_s, graph_font, axis_font, texts, legend=(k == 0))
            # NASA 3D models: credit NASA, no logo, no implied endorsement.
            draw.text((WIDTH - 560, HEIGHT - 40), texts["credit"], fill=(170, 170, 170), font=small_font)
        if frame is not None:
            ffmpeg.stdin.write(frame.tobytes())
    ffmpeg.stdin.close()
    if ffmpeg.wait() != 0:
        raise RuntimeError("ffmpeg failed.")
    print(f"Video: {args.output} ({len(trajectory) / FPS:.0f} s)")


if __name__ == "__main__":
    main()
