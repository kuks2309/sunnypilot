"""신호등(정지/출발) 검출 리뷰·시각화 도구.

검출은 sunnypilot greenLightAlert 방식(정차+앞차없음+경로 열림/닫힘)으로 RED/GREEN 판정.

사용법 (Windows 경로):
  python -m tools.traffic_light_review.review <세그> --summary           # 비대화형 통계
  python -m tools.traffic_light_review.review <세그> --render out.mp4     # 영상에 RED/GREEN 오버레이
  python -m tools.traffic_light_review.review batch --out <부모디렉터리>  # 전체 세그 발화빈도
  python -m tools.traffic_light_review.review <세그> [--out labels.json]  # pygame 인터랙티브(설치시)
"""
import argparse
import glob
import os
import sys

from tools.traffic_light_review.logreader import load_segment
from tools.traffic_light_review.detector_runner import run_detector
from tools.traffic_light_review.annotations import (
    Label, detect_red_onsets, compute_metrics, save, aggregate,
)

STATE_NAME = {0: "OFF", 1: "RED", 2: "GREEN"}
STATE_BGR = {0: (160, 160, 160), 1: (0, 0, 220), 2: (0, 200, 0)}   # OpenCV BGR


def summary(seg_dir):
    frames, _ = load_segment(seg_dir)
    states, outs = run_detector(frames)
    onsets = detect_red_onsets(states)
    n_green = sum(1 for s in states if s == 2)
    print(f"세그먼트: {os.path.basename(seg_dir.rstrip(os.sep))}")
    print(f"  프레임 {len(frames)}개, RED onset {len(onsets)}건, GREEN 프레임 {n_green}개")
    for i in onsets:
        d = outs[i].diagnostics
        print(f"   frame {i}: x_end={d['model_x_end']} v={d['v_ego']} "
              f"armed={d['armed']} lead={d['has_lead']}")


def batch(parent_dir):
    segs = sorted(glob.glob(os.path.join(parent_dir, "*--*")))
    per_seg = {}
    for s in segs:
        if not os.path.isdir(s):
            continue
        try:
            frames, _ = load_segment(s)
        except Exception:
            continue
        states, _ = run_detector(frames)
        per_seg[os.path.basename(s)] = detect_red_onsets(states)
    agg = aggregate(per_seg)
    print(f"세그먼트 {agg['segments']}개 중 {agg['segments_with_onset']}개에서 "
          f"RED 발화, 총 {agg['total_onsets']}건")
    for seg, onsets in per_seg.items():
        if onsets:
            print(f"   {seg}: {len(onsets)}건 @ {onsets}")
    return agg


def render(seg_dir, out_path):
    import cv2
    frames, qcam = load_segment(seg_dir)
    if not qcam:
        print("qcamera.ts 없음 — 렌더 불가", file=sys.stderr)
        return
    states, outs = run_detector(frames)
    cap = cv2.VideoCapture(qcam)
    fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    vw = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    i = 0
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        idx = min(i, len(outs) - 1)
        s = states[idx]
        d = outs[idx].diagnostics
        cv2.circle(fr, (44, 44), 24, STATE_BGR[s], -1)
        cv2.putText(fr, STATE_NAME[s], (78, 54), cv2.FONT_HERSHEY_SIMPLEX, 1.0, STATE_BGR[s], 2)
        info = [f"x_end:{d['model_x_end']}m", f"v:{d['v_ego']}", f"would_alert:{d['would_alert']}",
                f"lead:{d['has_lead']}", f"cc:{d['cc_enabled']}"]
        for k, ln in enumerate(info):
            cv2.putText(fr, ln, (12, 92 + k * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        vw.write(fr)
        i += 1
    cap.release()
    vw.release()
    n_red = sum(1 for s in states if s == 1)
    n_green = sum(1 for s in states if s == 2)
    print(f"렌더 완료: {out_path} ({i} 프레임, RED {n_red}f / GREEN {n_green}f)")


def interactive(seg_dir, out_path):
    import pygame
    import cv2
    frames, qcam = load_segment(seg_dir)
    states, outs = run_detector(frames)
    onsets = detect_red_onsets(states)
    cap = cv2.VideoCapture(qcam) if qcam else None
    n_video = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap else len(frames)
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    font = pygame.font.SysFont("consolas", 20)
    clock = pygame.time.Clock()
    labels = []
    idx = 0
    playing = False
    onset_ptr = 0

    def draw():
        screen.fill((20, 20, 20))
        if cap:
            cap.set(cv2.CAP_PROP_POS_FRAMES, min(idx, n_video - 1))
            ok, fr = cap.read()
            if ok:
                fr = cv2.cvtColor(cv2.resize(fr, (800, 600)), cv2.COLOR_BGR2RGB)
                screen.blit(pygame.surfarray.make_surface(fr.swapaxes(0, 1)), (10, 10))
        out = outs[min(idx, len(outs) - 1)]
        d = out.diagnostics
        lines = [
            f"frame {idx}/{len(frames)-1}",
            f"STATE: {out.state.name}",
            f"x_end: {d['model_x_end']} m",
            f"v_ego: {d['v_ego']}  armed: {d['armed']}",
            f"has_lead: {d['has_lead']}  cc: {d['cc_enabled']}",
            f"green_timer: {d['green_timer']}",
            "",
            f"onsets: {len(onsets)}  labeled: {len(labels)}",
            "1=정상 2=오탐 m=미검출 s=저장 q=종료",
        ]
        rgb = {"OFF": (160, 160, 160), "RED": (220, 60, 60), "GREEN": (60, 200, 60)}[out.state.name]
        pygame.draw.circle(screen, rgb, (1180, 40), 18)
        for k, ln in enumerate(lines):
            screen.blit(font.render(ln, True, (230, 230, 230)), (820, 20 + k * 26))
        pygame.display.flip()

    running = True
    while running:
        if playing and onset_ptr < len(onsets) and idx >= onsets[onset_ptr]:
            playing = False
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_SPACE:
                    playing = not playing
                elif ev.key == pygame.K_RIGHT:
                    idx = min(idx + 1, len(frames) - 1)
                elif ev.key == pygame.K_LEFT:
                    idx = max(idx - 1, 0)
                elif ev.key == pygame.K_q:
                    running = False
                elif ev.key == pygame.K_s:
                    save(labels, out_path)
                    print(f"저장: {out_path}")
                elif ev.key == pygame.K_m:
                    labels.append(Label(os.path.basename(seg_dir.rstrip(os.sep)), idx, "MISSED", "missed"))
                elif ev.key in (pygame.K_1, pygame.K_2):
                    verdict = "correct" if ev.key == pygame.K_1 else "false_positive"
                    at = onsets[onset_ptr] if onset_ptr < len(onsets) else idx
                    labels.append(Label(os.path.basename(seg_dir.rstrip(os.sep)), at, "RED", verdict))
                    onset_ptr = min(onset_ptr + 1, len(onsets))
                    playing = True
        if playing:
            idx = min(idx + 1, len(frames) - 1)
            if idx >= len(frames) - 1:
                playing = False
        draw()
        clock.tick(20)

    save(labels, out_path)
    m = compute_metrics(labels)
    print(f"지표: TP={m['tp']} FP={m['fp']} FN={m['fn']} "
          f"precision={m['precision']:.2f} recall={m['recall']:.2f}")
    pygame.quit()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seg_dir", help="세그먼트 디렉터리, 또는 'batch'")
    ap.add_argument("--out", default="tl_labels.json", help="라벨 출력 / batch 시 부모 디렉터리")
    ap.add_argument("--summary", action="store_true")
    ap.add_argument("--render", metavar="OUT.MP4", help="영상에 RED/GREEN 오버레이 출력")
    a = ap.parse_args()
    if a.seg_dir == "batch":
        batch(a.out)
        return
    if not os.path.isdir(a.seg_dir):
        print("세그먼트 디렉터리 없음", file=sys.stderr)
        sys.exit(1)
    if a.render:
        render(a.seg_dir, a.render)
    elif a.summary:
        summary(a.seg_dir)
    else:
        interactive(a.seg_dir, a.out)


if __name__ == "__main__":
    main()
