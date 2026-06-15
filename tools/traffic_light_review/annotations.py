"""라벨 데이터 + precision/recall 지표 + JSON 저장/로드."""
import json
from dataclasses import dataclass, asdict


@dataclass
class Label:
    seg: str
    frame_idx: int     # MISSED 는 재생시점 프레임
    kind: str          # "RED" | "MISSED"
    verdict: str       # "correct" | "false_positive" | "missed"


def detect_red_onsets(states):
    """상태값 리스트(RED=1)에서 비RED→RED 전이 인덱스 목록."""
    onsets = []
    prev = None
    for i, s in enumerate(states):
        if s == 1 and prev != 1:
            onsets.append(i)
        prev = s
    return onsets


def compute_metrics(labels):
    tp = sum(1 for l in labels if l.kind == "RED" and l.verdict == "correct")
    fp = sum(1 for l in labels if l.kind == "RED" and l.verdict == "false_positive")
    fn = sum(1 for l in labels if l.verdict == "missed")
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    return {"tp": tp, "fp": fp, "fn": fn, "precision": precision, "recall": recall}


def aggregate(per_seg):
    """{seg: [onset idx...]} → 요약 dict (세그수/발화세그수/총발화)."""
    total = sum(len(v) for v in per_seg.values())
    with_onset = sum(1 for v in per_seg.values() if v)
    return {
        "segments": len(per_seg),
        "segments_with_onset": with_onset,
        "total_onsets": total,
    }


def save(labels, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump([asdict(l) for l in labels], f, ensure_ascii=False, indent=2)


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        return [Label(**d) for d in json.load(f)]
