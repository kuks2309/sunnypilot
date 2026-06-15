from tools.traffic_light_review.annotations import (
    Label, detect_red_onsets, compute_metrics,
)


def test_detect_red_onsets():
    # 상태 시퀀스에서 비RED→RED 전이 인덱스를 onset 으로
    states = [0, 0, 1, 1, 1, 0, 1]   # RED=1
    onsets = detect_red_onsets(states)
    assert onsets == [2, 6]


def test_compute_metrics_precision_recall():
    # 발화 3건 중 2건 정상(TP), 1건 오탐(FP), 미검출 1건(FN)
    labels = [Label("seg", 2, "RED", "correct"),
              Label("seg", 6, "RED", "correct"),
              Label("seg", 9, "RED", "false_positive"),
              Label("seg", 20, "MISSED", "missed")]
    m = compute_metrics(labels)
    assert m["tp"] == 2 and m["fp"] == 1 and m["fn"] == 1
    assert abs(m["precision"] - 2 / 3) < 1e-9
    assert abs(m["recall"] - 2 / 3) < 1e-9


def test_save_load_roundtrip(tmp_path):
    from tools.traffic_light_review.annotations import save, load
    labels = [Label("seg", 2, "RED", "correct")]
    p = tmp_path / "a.json"
    save(labels, str(p))
    assert load(str(p)) == labels


def test_aggregate_onsets_counts():
    from tools.traffic_light_review.annotations import aggregate
    per_seg = {"s1": [2, 10], "s2": [], "s3": [4]}
    agg = aggregate(per_seg)
    assert agg["total_onsets"] == 3
    assert agg["segments"] == 3
    assert agg["segments_with_onset"] == 2
