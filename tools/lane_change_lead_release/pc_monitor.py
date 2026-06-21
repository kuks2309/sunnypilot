#!/usr/bin/env python3
"""PC에서 실행 — SSH로 comma 기기의 실시간 모니터를 띄우고 출력을 PC 터미널로 스트리밍.

이 PC(Windows)엔 openpilot/cereal 이 없으므로, 무거운 cereal 구독은 기기에서 돌리고
PC 는 SSH 표준출력만 실시간으로 받아 표시한다.

사용 (git-bash 권장):
    python3 tools/lane_change_lead_release/pc_monitor.py [duration_s] [host] [key]
기본값: duration=3600s, host=172.16.87.1, key=~/.ssh/id_ed25519
전제: 기기에 /tmp/lcr/tools/lane_change_lead_release/live_monitor.py 가 배치돼 있어야 함.
중지: Ctrl-C
"""
import os
import shutil
import subprocess
import sys

REMOTE_MON = "/tmp/lcr/tools/lane_change_lead_release/live_monitor.py"

# Windows PowerShell 은 ssh 가 PATH 에 없을 수 있어 표준 위치를 폴백으로 탐색
SSH = shutil.which("ssh")
if SSH is None and os.name == "nt":
    SSH = r"C:\Windows\System32\OpenSSH\ssh.exe"
if SSH is None:
    SSH = "ssh"


def main():
    dur = sys.argv[1] if len(sys.argv) > 1 else "3600"
    host = sys.argv[2] if len(sys.argv) > 2 else "172.16.87.1"
    key = sys.argv[3] if len(sys.argv) > 3 else os.path.expanduser("~/.ssh/id_ed25519")

    remote = (
        "export PYTHONPATH=/tmp/lcr:/data/openpilot; "
        f"/usr/local/venv/bin/python -u {REMOTE_MON} {dur}"
    )
    cmd = [
        SSH, "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
        "-o", "ServerAliveInterval=5", "-i", key, f"comma@{host}", remote,
    ]

    print(f"[PC] connecting {host} ... (Ctrl-C to stop)", flush=True)
    p = None
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in p.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n[PC] stopped.", flush=True)
    finally:
        if p is not None:
            try:
                p.terminate()
            except Exception:
                pass


if __name__ == "__main__":
    main()
