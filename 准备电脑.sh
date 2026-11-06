#!/usr/bin/env bash
set -euo pipefail
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python_only=false
case "${1:-}" in
  '') ;;
  --python-only) python_only=true ;;
  *) echo '用法: bash 准备电脑.sh [--python-only]' >&2; exit 2 ;;
esac
if (( EUID == 0 )); then
  echo '请用普通用户运行；系统依赖安装会单独调用 sudo。' >&2
  exit 1
fi
mkdir -p "$task_root/deployment/logs"
exec > >(tee "$task_root/deployment/logs/setup-latest.log") 2>&1
missing=()
for package in python3.12-venv; do
  if [[ $(dpkg-query -W -f='${db:Status-Status}' "$package" 2>/dev/null || true) != installed ]]; then
    missing+=("$package")
  fi
done
if (( ${#missing[@]} )); then
  sudo apt-get update
  sudo apt-get install -y "${missing[@]}"
fi
python3 -E -s -m venv "$task_root/project/.venv"
py="$task_root/project/.venv/bin/python"
"$py" -E -s -m pip install --disable-pip-version-check -r "$task_root/deployment/requirements.txt"
"$py" -E -s -m pip check
if "$python_only"; then
  echo 'Python 环境准备完成；未连接设备，未启动 Unity 或控制程序。'
  echo '离线测试: project/.venv/bin/python -E -s -m pytest -q project/tests deployment/test_pc.py'
  echo '完整复现步骤见 docs/操作手册.md。'
  exit 0
fi
bash "$task_root/启动.sh" test
bash "$task_root/启动.sh" build-apk
bash "$task_root/启动.sh" doctor --offline
echo '电脑依赖与 APK 准备完成。接入 Wi-Fi/网线后，按本机部署说明配置网络并安装 APK。'
echo '本脚本没有连接机械臂或启动控制程序。'
