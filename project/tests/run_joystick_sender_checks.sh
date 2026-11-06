#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
unity_data="${UNITY_EDITOR_DATA:-$HOME/Unity/Hub/Editor/2022.3.62f3/Editor/Data}"
mono_bin="$unity_data/MonoBleedingEdge/bin"
scratch_dir=$(mktemp -d /tmp/rm75-stick-checks.XXXXXX)
trap 'rm -rf "$scratch_dir"' EXIT
for target in UNITY_ANDROID UNITY_EDITOR; do
  "$mono_bin/mcs" -nowarn:0436,0169,0414 -define:"$target" -out:"$scratch_dir/$target.exe" \
    tests/JoystickSenderHarness.cs Assets/Scripts/PicoJoystickSender.cs Assets/Scripts/PICOEndpointConfig.cs
  "$mono_bin/mono" "$scratch_dir/$target.exe"
done
"$mono_bin/mcs" -target:library -define:UNITY_ANDROID -out:"$scratch_dir/stick.dll" \
  -r:"$unity_data/MonoBleedingEdge/lib/mono/4.7.1-api/Facades/netstandard.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.CoreModule.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.XRModule.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.JSONSerializeModule.dll" \
  Assets/Scripts/PicoJoystickSender.cs Assets/Scripts/PICOEndpointConfig.cs
printf '%s\n' 'Joystick sender compiled against installed Unity APIs'
