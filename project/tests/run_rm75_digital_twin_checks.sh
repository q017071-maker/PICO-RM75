#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
unity_data="${UNITY_EDITOR_DATA:-$HOME/Unity/Hub/Editor/2022.3.62f3/Editor/Data}"
mono_bin="$unity_data/MonoBleedingEdge/bin"
scratch_dir=$(mktemp -d /tmp/rm75-digital-twin-checks.XXXXXX)
trap 'rm -rf "$scratch_dir"' EXIT

runtime_scripts=(
  Assets/Scripts/RM75StateProtocol.cs
  Assets/Scripts/RM75StateReceiver.cs
  Assets/Scripts/RM75JointAnimator.cs
  Assets/Scripts/RM75FixedPlacement.cs
)

"$mono_bin/mcs" -langversion:latest -out:"$scratch_dir/digital-twin-tests.exe" \
  -r:"$mono_bin/../lib/mono/4.7.1-api/System.Web.Extensions.dll" \
  tests/Rm75DigitalTwinHarness.cs "${runtime_scripts[@]}"
"$mono_bin/mono" "$scratch_dir/digital-twin-tests.exe"

"$mono_bin/mcs" -langversion:latest -target:library \
  -out:"$scratch_dir/digital-twin.dll" \
  -r:"$unity_data/MonoBleedingEdge/lib/mono/4.7.1-api/Facades/netstandard.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.CoreModule.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.JSONSerializeModule.dll" \
  "${runtime_scripts[@]}"
printf '%s\n' 'RM75 digital twin scripts compiled against installed Unity APIs'
