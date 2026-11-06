#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
unity_data="${UNITY_EDITOR_DATA:-$HOME/Unity/Hub/Editor/2022.3.62f3/Editor/Data}"
mono_bin="$unity_data/MonoBleedingEdge/bin"
scratch_dir=$(mktemp -d /tmp/rm75-sender-checks.XXXXXX)
trap 'rm -rf "$scratch_dir"' EXIT
"$mono_bin/mcs" -nowarn:0436 -out:"$scratch_dir/sender-tests.exe" \
  tests/PicoSenderHarness.cs Assets/Scripts/PICO_UDP_Sender.cs Assets/Scripts/PICOEndpointConfig.cs
"$mono_bin/mono" "$scratch_dir/sender-tests.exe"
"$mono_bin/mcs" -nowarn:0436 -define:UNITY_ANDROID -out:"$scratch_dir/android-sender-tests.exe" \
  tests/PicoSenderHarness.cs Assets/Scripts/PICO_UDP_Sender.cs Assets/Scripts/PICOEndpointConfig.cs
"$mono_bin/mono" "$scratch_dir/android-sender-tests.exe"
"$mono_bin/mcs" -target:library -out:"$scratch_dir/sender.dll" \
  -r:"$unity_data/MonoBleedingEdge/lib/mono/4.7.1-api/Facades/netstandard.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.CoreModule.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.XRModule.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.JSONSerializeModule.dll" \
  Assets/Scripts/PICO_UDP_Sender.cs Assets/Scripts/PICOEndpointConfig.cs
"$mono_bin/mcs" -out:"$scratch_dir/endpoint-tests.exe" \
  tests/PicoEndpointHarness.cs Assets/Scripts/PICOEndpointConfig.cs
"$mono_bin/mono" "$scratch_dir/endpoint-tests.exe"
"$mono_bin/mcs" -target:library -define:UNITY_ANDROID -out:"$scratch_dir/android-sender.dll" \
  -r:"$unity_data/MonoBleedingEdge/lib/mono/4.7.1-api/Facades/netstandard.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.CoreModule.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.XRModule.dll" \
  -r:"$unity_data/Managed/UnityEngine/UnityEngine.JSONSerializeModule.dll" \
  Assets/Scripts/PICO_UDP_Sender.cs Assets/Scripts/PICOEndpointConfig.cs
printf '%s\n' 'PICO sender compiled against installed Unity APIs'
