from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]/'project'; data=Path.home()/'Unity/Hub/Editor/2022.3.62f3/Editor/Data'
refs=[data/'MonoBleedingEdge/lib/mono/4.7.1-api/Facades/netstandard.dll',data/'PlaybackEngines/AndroidPlayer/UnityEditor.Android.Extensions.dll',*[data/'Managed/UnityEngine'/n for n in ['UnityEngine.CoreModule.dll','UnityEngine.XRModule.dll','UnityEngine.JSONSerializeModule.dll','UnityEditor.CoreModule.dll','UnityEditor.SceneViewModule.dll','UnityEditor.UnityConnectModule.dll']],root/'Library/ScriptAssemblies/Unity.XR.Management.dll',root/'Library/ScriptAssemblies/Unity.XR.Management.Editor.dll',root/'Library/ScriptAssemblies/Assembly-CSharp.dll',root/'Library/ScriptAssemblies/Assembly-CSharp-Editor.dll']
with tempfile.TemporaryDirectory(prefix='rm75-editor-check-') as tmp:
 args=[str(data/'MonoBleedingEdge/bin/mcs'),'-target:library','-define:UNITY_EDITOR','-nowarn:0436',f'-out:{tmp}/new-editor.dll',*[f'-r:{p}' for p in refs],*[str(root/'Assets'/n) for n in ['Scripts/PICO_UDP_Sender.cs','Scripts/PicoJoystickSender.cs','Scripts/PICOEndpointConfig.cs','Editor/Rm75ProjectSetup.cs','Editor/Rm75CommandLineBuild.cs']]]
 subprocess.run(args,check=True)
print('New editor configuration and build scripts compiled against installed Unity APIs (native scene execution still pending).')
