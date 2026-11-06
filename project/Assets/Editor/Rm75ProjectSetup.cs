#if UNITY_EDITOR
using System;
using System.IO;
using System.Linq;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using UnityEditor;
using UnityEditor.Android;
using UnityEditor.SceneManagement;
using UnityEditor.XR.Management;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.XR.Management;

public static class Rm75ProjectSetup
{
    const string ScenePath = "Assets/Scenes/SampleScene.unity";

    [MenuItem("RM75/Configure Current Project")]
    public static void ConfigureAndOpen()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Exit Play mode before configuring the project.");
        string android = Path.Combine(EditorApplication.applicationContentsPath,
                                       "PlaybackEngines", "AndroidPlayer");
        RequireFile(Path.Combine(android, "SDK", "platform-tools", "adb"));
        RequireFile(Path.Combine(android, "SDK", "platforms", "android-34", "android.jar"));
        RequireFile(Path.Combine(android, "NDK", "source.properties"));
        RequireFile(Path.Combine(android, "OpenJDK", "bin", "java"));
        AndroidExternalToolsSettings.sdkRootPath = Path.Combine(android, "SDK");
        AndroidExternalToolsSettings.ndkRootPath = Path.Combine(android, "NDK");
        AndroidExternalToolsSettings.jdkRootPath = Path.Combine(android, "OpenJDK");
        AndroidExternalToolsSettings.gradlePath = Path.Combine(android, "Tools", "gradle");

        if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.Android &&
            !EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Android, BuildTarget.Android))
            throw new InvalidOperationException("Cannot switch the current project to Android.");
        EditorUserBuildSettings.androidBuildSystem = AndroidBuildSystem.Gradle;
        EditorUserBuildSettings.exportAsGoogleAndroidProject = false;
        EditorUserBuildSettings.buildAppBundle = false;
        PlayerSettings.productName = "PICO_RM75_Teleop";
        PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Android,
                                               "com.DefaultCompany.PICO_RM75_Teleop");
        PlayerSettings.bundleVersion = "2026.11.06-joystick-50hz";
        PlayerSettings.Android.bundleVersionCode = 20261107;
        PlayerSettings.Android.minSdkVersion = (AndroidSdkVersions)29;
        PlayerSettings.Android.targetSdkVersion = (AndroidSdkVersions)34;
        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
        PlayerSettings.Android.forceInternetPermission = true;
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Android, ScriptingImplementation.IL2CPP);
        PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.Android, false);
        PlayerSettings.SetGraphicsAPIs(BuildTarget.Android, new[] { GraphicsDeviceType.OpenGLES3 });
        EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };

        XRGeneralSettings xr = XRGeneralSettingsPerBuildTarget.XRGeneralSettingsForBuildTarget(
            BuildTargetGroup.Android);
        if (xr == null || xr.Manager == null ||
            !xr.Manager.activeLoaders.Any(loader => loader != null &&
                                         loader.GetType().FullName == "Unity.XR.PXR.PXR_Loader"))
            throw new InvalidOperationException("Android PICO XR loader is not configured.");
        xr.InitManagerOnStart = true;
        EditorUtility.SetDirty(xr);

        // The migration scene can omit the prefab despite containing all model
        // files. Use the existing native builder and its actual-asset checks.
        RM75DigitalTwinEditorChecks.Run();
        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        PICO_UDP_Sender[] senders = scene.GetRootGameObjects()
            .SelectMany(root => root.GetComponentsInChildren<PICO_UDP_Sender>(true)).ToArray();
        if (senders.Length != 1 || !senders[0].gameObject.activeInHierarchy || !senders[0].enabled)
            throw new InvalidOperationException("SampleScene requires one enabled PICO UDP sender.");
        string host = DetectWifiAddress();
        // Android still requires ubuntu_udp.txt; the Inspector mirrors the
        // host address for clarity and cannot bypass that runtime check.
        senders[0].ubuntuIP = host;
        senders[0].port = 5005;
        senders[0].sendHz = 50f;
        EditorUtility.SetDirty(senders[0]);
        PicoJoystickSender[] sticks = scene.GetRootGameObjects()
            .SelectMany(root => root.GetComponentsInChildren<PicoJoystickSender>(true)).ToArray();
        if (sticks.Length > 1)
            throw new InvalidOperationException("Duplicate PicoJoystickSender components.");
        PicoJoystickSender stick = sticks.Length == 1 ? sticks[0] :
            new GameObject("PICO_JoystickSender").AddComponent<PicoJoystickSender>();
        if (!stick.enabled || !stick.gameObject.activeInHierarchy)
            throw new InvalidOperationException("PicoJoystickSender must be enabled.");
        stick.ubuntuIp = host;
        stick.joystickPort = 5007;
        stick.sendHz = 50f;
        EditorUtility.SetDirty(stick);
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();

        GameObject model = GameObject.Find("RM75_DigitalTwin");
        if (!Application.isBatchMode && model != null)
        {
            Selection.activeGameObject = model;
            if (SceneView.lastActiveSceneView != null)
                SceneView.lastActiveSceneView.FrameSelected();
        }
        string logs = Path.GetFullPath(Path.Combine(Application.dataPath, "../../deployment/logs"));
        Directory.CreateDirectory(logs);
        File.WriteAllText(Path.Combine(logs, "unity-project-config.json"),
            JsonUtility.ToJson(new ConfigReport {
                project = Directory.GetParent(Application.dataPath).FullName,
                unity = Application.unityVersion,
                scene = ScenePath, androidTargetSdk = 34, wifiIPv4 = host,
                modelValidated = true, joystickConfigured = true,
                joystickPort = 5007, inputRateHz = 50, sdkPath = AndroidExternalToolsSettings.sdkRootPath,
                ndkPath = AndroidExternalToolsSettings.ndkRootPath,
                jdkPath = AndroidExternalToolsSettings.jdkRootPath,
                configuredAtUtc = DateTime.UtcNow.ToString("o")
            }, true));
        Debug.Log("[RM75_SETUP] Latest project configured. Scene is open; Play mode was not started.");
    }

    static string DetectWifiAddress()
    {
        try
        {
            string[] hosts = NetworkInterface.GetAllNetworkInterfaces()
                .Where(nic => Directory.Exists("/sys/class/net/" + nic.Name + "/wireless"))
                .SelectMany(nic => nic.GetIPProperties().UnicastAddresses)
                .Where(address => address.Address.AddressFamily == AddressFamily.InterNetwork)
                .Select(address => address.Address.ToString()).ToArray();
            if (hosts.Length == 1)
                return hosts[0];
        }
        catch (Exception exception)
        {
            Debug.LogWarning("[RM75_SETUP] Cannot detect Wi-Fi IPv4: " + exception.Message);
        }
        return "";
    }

    static void RequireFile(string path)
    {
        if (!File.Exists(path))
            throw new FileNotFoundException("Required Android tool is missing", path);
    }

    [Serializable]
    sealed class ConfigReport
    {
        public string project, unity, scene, wifiIPv4, sdkPath, ndkPath, jdkPath, configuredAtUtc;
        public int androidTargetSdk, joystickPort, inputRateHz;
        public bool modelValidated, joystickConfigured;
    }
}
#endif
