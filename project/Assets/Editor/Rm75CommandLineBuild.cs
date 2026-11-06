#if UNITY_EDITOR
using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class Rm75CommandLineBuild
{
    [MenuItem("RM75/Build Android APK")]
    public static void BuildAndroid()
    {
        Rm75ProjectSetup.ConfigureAndOpen();
        RM75DigitalTwinBuilder.ValidateInstalledAssets();

        string output = Environment.GetEnvironmentVariable("RM75_BUILD_PATH");
        if (String.IsNullOrEmpty(output))
            output = Path.Combine(Directory.GetCurrentDirectory(), "Build", "PICO_RM75_Teleop.apk");

        string[] scenes = EditorBuildSettings.scenes
            .Where(scene => scene.enabled)
            .Select(scene => scene.path)
            .ToArray();
        if (scenes.Length == 0)
            throw new InvalidOperationException("No enabled scenes are configured for the Android build.");

        Directory.CreateDirectory(Path.GetDirectoryName(output));
        var options = new BuildPlayerOptions
        {
            scenes = scenes,
            locationPathName = output,
            target = BuildTarget.Android,
            options = BuildOptions.None
        };
        BuildReport report = BuildPipeline.BuildPlayer(options);
        if (report.summary.result != BuildResult.Succeeded)
            throw new InvalidOperationException("Android build failed: " + report.summary.result);
        AssetDatabase.SaveAssets();
        string project = Directory.GetParent(Application.dataPath).FullName;
        string logs = Path.GetFullPath(Path.Combine(project, "../deployment/logs"));
        Directory.CreateDirectory(logs);
        File.WriteAllText(Path.Combine(logs, "apk-build.json"), JsonUtility.ToJson(new ApkReport {
            project = project, bundleVersionCode = PlayerSettings.Android.bundleVersionCode,
            joystickIncluded = true, builtAtUtc = DateTime.UtcNow.ToString("o"),
            apkSha256 = Hash(File.ReadAllBytes(output)), sourcesSha256 = SourceHash(project)
        }, true));
        Console.WriteLine("RM75_APK=" + output);
    }

    static string Hash(byte[] data)
    {
        using (var sha = SHA256.Create())
            return BitConverter.ToString(sha.ComputeHash(data)).Replace("-", "").ToLowerInvariant();
    }

    static string SourceHash(string project)
    {
        var paths = Directory.GetFiles(Path.Combine(project, "Assets"), "*", SearchOption.AllDirectories)
            .Where(path => new[] { ".cs", ".unity", ".prefab" }.Contains(Path.GetExtension(path)))
            .Select(path => path.Substring(project.Length + 1).Replace('\\', '/'))
            .Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json",
                            "ProjectSettings/ProjectSettings.asset" })
            .OrderBy(path => path, StringComparer.Ordinal);
        using (var stream = new MemoryStream())
        {
            foreach (string path in paths)
            {
                byte[] name = Encoding.UTF8.GetBytes(path + "\0");
                byte[] data = File.ReadAllBytes(Path.Combine(project, path));
                stream.Write(name, 0, name.Length);
                stream.Write(data, 0, data.Length);
            }
            return Hash(stream.ToArray());
        }
    }

    [Serializable]
    sealed class ApkReport
    {
        public string project, builtAtUtc, apkSha256, sourcesSha256;
        public int bundleVersionCode;
        public bool joystickIncluded;
    }
}
#endif
