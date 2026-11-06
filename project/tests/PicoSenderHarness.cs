// Behavioral harness: execute the real sender against deterministic XR/time/UDP boundaries.
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Text;

namespace UnityEngine {
    public class MonoBehaviour { public bool enabled = true; }
    public static class Application { public static string persistentDataPath; }
    public struct Vector3 {
        public float x, y, z;
        public static Vector3 zero { get { return new Vector3(); } }
    }
    public struct Quaternion { public float x, y, z, w; public static Quaternion identity { get { return new Quaternion { w = 1 }; } } }
    public static class Time { public static float deltaTime = .034f; public static double realtimeSinceStartupAsDouble; public static float unscaledDeltaTime = .034f; }
    public static class Debug { public static void Log(object o) {} public static void LogError(object o) {} }
    public static class JsonUtility {
        public static object Last;
        public static string ToJson(object data) { Last = data; return "{}"; }
    }
}
namespace UnityEngine.XR {
    public enum XRNode { RightHand }
    public struct InputFeatureUsage<T> { public string name; public InputFeatureUsage(string name) { this.name = name; } }
    public static class CommonUsages {
        public static InputFeatureUsage<UnityEngine.Vector3> devicePosition = new InputFeatureUsage<UnityEngine.Vector3>("position");
        public static InputFeatureUsage<UnityEngine.Quaternion> deviceRotation = new InputFeatureUsage<UnityEngine.Quaternion>("rotation");
        public static InputFeatureUsage<float> trigger = new InputFeatureUsage<float>("trigger");
        public static InputFeatureUsage<float> grip = new InputFeatureUsage<float>("grip");
        public static InputFeatureUsage<bool> isTracked = new InputFeatureUsage<bool>("tracked");
    }
    public struct InputDevice {
        public bool isValid { get { return InputDevices.Connected; } }
        public string name { get { return "Fake right controller"; } }
        public bool TryGetFeatureValue<T>(InputFeatureUsage<T> usage, out T value) {
            object found;
            if (isValid && InputDevices.Features.TryGetValue(usage.name, out found)) { value = (T)found; return true; }
            value = default(T); return false;
        }
    }
    public static class InputDevices {
        public static bool Connected;
        public static int Searches;
        public static Dictionary<string, object> Features = new Dictionary<string, object>();
        public static void GetDevicesAtXRNode(XRNode node, List<InputDevice> devices) {
            Searches++; if (Connected) devices.Add(new InputDevice());
        }
    }
}
namespace System.Net.Sockets {
    public class UdpClient {
        public static int Sends; public static bool Closed;
        public static string LastHost; public static int LastPort;
        public int Send(byte[] bytes, int count, string host, int port) {
            Sends++; LastHost = host; LastPort = port; return count;
        }
        public void Close() { Closed = true; }
    }
}
public static class PicoSenderHarness {
    static object Field(object obj, string name) {
        FieldInfo field = obj.GetType().GetField(name);
        if (field == null) throw new Exception("Missing packet field: " + name);
        return field.GetValue(obj);
    }
    static void Call(object obj, string method) {
        obj.GetType().GetMethod(method, BindingFlags.NonPublic | BindingFlags.Instance).Invoke(obj, null);
    }
    static object Tick(object sender, double time) {
        UnityEngine.Time.realtimeSinceStartupAsDouble = time;
        UnityEngine.JsonUtility.Last = null;
        Call(sender, "Update");
        if (UnityEngine.JsonUtility.Last == null) throw new Exception("Disconnected/tracking-loss heartbeat missing");
        return UnityEngine.JsonUtility.Last;
    }
    static void Check(bool ok, string message) { if (!ok) throw new Exception(message); }
    public static int Main() {
#if UNITY_ANDROID
        string scratch = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "rm75-endpoint-" + Guid.NewGuid().ToString("N"));
        System.IO.Directory.CreateDirectory(scratch);
        UnityEngine.Application.persistentDataPath = scratch;
        try { return RunChecks(); }
        finally { System.IO.Directory.Delete(scratch, true); }
#else
        return RunChecks();
#endif
    }
    static void CheckRate(int fps) {
        var sender=new PICO_UDP_Sender(); Call(sender,"Start");
        int before=System.Net.Sockets.UdpClient.Sends;
        UnityEngine.Time.unscaledDeltaTime=1f/fps;
        for(int i=0;i<fps*2;i++) {
            UnityEngine.Time.realtimeSinceStartupAsDouble=10.0+i/(double)fps;
            Call(sender,"Update");
        }
        int count=System.Net.Sockets.UdpClient.Sends-before;
        Check(count>=99 && count<=101,"Expected 50Hz average at display FPS "+fps+", got "+count/2f);
        before=System.Net.Sockets.UdpClient.Sends;
        UnityEngine.Time.unscaledDeltaTime=.5f; Call(sender,"Update");
        Check(System.Net.Sockets.UdpClient.Sends==before+1,"Stall must not produce a catch-up burst");
        UnityEngine.Time.unscaledDeltaTime=0; Call(sender,"Update");
        Check(System.Net.Sockets.UdpClient.Sends==before+1,"No stale backlog sends after stall");
        Call(sender,"OnDestroy");
    }
    static int RunChecks() {
#if UNITY_ANDROID
        var missingConfig = new PICO_UDP_Sender();
        Call(missingConfig, "Start");
        Check(!missingConfig.enabled && System.Net.Sockets.UdpClient.Sends == 0,
              "Android sender must wait for endpoint configuration");
        string config = System.IO.Path.Combine(UnityEngine.Application.persistentDataPath, PICOEndpointConfig.FileName);
        System.IO.File.WriteAllText(config, "bad-endpoint");
        var invalidConfig = new PICO_UDP_Sender();
        Call(invalidConfig, "Start");
        Check(!invalidConfig.enabled, "Malformed endpoint must disable Android sender");
        System.IO.File.WriteAllText(config, "192.168.50.123:5005\n");
#endif
        var sender = new PICO_UDP_Sender();
        Call(sender, "Start");
        object first = Tick(sender, .034);
#if UNITY_ANDROID
        Check(System.Net.Sockets.UdpClient.LastHost == "192.168.50.123" &&
              System.Net.Sockets.UdpClient.LastPort == 5005,
              "Persistent endpoint did not override old Inspector address");
#endif
        Check(!(bool)Field(first, "valid"), "Disconnected device marked valid");
        string session = (string)Field(first, "session");
        Check(!String.IsNullOrEmpty(session), "Session missing");
        long sequence = (long)Field(first, "sequence");
        object second = Tick(sender, .068);
        Check((long)Field(second, "sequence") > sequence, "Sequence did not increase");
        Check((double)Field(second, "sent_time") > (double)Field(first, "sent_time"), "Sender time did not increase");
        Check(UnityEngine.XR.InputDevices.Searches <= 2, "Controller search is unthrottled");
        UnityEngine.XR.InputDevices.Connected = true;
        var features = UnityEngine.XR.InputDevices.Features;
        features["position"] = new UnityEngine.Vector3 { x = .1f, y = .2f, z = .3f };
        features["rotation"] = UnityEngine.Quaternion.identity;
        features["trigger"] = .2f; features["grip"] = .8f; features["tracked"] = true;
        object tracked = Tick(sender, 1.2);
        Check((bool)Field(tracked, "valid"), "Tracked controller not valid");
        Check((bool)Field(tracked, "device_valid"), "Device validity diagnostic missing");
        Check((bool)Field(tracked, "position_valid"), "Position diagnostic missing");
        Check((bool)Field(tracked, "rotation_valid"), "Rotation diagnostic missing");
        Check((bool)Field(tracked, "grip_valid"), "Grip diagnostic missing");
        Check((bool)Field(tracked, "tracked_valid"), "Tracking feature diagnostic missing");
        Check((bool)Field(tracked, "tracked"), "Tracking value diagnostic missing");
        Check((float)Field(tracked, "x") == .1f, "Position missing");
        Check((string)Field(tracked, "session") == session, "Session changed during run");
        features["tracked"] = false;
        Check(!(bool)Field(Tick(sender, 1.234), "valid"), "Tracking loss not reported");
        features["tracked"] = true; features.Remove("grip");
        Check(!(bool)Field(Tick(sender, 1.268), "valid"), "Missing grip marked valid");
        int searchesBeforeRecovery = UnityEngine.XR.InputDevices.Searches;
        Tick(sender, 2.3);
        Check(UnityEngine.XR.InputDevices.Searches > searchesBeforeRecovery,
              "Feature read failure did not trigger throttled controller reacquisition");
        Call(sender, "OnDestroy");
        Check(System.Net.Sockets.UdpClient.Closed, "UDP not closed");
        CheckRate(72); CheckRate(90);
        Console.WriteLine("PICO sender behavioral checks passed");
        return 0;
    }
}
