using System;
using System.Reflection;
using System.Web.Script.Serialization;

namespace UnityEngine
{
    [AttributeUsage(AttributeTargets.Field)] public sealed class SerializeField : Attribute { }
    [AttributeUsage(AttributeTargets.Field)] public sealed class HeaderAttribute : Attribute { public HeaderAttribute(string value) { } }
    [AttributeUsage(AttributeTargets.Method)] public sealed class ContextMenu : Attribute { public ContextMenu(string value) { } }
    public class Object { }
    public class MonoBehaviour : Object
    {
        public readonly Transform transform = new Transform();
        protected T[] GetComponentsInChildren<T>(bool includeInactive) { return new T[0]; }
    }
    public struct Vector3
    {
        public float x, y, z;
        public Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public static Vector3 forward { get { return new Vector3(0, 0, 1); } }
        public static Vector3 up { get { return new Vector3(0, 1, 0); } }
        public float sqrMagnitude { get { return x * x + y * y + z * z; } }
        public Vector3 normalized {
            get {
                float length = (float)Math.Sqrt(sqrMagnitude);
                return length > 1e-8f ? new Vector3(x / length, y / length, z / length) : new Vector3();
            }
        }
        public static Vector3 operator +(Vector3 a, Vector3 b) { return new Vector3(a.x + b.x, a.y + b.y, a.z + b.z); }
        public static Vector3 operator *(Vector3 a, float b) { return new Vector3(a.x * b, a.y * b, a.z * b); }
    }
    public struct Quaternion
    {
        public float x, y, z, w;
        public static Quaternion identity { get { return new Quaternion { w = 1 }; } }
        public static Quaternion AngleAxis(float angle, Vector3 axis) { return identity; }
        public static Quaternion Euler(float x, float y, float z) { return identity; }
        public static Quaternion LookRotation(Vector3 forward, Vector3 up) { return identity; }
        public static Quaternion operator *(Quaternion a, Quaternion b) { return identity; }
    }
    public class Transform
    {
        public Vector3 position;
        public Vector3 forward = Vector3.forward;
        public Quaternion rotation = Quaternion.identity;
        public Quaternion localRotation = Quaternion.identity;
    }
    public class Camera : Object { public static Camera main; public readonly Transform transform = new Transform(); }
    public struct Color
    {
        public float r, g, b, a;
        public Color(float r, float g, float b, float a = 1) { this.r = r; this.g = g; this.b = b; this.a = a; }
    }
    public class Renderer : Object { public void SetPropertyBlock(MaterialPropertyBlock block) { } }
    public class MaterialPropertyBlock { public void SetColor(int id, Color color) { } }
    public static class Shader { public static int PropertyToID(string name) { return 1; } }
    public static class Time { public static float deltaTime = .016f; public static double realtimeSinceStartupAsDouble; }
    public static class Mathf
    {
        public static float Exp(float value) { return (float)Math.Exp(value); }
        public static float Repeat(float value, float length) { return value - (float)Math.Floor(value / length) * length; }
    }
    public static class Debug
    {
        public static void Log(object value) { }
        public static void LogWarning(object value) { }
        public static void LogError(object value) { }
    }
    public static class JsonUtility
    {
        public static T FromJson<T>(string json) { return new JavaScriptSerializer().Deserialize<T>(json); }
    }
}

public static class Rm75DigitalTwinHarness
{
    static void Check(bool condition, string message)
    {
        if (!condition) throw new Exception(message);
    }

    static void Near(float actual, float expected, string message)
    {
        if (Math.Abs(actual - expected) > 0.0001f)
            throw new Exception(message + ": " + actual + " != " + expected);
    }

    static string Packet(long sequence, string joints = "[0.416,9.003,0.346,-34.931,-1.889,-56.03,-0.179]",
                         string type = "rm75_state", int version = 1,
                         double sentTime = 3.0, bool connected = true)
    {
        return "{\"type\":\"" + type + "\",\"version\":" + version +
            ",\"sequence\":" + sequence + ",\"sent_time\":" +
            sentTime.ToString(System.Globalization.CultureInfo.InvariantCulture) +
            ",\"connected\":" + connected.ToString().ToLowerInvariant() +
            ",\"joint_deg\":" + joints + "}";
    }

    static void ProtocolChecks()
    {
        var protocol = new RM75StateProtocol();
        RM75StateSnapshot snapshot;
        Check(protocol.TryAccept(Packet(0), 10.0, out snapshot), "Valid packet rejected");
        Check(snapshot.Sequence == 0 && snapshot.JointDegrees.Length == 7, "Snapshot fields missing");
        Near(snapshot.JointDegrees[3], -34.931f, "Joint value changed");
        Check(!snapshot.IsStale(10.59, .60), "Snapshot stale before boundary");
        Check(snapshot.IsStale(10.60, .60), "Snapshot fresh at stale boundary");
        Check(!protocol.TryAccept(Packet(0), 10.1, out snapshot), "Duplicate sequence accepted");
        Check(!protocol.TryAccept(Packet(-1), 10.2, out snapshot), "Negative sequence accepted");
        Check(!protocol.TryAccept(Packet(1, type: "other"), 10.2, out snapshot), "Wrong type accepted");
        Check(!protocol.TryAccept(Packet(1, version: 2), 10.2, out snapshot), "Wrong version accepted");
        Check(!protocol.TryAccept(Packet(1, joints: "[1,2,3,4,5,6]"), 10.2, out snapshot), "Short joint list accepted");
        Check(!protocol.TryAccept(Packet(1, joints: "[1,2,3,4,5,6,7,8]"), 10.2, out snapshot), "Long joint list accepted");
        Check(!protocol.TryAccept(Packet(1, joints: "[1,2,3,4,5,6,1e999]"), 10.2, out snapshot), "Infinite joint accepted");
        Check(!protocol.TryAccept(Packet(1, connected: false), 10.2, out snapshot), "Disconnected packet accepted");
        Check(protocol.TryAccept(Packet(2), 10.3, out snapshot), "Valid packet after rejects not accepted");
        Check(snapshot.Sequence == 2, "Latest valid snapshot not retained");
    }

    static void AngleAndPlacementChecks()
    {
        Near(RM75JointAnimator.ShortestDeltaDegrees(179, -179), 2, "Positive wrap incorrect");
        Near(RM75JointAnimator.ShortestDeltaDegrees(-179, 179), -2, "Negative wrap incorrect");
        var camera = new UnityEngine.Transform();
        camera.position = new UnityEngine.Vector3(1, 2, 3);
        camera.forward = new UnityEngine.Vector3(3, 4, 0);
        var placement = new RM75FixedPlacement();
        placement.Place(camera);
        Near(placement.transform.position.x, 2.2f, "Placement distance incorrect");
        Near(placement.transform.position.y, 1.25f, "Placement vertical offset incorrect");
        Near(placement.transform.position.z, 3f, "Placement horizontal projection incorrect");
        camera.position = new UnityEngine.Vector3(20, 20, 20);
        Near(placement.transform.position.x, 2.2f, "Placement followed camera after initialization");
    }

    public static int Main()
    {
        ProtocolChecks();
        AngleAndPlacementChecks();
        Console.WriteLine("RM75 digital twin behavioral checks passed");
        return 0;
    }
}
