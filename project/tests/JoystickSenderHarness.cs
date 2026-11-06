// Real sender behavior with fake XR and UDP boundaries; no hardware or network.
using System;
using System.Collections.Generic;
using System.Reflection;
namespace UnityEngine {
    public class MonoBehaviour { public bool enabled = true; }
    public class HeaderAttribute : Attribute { public HeaderAttribute(string s) {} }
    public class SerializeField : Attribute {}
    public class RangeAttribute : Attribute { public RangeAttribute(float a, float b) {} }
    public struct Vector2 { public float x,y; public static Vector2 zero { get { return new Vector2(); } } }
    public static class Time { public static float unscaledTime; public static float unscaledDeltaTime; public static double realtimeSinceStartupAsDouble; }
    public static class Application { public static string persistentDataPath; }
    public static class Mathf { public static float Max(float a,float b) { return Math.Max(a,b); } public static float Clamp(float v,float a,float b) { return Math.Max(a,Math.Min(b,v)); } }
    public static class Debug { public static void LogError(object s) {} public static void LogWarning(object s) {} }
    public static class JsonUtility { public static object Last; public static string ToJson(object o) { Last=o; return "{}"; } }
}
namespace UnityEngine.XR {
    [Flags] public enum InputDeviceCharacteristics { Controller=1, Left=2, Right=4 }
    public struct InputFeatureUsage<T> { public string name; public InputFeatureUsage(string n) { name=n; } }
    public static class CommonUsages {
        public static InputFeatureUsage<UnityEngine.Vector2> primary2DAxis = new InputFeatureUsage<UnityEngine.Vector2>("axis");
        public static InputFeatureUsage<bool> isTracked = new InputFeatureUsage<bool>("tracked");
    }
    public struct InputDevice {
        public int hand;
        public bool isValid { get { return hand!=0 && InputDevices.Connected[hand]; } }
        public bool TryGetFeatureValue<T>(InputFeatureUsage<T> use, out T value) {
            object data; if (isValid && InputDevices.Features[hand].TryGetValue(use.name,out data)) { value=(T)data; return true; }
            value=default(T); return false;
        }
    }
    public static class InputDevices {
        public static bool[] Connected={false,true,true};
        public static Dictionary<string,object>[] Features={null,new Dictionary<string,object>(),new Dictionary<string,object>()};
        public static void GetDevicesWithCharacteristics(InputDeviceCharacteristics kind,List<InputDevice> list) {
            if ((kind & InputDeviceCharacteristics.Controller)==0) throw new Exception("Controller characteristic required");
            list.Add(new InputDevice {hand=(kind & InputDeviceCharacteristics.Left)!=0 ? 1 : 2});
        }
    }
}
namespace System.Net.Sockets {
    public class UdpClient {
        public static int Sends; public static string Host; public static int Port;
        public void Connect(string host,int port) { Host=host; Port=port; }
        public void Send(byte[] b,int n) { Sends++; }
        public void Close() {}
    }
}
public static class JoystickSenderHarness {
    static void Check(bool ok,string message) { if(!ok) throw new Exception(message); }
    static void Call(object o,string name) { o.GetType().GetMethod(name,BindingFlags.Instance|BindingFlags.NonPublic).Invoke(o,null); }
    static object Field(object o,string name) { return o.GetType().GetField(name).GetValue(o); }
    static object Tick(object sender,float at) {
        UnityEngine.Time.unscaledDeltaTime=at-UnityEngine.Time.unscaledTime;
        UnityEngine.Time.unscaledTime=at; UnityEngine.Time.realtimeSinceStartupAsDouble=at;
        UnityEngine.JsonUtility.Last=null; Call(sender,"Update"); return UnityEngine.JsonUtility.Last;
    }
    public static int Main() {
        string dir=System.IO.Path.Combine(System.IO.Path.GetTempPath(),"rm75-stick-"+Guid.NewGuid().ToString("N"));
        System.IO.Directory.CreateDirectory(dir); UnityEngine.Application.persistentDataPath=dir;
        try { Run(dir); Console.WriteLine("Joystick sender behavioral checks passed"); return 0; }
        finally { System.IO.Directory.Delete(dir,true); }
    }
    static void CheckRate(int fps) {
        var sender=new PicoJoystickSender(); sender.ubuntuIp="192.168.50.10";
        Call(sender,"Start");
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
    static void Run(string dir) {
#if UNITY_EDITOR
        var editor=new PicoJoystickSender(); Call(editor,"Start"); Check(!editor.enabled && System.Net.Sockets.UdpClient.Sends==0,"Editor must never send");
#else
#if UNITY_ANDROID
        var missing=new PicoJoystickSender(); Call(missing,"Start"); Check(!missing.enabled,"Missing endpoint must disable");
        string path=System.IO.Path.Combine(dir,PICOEndpointConfig.FileName);
        System.IO.File.WriteAllText(path,"192.168.50.10:5007\n");
        var wrong=new PicoJoystickSender(); Call(wrong,"Start"); Check(!wrong.enabled,"Endpoint must describe base port5005");
        System.IO.File.WriteAllText(path,"192.168.50.10:5005\n");
#endif
        var s=new PicoJoystickSender(); s.ubuntuIp="192.168.50.10"; Call(s,"Start");
        Check(s.enabled && System.Net.Sockets.UdpClient.Port==5007,"Wrong joystick endpoint");
        Check(System.Net.Sockets.UdpClient.Host=="192.168.50.10","Wrong host");
        UnityEngine.XR.InputDevices.Features[1]["axis"]=new UnityEngine.Vector2 {x=.4f,y=-.5f};
        UnityEngine.XR.InputDevices.Features[2]["axis"]=new UnityEngine.Vector2 {x=-.6f,y=.7f};
        UnityEngine.XR.InputDevices.Features[1]["tracked"]=true; UnityEngine.XR.InputDevices.Features[2]["tracked"]=true;
        object a=Tick(s,1); Check((bool)Field(a,"valid"),"Axes must be valid");
        Check((float)Field(a,"left_x")==.4f && (float)Field(a,"left_y")==-.5f && (float)Field(a,"right_x")==-.6f && (float)Field(a,"right_y")==.7f,"Left/right axes crossed");
        Check((long)Field(a,"sequence")==0 && (double)Field(a,"sent_time")==1 && !String.IsNullOrEmpty((string)Field(a,"session")),"Missing metadata");
        UnityEngine.XR.InputDevices.Features[1]["tracked"]=false;
        object b=Tick(s,1.04f); Check(!(bool)Field(b,"valid") && (float)Field(b,"right_y")==0,"Tracking loss must send invalid zero heartbeat");
        UnityEngine.XR.InputDevices.Features[1]["tracked"]=true; UnityEngine.XR.InputDevices.Features[2].Remove("axis");
        Check(!(bool)Field(Tick(s,1.08f),"valid"),"Missing primary2DAxis must invalidate");
        UnityEngine.XR.InputDevices.Features[2]["axis"]=new UnityEngine.Vector2 {x=float.NaN};
        Check(!(bool)Field(Tick(s,1.12f),"valid"),"NaN axes must invalidate");
        UnityEngine.XR.InputDevices.Features[2]["axis"]=new UnityEngine.Vector2(); UnityEngine.XR.InputDevices.Connected[1]=false;
        Check(!(bool)Field(Tick(s,1.16f),"valid"),"Disconnect must invalidate");
        UnityEngine.XR.InputDevices.Connected[1]=true; object c=Tick(s,2.2f);
        Check((bool)Field(c,"valid") && (long)Field(c,"sequence")==5 && (string)Field(c,"session")== (string)Field(a,"session"),"Reconnect metadata continuity failed");
        Call(s,"OnDestroy");
        CheckRate(72); CheckRate(90);
#endif
    }
}
