// Add this COMPONENT to a GameObject in the existing PICO Unity project.
// It only SENDS thumbstick data.  The existing 6DoF sender, right-Grip safety,
// and verified tracking metadata MUST remain unchanged.
using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

public class PicoJoystickSender : MonoBehaviour
{
    [Header("Ubuntu machine running rm75_joystick_bridge.py")]
    public string ubuntuIp = "";
    public int joystickPort = 5007;
    [Range(10f, 90f)] public float sendHz = 50f;

    [Serializable]
    private class Frame
    {
        public string session;
        public long sequence;
        public double sent_time;
        public bool valid;
        public float left_x, left_y, right_x, right_y;
    }

    private UdpClient udp;
    private string session;
    private long sequence;
    private float sendTimer;
    private readonly List<InputDevice> devices = new List<InputDevice>();
    private InputDevice left, right;
    private bool warned;
    private double nextSearch;

    private void Start()
    {
#if UNITY_EDITOR
        enabled = false;
        return;
#endif
        // Share the existing deployment configuration. DHCP changes require
        // configure-pico, never another APK build or separate Inspector IP.
#if UNITY_ANDROID
        try
        {
            string host;
            int basePort;
            string path = System.IO.Path.Combine(Application.persistentDataPath,
                                                 PICOEndpointConfig.FileName);
            if (!System.IO.File.Exists(path) ||
                !PICOEndpointConfig.TryParse(System.IO.File.ReadAllText(path), out host, out basePort) ||
                basePort != 5005)
            {
                Debug.LogError("PicoJoystickSender: Configure Ubuntu endpoint :5005 first.");
                enabled = false;
                return;
            }
            ubuntuIp = host;
            joystickPort = 5007;
        }
        catch (Exception e)
        {
            Debug.LogError("PicoJoystickSender: Endpoint read failed: " + e.Message);
            enabled = false;
            return;
        }
#endif
        session = Guid.NewGuid().ToString("N");
        if (string.IsNullOrWhiteSpace(ubuntuIp))
        {
            Debug.LogError("PicoJoystickSender: Set Ubuntu IP in Inspector. No data is sent.");
            enabled = false;
            return;
        }
        try
        {
            udp = new UdpClient();
            udp.Connect(ubuntuIp, joystickPort);
        }
        catch (Exception e)
        {
            Debug.LogError("PicoJoystickSender: UDP init failed: " + e.Message);
            enabled = false;
        }
    }

    private static InputDevice FindController(InputDeviceCharacteristics hand,
        List<InputDevice> list)
    {
        list.Clear();
        InputDevices.GetDevicesWithCharacteristics(
            InputDeviceCharacteristics.Controller | hand, list);
        foreach (var device in list)
            if (device.isValid) return device;
        return default(InputDevice);
    }

    private void Update()
    {
        if (udp == null) return;
        sendTimer += Time.unscaledDeltaTime;
        float period = 1f / Math.Max(10f, Math.Min(90f, sendHz));
        if (sendTimer + 0.000001f < period) return;
        sendTimer = Math.Max(0f, (sendTimer - period) % period);
        if (Time.realtimeSinceStartupAsDouble >= nextSearch)
        {
            nextSearch = Time.realtimeSinceStartupAsDouble + 1.0;
            left = FindController(InputDeviceCharacteristics.Left, devices);
            right = FindController(InputDeviceCharacteristics.Right, devices);
        }
        Vector2 l = Vector2.zero, r = Vector2.zero;
        bool leftTracked = false, rightTracked = false;
        bool valid = left.isValid && right.isValid &&
            left.TryGetFeatureValue(CommonUsages.isTracked, out leftTracked) && leftTracked &&
            right.TryGetFeatureValue(CommonUsages.isTracked, out rightTracked) && rightTracked &&
            left.TryGetFeatureValue(CommonUsages.primary2DAxis, out l) &&
            right.TryGetFeatureValue(CommonUsages.primary2DAxis, out r) &&
            Finite(l.x) && Finite(l.y) && Finite(r.x) && Finite(r.y);
        var frame = new Frame
        {
            session = session,
            sequence = sequence++,
            sent_time = Time.realtimeSinceStartupAsDouble,
            valid = valid,
            left_x = valid ? Mathf.Clamp(l.x, -1f, 1f) : 0f,
            left_y = valid ? Mathf.Clamp(l.y, -1f, 1f) : 0f,
            right_x = valid ? Mathf.Clamp(r.x, -1f, 1f) : 0f,
            right_y = valid ? Mathf.Clamp(r.y, -1f, 1f) : 0f
        };
        if (!valid && !warned)
        {
            Debug.LogWarning("PicoJoystickSender: Joystick not available; no motion permitted.");
            warned = true;
        }
        if (valid) warned = false;
        try
        {
            byte[] packet = Encoding.UTF8.GetBytes(JsonUtility.ToJson(frame));
            udp.Send(packet, packet.Length);
        }
        catch (Exception e)
        {
            if (!warned)
            {
                Debug.LogError("PicoJoystickSender: UDP send failed: " + e.Message);
                warned = true;
            }
        }
    }

    private static bool Finite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private void OnDestroy()
    {
        if (udp != null) { udp.Close(); udp = null; }
    }
}
