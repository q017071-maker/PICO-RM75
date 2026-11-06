using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

public class PICO_UDP_Sender : MonoBehaviour
{
    // Existing Inspector/scene values take precedence over this default.
    public string ubuntuIP = "192.168.50.177";
    public int port = 5005;
    public float sendHz = 50f;

    private UdpClient udpClient;
    private InputDevice rightController;
    private float sendTimer;
    private long sequence;
    private string session;
    private double nextControllerSearch;
    private bool controllerFeaturesValid;

    [Serializable]
    public class ControllerData
    {
        public float x, y, z;
        public float qx, qy, qz, qw;
        public float trigger, grip;
        public bool valid;
        public bool device_valid, position_valid, rotation_valid, grip_valid;
        public bool tracked_valid, tracked;
        public string session;
        public long sequence;
        public double sent_time;
    }

    void Start()
    {
#if UNITY_EDITOR
        // Unity Editor 只负责显示数字孪生。
        // 不允许 Editor 再发送一套 UDP 5005，
        // 否则会和真正的 PICO APK 抢占/干扰遥操作数据源。
        Debug.Log("[RM75_PICO] Unity Editor detected: PICO UDP sender disabled.");
        enabled = false;
        return;
#endif

#if UNITY_ANDROID
        // Written by the PC deploy command after Wi-Fi is connected.
        string endpointPath = System.IO.Path.Combine(
            Application.persistentDataPath, PICOEndpointConfig.FileName);
        try
        {
            string host;
            int configuredPort;
            if (!System.IO.File.Exists(endpointPath) ||
                !PICOEndpointConfig.TryParse(System.IO.File.ReadAllText(endpointPath),
                                            out host, out configuredPort))
            {
                Debug.LogError("[RM75_PICO] Configure Ubuntu endpoint first: " + endpointPath);
                enabled = false;
                return;
            }
            ubuntuIP = host;
            port = configuredPort;
        }
        catch (Exception e)
        {
            Debug.LogError("[RM75_PICO] Cannot read Ubuntu endpoint: " + e.Message);
            enabled = false;
            return;
        }
#endif

        session = Guid.NewGuid().ToString("N");
        udpClient = new UdpClient();

        FindRightController();

        Debug.Log("[RM75_PICO] UDP sender: " + ubuntuIP + ":" + port);
    }

    void FindRightController()
    {
        nextControllerSearch = Time.realtimeSinceStartupAsDouble + 1.0;

        var devices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, devices);

        if (devices.Count > 0)
        {
            rightController = devices[0];

            Debug.Log(
                "[RM75_PICO] Right controller: " +
                rightController.name
            );
        }
        else
        {
            Debug.Log("[RM75_PICO] Waiting for right controller");
        }
    }

    void Update()
    {
        sendTimer += Time.unscaledDeltaTime;

        float period = 1f / Math.Max(10f, Math.Min(90f, sendHz));
        if (sendTimer + 0.000001f < period)
            return;

        // Keep frame-time remainder for 50Hz average on a 72/90Hz display.
        // Sample once per Update; do not send stale catch-up bursts after stalls.
        sendTimer = Math.Max(0f, (sendTimer - period) % period);

        double now = Time.realtimeSinceStartupAsDouble;

        if ((!rightController.isValid || !controllerFeaturesValid) &&
            now >= nextControllerSearch)
        {
            FindRightController();
        }

        Vector3 pos = Vector3.zero;
        Quaternion rot = Quaternion.identity;

        float trigger = 0f;
        float grip = 0f;

        bool tracked = false;

        bool posOK = false;
        bool rotOK = false;
        bool gripOK = false;
        bool trackedOK = false;

        if (rightController.isValid)
        {
            posOK = rightController.TryGetFeatureValue(
                CommonUsages.devicePosition,
                out pos
            );

            rotOK = rightController.TryGetFeatureValue(
                CommonUsages.deviceRotation,
                out rot
            );

            gripOK = rightController.TryGetFeatureValue(
                CommonUsages.grip,
                out grip
            );

            trackedOK = rightController.TryGetFeatureValue(
                CommonUsages.isTracked,
                out tracked
            );

            rightController.TryGetFeatureValue(
                CommonUsages.trigger,
                out trigger
            );
        }

        controllerFeaturesValid =
            rightController.isValid &&
            posOK &&
            rotOK &&
            gripOK &&
            trackedOK;

        var data = new ControllerData
        {
            x = pos.x,
            y = pos.y,
            z = pos.z,

            qx = rot.x,
            qy = rot.y,
            qz = rot.z,
            qw = rot.w,

            trigger = trigger,
            grip = grip,

            valid = controllerFeaturesValid && tracked,

            device_valid = rightController.isValid,
            position_valid = posOK,
            rotation_valid = rotOK,
            grip_valid = gripOK,

            tracked_valid = trackedOK,
            tracked = tracked,

            session = session,
            sequence = sequence++,
            sent_time = now
        };

        // Send invalid heartbeats too,
        // so tracking loss reaches the receiver immediately.
        string json = JsonUtility.ToJson(data);

        byte[] bytes = Encoding.UTF8.GetBytes(json);

        try
        {
            udpClient.Send(
                bytes,
                bytes.Length,
                ubuntuIP,
                port
            );

            if (sequence % 30 == 0)
            {
                Debug.Log(
                    "[RM75_PICO] UDP: " + json
                );
            }
        }
        catch (Exception e)
        {
            Debug.LogError(
                "[RM75_PICO] UDP error: " +
                e.Message
            );
        }
    }

    void OnDestroy()
    {
        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }
    }
}
