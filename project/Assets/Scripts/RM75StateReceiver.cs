using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using Stopwatch = System.Diagnostics.Stopwatch;

public sealed class RM75StateReceiver : MonoBehaviour
{
    [SerializeField] int listenPort = 5006;
    [SerializeField] RM75JointAnimator animator;

    readonly object snapshotLock = new object();
    readonly RM75StateProtocol protocol = new RM75StateProtocol();
    UdpClient udp;
    Thread worker;
    volatile bool running;
    RM75StateSnapshot latest;
    bool hasLatest;
    bool reportedFirstSnapshot;

    public static double MonotonicSeconds
    {
        get { return (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency; }
    }

    void OnEnable()
    {
        StartReceiver();
    }

    void Update()
    {
        RM75StateSnapshot snapshot;
        if (animator != null && TryGetLatest(out snapshot))
        {
            animator.SetSnapshot(snapshot);
            if (!reportedFirstSnapshot)
            {
                reportedFirstSnapshot = true;
                Debug.Log("[RM75_VIEW] First robot state received");
            }
        }
    }

    public bool TryGetLatest(out RM75StateSnapshot snapshot)
    {
        lock (snapshotLock)
        {
            snapshot = latest;
            return hasLatest;
        }
    }

    void StartReceiver()
    {
        if (running)
            return;
        if (listenPort < 1 || listenPort > 65535)
        {
            Debug.LogError("[RM75_VIEW] State port must be between 1 and 65535");
            return;
        }
        try
        {
            udp = new UdpClient(new IPEndPoint(IPAddress.Any, listenPort));
            udp.Client.ReceiveTimeout = 250;
        }
        catch (Exception exc)
        {
            Debug.LogError("[RM75_VIEW] Cannot bind state UDP " + listenPort + ": " + exc.Message);
            udp = null;
            return;
        }
        running = true;
        worker = new Thread(ReceiveLoop);
        worker.IsBackground = true;
        worker.Name = "RM75 state receiver";
        worker.Start();
        Debug.Log("[RM75_VIEW] Listening for robot state on UDP " + listenPort);
    }

    void ReceiveLoop()
    {
        var peer = new IPEndPoint(IPAddress.Any, 0);
        while (running)
        {
            try
            {
                byte[] bytes = udp.Receive(ref peer);
                string json = Encoding.UTF8.GetString(bytes);
                RM75StateSnapshot snapshot;
                if (!protocol.TryAccept(json, MonotonicSeconds, out snapshot))
                    continue;
                lock (snapshotLock)
                {
                    latest = snapshot;
                    hasLatest = true;
                }
            }
            catch (SocketException exc)
            {
                if (!running)
                    return;
                if (exc.SocketErrorCode != SocketError.TimedOut &&
                    exc.SocketErrorCode != SocketError.WouldBlock)
                {
                    Debug.LogWarning("[RM75_VIEW] State UDP receive failed: " + exc.Message);
                    return;
                }
            }
            catch (ObjectDisposedException)
            {
                return;
            }
            catch (Exception exc)
            {
                if (running)
                    Debug.LogWarning("[RM75_VIEW] State packet rejected: " + exc.Message);
            }
        }
    }

    void StopReceiver()
    {
        running = false;
        UdpClient socket = udp;
        udp = null;
        if (socket != null)
            socket.Close();
        Thread thread = worker;
        worker = null;
        if (thread != null && thread.IsAlive && !thread.Join(500))
            Debug.LogWarning("[RM75_VIEW] State receiver thread did not stop within 500 ms");
    }

    void OnApplicationPause(bool paused)
    {
        if (paused)
            StopReceiver();
        else
            StartReceiver();
    }

    void OnDisable()
    {
        StopReceiver();
    }

    void OnDestroy()
    {
        StopReceiver();
    }

    void OnApplicationQuit()
    {
        StopReceiver();
    }
}
