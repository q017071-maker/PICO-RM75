using System;
using UnityEngine;

[Serializable]
public sealed class RM75StatePacket
{
    public string type;
    public int version;
    public long sequence;
    public double sent_time;
    public bool connected;
    public float[] joint_deg;
}

public struct RM75StateSnapshot
{
    public long Sequence { get; private set; }
    public double SentTime { get; private set; }
    public double ReceivedAt { get; private set; }
    public float[] JointDegrees { get; private set; }

    public RM75StateSnapshot(long sequence, double sentTime,
                             double receivedAt, float[] jointDegrees)
    {
        Sequence = sequence;
        SentTime = sentTime;
        ReceivedAt = receivedAt;
        JointDegrees = (float[])jointDegrees.Clone();
    }

    public bool IsStale(double now, double staleAfter)
    {
        if (!IsFinite(now) || !IsFinite(staleAfter) || staleAfter <= 0)
            return true;
        return now >= ReceivedAt + staleAfter;
    }

    static bool IsFinite(double value)
    {
        return !Double.IsNaN(value) && !Double.IsInfinity(value);
    }
}

public sealed class RM75StateProtocol
{
    long lastSequence = -1;

    public bool TryAccept(string json, double receivedAt,
                          out RM75StateSnapshot snapshot)
    {
        snapshot = default(RM75StateSnapshot);
        if (String.IsNullOrEmpty(json) || !IsFinite(receivedAt) || receivedAt < 0)
            return false;

        RM75StatePacket packet;
        try
        {
            packet = JsonUtility.FromJson<RM75StatePacket>(json);
        }
        catch (Exception)
        {
            return false;
        }

        if (packet == null || packet.type != "rm75_state" || packet.version != 1 ||
            !packet.connected || packet.sequence < 0 || packet.sequence <= lastSequence ||
            !IsFinite(packet.sent_time) || packet.sent_time < 0 ||
            packet.joint_deg == null || packet.joint_deg.Length != 7)
            return false;

        for (int index = 0; index < packet.joint_deg.Length; index++)
        {
            if (Single.IsNaN(packet.joint_deg[index]) ||
                Single.IsInfinity(packet.joint_deg[index]))
                return false;
        }

        snapshot = new RM75StateSnapshot(packet.sequence, packet.sent_time,
                                         receivedAt, packet.joint_deg);
        lastSequence = packet.sequence;
        return true;
    }

    static bool IsFinite(double value)
    {
        return !Double.IsNaN(value) && !Double.IsInfinity(value);
    }
}
