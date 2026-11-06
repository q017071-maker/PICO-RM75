using System;
using System.Net;
using System.Net.Sockets;

// The Android app reads this endpoint from its own persistent data directory.
// This keeps DHCP changes independent of APK builds.
public static class PICOEndpointConfig
{
    public const string FileName = "ubuntu_udp.txt";

    public static bool TryParse(string text, out string host, out int port)
    {
        host = null;
        port = 0;
        if (String.IsNullOrWhiteSpace(text))
            return false;
        string[] parts = text.Trim().Split(':');
        IPAddress address;
        int parsedPort;
        if (parts.Length != 2 || !IPAddress.TryParse(parts[0], out address) ||
            address.AddressFamily != AddressFamily.InterNetwork ||
            !Int32.TryParse(parts[1], out parsedPort) || parsedPort < 1 || parsedPort > 65535)
            return false;
        byte[] bytes = address.GetAddressBytes();
        if (bytes[0] == 0 || bytes[0] == 127 || bytes[0] >= 224)
            return false;
        host = address.ToString();
        port = parsedPort;
        return true;
    }
}
