using System;

public static class PicoEndpointHarness
{
    public static int Main()
    {
        string host;
        int port;
        if (!PICOEndpointConfig.TryParse("192.168.50.123:5005\n", out host, out port) ||
            host != "192.168.50.123" || port != 5005)
            throw new Exception("Valid DHCP endpoint rejected");
        foreach (string text in new[] { "", "192.168.1.2", "127.0.0.1:5005",
                    "0.0.0.0:5005", "224.1.1.1:5005", "::1:5005", "host:5005",
                    "192.168.1.2:0", "192.168.1.2:65536", "192.168.1.2:5005:bad" })
            if (PICOEndpointConfig.TryParse(text, out host, out port))
                throw new Exception("Unsafe or malformed endpoint accepted: " + text);
        Console.WriteLine("PICO endpoint configuration checks passed");
        return 0;
    }
}
