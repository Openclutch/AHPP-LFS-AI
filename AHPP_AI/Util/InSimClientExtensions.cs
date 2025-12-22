using System;
using InSimDotNet;
using InSimDotNet.Packets;

namespace AHPP_AI.Util
{
    /// <summary>
    ///     Synchronous helpers for the asynchronous InSimClient API.
    /// </summary>
    public static class InSimClientExtensions
    {
        public static void Initialize(this InSimClient client, InSimSettings settings)
        {
            client.InitializeAsync(settings).GetAwaiter().GetResult();
        }

        public static void Send(this InSimClient client, ISendable packet)
        {
            client.SendAsync(packet).GetAwaiter().GetResult();
        }

        public static void Send(this InSimClient client, params ISendable[] packets)
        {
            client.SendAsync(packets).GetAwaiter().GetResult();
        }

        public static void Send(this InSimClient client, string message, params object[] args)
        {
            client.SendAsync(message, args).GetAwaiter().GetResult();
        }

        public static void DisconnectSafe(this InSimClient client)
        {
            try
            {
                client.Disconnect();
            }
            catch (Exception)
            {
                // Ignore shutdown race conditions
            }
        }
    }
}
