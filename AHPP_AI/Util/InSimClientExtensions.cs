using System;
using System.Collections.Generic;
using System.Threading;
using InSimDotNet;
using InSimDotNet.Packets;

namespace AHPP_AI.Util
{
    /// <summary>
    ///     Synchronous helpers for the asynchronous InSimClient API.
    /// </summary>
    public static class InSimClientExtensions
    {
        private static readonly PacketRateLimiter PacketLimiter =
            new PacketRateLimiter(200, TimeSpan.FromSeconds(1));

        /// <summary>
        /// Override the per-second send limit used to throttle outbound InSim packets.
        /// </summary>
        public static void SetPacketRateLimit(int maxPacketsPerSecond)
        {
            PacketLimiter.SetLimit(Math.Max(10, maxPacketsPerSecond));
        }

        public static void Initialize(this InSimClient client, InSimSettings settings)
        {
            client.InitializeAsync(settings).GetAwaiter().GetResult();
        }

        public static void Send(this InSimClient client, ISendable packet)
        {
            PacketLimiter.WaitForAvailability();
            client.SendAsync(packet).GetAwaiter().GetResult();
        }

        public static void Send(this InSimClient client, params ISendable[] packets)
        {
            PacketLimiter.WaitForAvailability();
            client.SendAsync(packets).GetAwaiter().GetResult();
        }

        public static void Send(this InSimClient client, string message, params object[] args)
        {
            PacketLimiter.WaitForAvailability();
            client.SendAsync(message, args).GetAwaiter().GetResult();
        }

        /// <summary>
        ///     Sends a message to the host connection without broadcasting it to all players.
        /// </summary>
        /// <param name="client">Connected InSim client.</param>
        /// <param name="message">Message text to deliver privately.</param>
        /// <param name="sound">Optional sound effect to play with the message.</param>
        public static void SendPrivateMessage(this InSimClient client, string message,
            MessageSound sound = MessageSound.SND_SILENT)
        {
            client.SendPrivateMessage(0, message, sound);
        }

        /// <summary>
        ///     Sends a message to a specific connection without exposing it to the full server chat.
        /// </summary>
        /// <param name="client">Connected InSim client.</param>
        /// <param name="ucid">Target connection ID (0 = host).</param>
        /// <param name="message">Message text to deliver privately.</param>
        /// <param name="sound">Optional sound effect to play with the message.</param>
        public static void SendPrivateMessage(this InSimClient client, byte ucid, string message,
            MessageSound sound = MessageSound.SND_SILENT)
        {
            if (client == null) throw new ArgumentNullException(nameof(client));

            var safeMessage = message ?? string.Empty;
            // IS_MTC is host-only and LFS throws runtime errors when sent as a client. Use /echo via MST instead so
            // messages stay local without spamming host-only errors.
            client.Send(new IS_MST { Msg = $"/echo {safeMessage}" });
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

        /// <summary>
        ///     Simple rolling-window rate limiter to avoid overwhelming LFS with bursts of packets.
        /// </summary>
        private sealed class PacketRateLimiter
        {
            private int maxPackets;
            private readonly TimeSpan window;
            private readonly Queue<DateTime> sendTimes = new Queue<DateTime>();
            private readonly object gate = new object();

            public PacketRateLimiter(int maxPackets, TimeSpan window)
            {
                this.maxPackets = maxPackets;
                this.window = window;
            }

            public void SetLimit(int maxPacketsPerWindow)
            {
                lock (gate)
                {
                    maxPackets = Math.Max(1, maxPacketsPerWindow);
                }
            }

            /// <summary>
            /// Block until a send slot is available inside the configured window.
            /// </summary>
            public void WaitForAvailability()
            {
                while (true)
                {
                    DateTime? waitUntil = null;
                    lock (gate)
                    {
                        var now = DateTime.UtcNow;
                        while (sendTimes.Count > 0 && now - sendTimes.Peek() >= window)
                        {
                            sendTimes.Dequeue();
                        }

                        if (sendTimes.Count < maxPackets)
                        {
                            sendTimes.Enqueue(now);
                            return;
                        }

                        waitUntil = sendTimes.Peek().Add(window);
                    }

                    var delay = waitUntil.GetValueOrDefault() - DateTime.UtcNow;
                    if (delay <= TimeSpan.Zero)
                        continue;

                    var sleep = delay > TimeSpan.FromMilliseconds(5) ? TimeSpan.FromMilliseconds(5) : delay;
                    Thread.Sleep(sleep);
                }
            }
        }
    }
}
