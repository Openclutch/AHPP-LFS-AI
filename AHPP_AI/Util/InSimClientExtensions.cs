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
            client.Send(new IS_MTC
            {
                UCID = ucid,
                Msg = safeMessage,
                Sound = sound
            });
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
