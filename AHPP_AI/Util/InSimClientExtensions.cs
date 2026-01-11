using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using InSimDotNet;
using InSimDotNet.Packets;

namespace AHPP_AI.Util
{
    /// <summary>
    ///     Synchronous helpers for the asynchronous InSimClient API.
    /// </summary>
    public static class InSimClientExtensions
    {
        public enum PacketPriority
        {
            High,
            Normal
        }

        private static readonly OutboundSendQueue SendQueue =
            new OutboundSendQueue(200);

        /// <summary>
        /// Override the per-second send limit used to throttle outbound InSim packets.
        /// </summary>
        public static void SetPacketRateLimit(int maxPacketsPerSecond)
        {
            SendQueue.SetLimit(Math.Max(10, maxPacketsPerSecond));
        }

        public static void Initialize(this InSimClient client, InSimSettings settings)
        {
            SendQueue.BindClient(client);
            client.InitializeAsync(settings).GetAwaiter().GetResult();
            SendQueue.Start();
        }

        public static void Send(this InSimClient client, ISendable packet, PacketPriority priority = PacketPriority.Normal)
        {
            SendQueue.Enqueue(client, priority, packet);
        }

        public static void Send(this InSimClient client, params ISendable[] packets)
        {
            SendQueue.Enqueue(client, PacketPriority.Normal, packets);
        }

        public static void Send(this InSimClient client, PacketPriority priority, params ISendable[] packets)
        {
            SendQueue.Enqueue(client, priority, packets);
        }

        /// <summary>
        /// Send UI packets without consuming rate-limit tokens (used for small button updates).
        /// </summary>
        public static void SendUi(this InSimClient client, ISendable packet)
        {
            if (packet == null) return;
            SendQueue.Enqueue(client, PacketPriority.High, true, packet);
        }

        public static void Send(this InSimClient client, string message, params object[] args)
        {
            SendQueue.Enqueue(client, priority: PacketPriority.Normal, message, args);
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
                SendQueue.Shutdown();
                client.Disconnect();
            }
            catch (Exception)
            {
                // Ignore shutdown race conditions
            }
        }

        /// <summary>
        /// Get queue depth and measured send rate for diagnostics.
        /// </summary>
        public static OutboundQueueStats GetOutboundQueueStats()
        {
            return SendQueue.GetStats();
        }

        private sealed class OutboundSendQueue
        {
            private readonly ConcurrentQueue<OutboundSendItem> highPriority =
                new ConcurrentQueue<OutboundSendItem>();
            private readonly ConcurrentQueue<OutboundSendItem> normalPriority =
                new ConcurrentQueue<OutboundSendItem>();
            private readonly ManualResetEventSlim queueSignal = new ManualResetEventSlim(false);
            private readonly Queue<DateTime> sendTimes = new Queue<DateTime>();
            private readonly object rateLock = new object();
            private readonly object stateLock = new object();
            private CancellationTokenSource? cts;
            private Task? senderTask;
            private int maxPacketsPerSecond;
            private double tokenBalance;
            private DateTime lastRefill = DateTime.UtcNow;
            private bool stopRequested;
            private InSimClient? boundClient;

            public OutboundSendQueue(int maxPacketsPerSecond)
            {
                this.maxPacketsPerSecond = Math.Max(1, maxPacketsPerSecond);
                tokenBalance = this.maxPacketsPerSecond;
            }

            /// <summary>
            /// Bind the queue to the active InSim client so in-flight sends survive reconnects.
            /// </summary>
            public void BindClient(InSimClient client)
            {
                lock (stateLock)
                {
                    boundClient = client;
                }
            }

            /// <summary>
            /// Begin processing the outbound queue if not already running.
            /// </summary>
            public void Start()
            {
                lock (stateLock)
                {
                    if (senderTask != null && !senderTask.IsCompleted) return;

                    stopRequested = false;
                    tokenBalance = maxPacketsPerSecond;
                    lastRefill = DateTime.UtcNow;
                    cts = new CancellationTokenSource();
                    senderTask = Task.Run(() => RunAsync(cts.Token));
                }
            }

            /// <summary>
            /// Request a graceful shutdown and optionally flush remaining packets.
            /// </summary>
            public void Shutdown()
            {
                Task? taskToWait;
                lock (stateLock)
                {
                    stopRequested = true;
                    queueSignal.Set();
                    taskToWait = senderTask;
                }

                try
                {
                    taskToWait?.Wait(TimeSpan.FromSeconds(2));
                }
                catch (Exception)
                {
                    // Ignore teardown races
                }
            }

            /// <summary>
            /// Update the packet-per-second limit at runtime.
            /// </summary>
            public void SetLimit(int limit)
            {
                lock (rateLock)
                {
                    maxPacketsPerSecond = Math.Max(1, limit);
                    tokenBalance = Math.Min(tokenBalance, maxPacketsPerSecond);
                }
            }

            /// <summary>
            /// Enqueue one or more packets with the provided priority.
            /// </summary>
            public void Enqueue(InSimClient client, PacketPriority priority, params ISendable[] packets)
            {
                Enqueue(client, priority, false, packets);
            }

            public void Enqueue(InSimClient client, PacketPriority priority, bool bypassLimiter,
                params ISendable[] packets)
            {
                if (packets == null || packets.Length == 0) return;

                foreach (var packet in packets)
                {
                    if (packet == null) continue;
                    EnqueueInternal(new OutboundSendItem(client, priority, bypassLimiter, packet));
                }
            }

            /// <summary>
            /// Enqueue a text message for delivery.
            /// </summary>
            public void Enqueue(InSimClient client, PacketPriority priority, string message, params object[] args)
            {
                if (string.IsNullOrWhiteSpace(message)) return;
                EnqueueInternal(new OutboundSendItem(client, priority, false, message, args));
            }

            private void EnqueueInternal(OutboundSendItem item)
            {
                if (item.Priority == PacketPriority.High)
                    highPriority.Enqueue(item);
                else
                    normalPriority.Enqueue(item);

                queueSignal.Set();
            }

            public OutboundQueueStats GetStats()
            {
                var stats = new OutboundQueueStats
                {
                    HighPriorityDepth = highPriority.Count,
                    NormalPriorityDepth = normalPriority.Count,
                    SendRatePerSecond = GetSendRate()
                };

                return stats;
            }

            private async Task RunAsync(CancellationToken token)
            {
                while (true)
                {
                    if (stopRequested && highPriority.IsEmpty && normalPriority.IsEmpty)
                        break;

                    if (!TryDequeue(out var item))
                    {
                        WaitForEnqueue(token);
                        continue;
                    }

                    if (!TryConsumeToken(out var wait, item.BypassLimiter))
                    {
                        await Task.Delay(wait, token).ConfigureAwait(false);
                        RequeueFront(item);
                        continue;
                    }

                    try
                    {
                        await item.SendAsync(boundClient, token).ConfigureAwait(false);
                        RecordSend();
                    }
                    catch (OperationCanceledException)
                    {
                        if (token.IsCancellationRequested)
                            break;
                    }
                    catch (Exception)
                    {
                        // Ignore send errors; queue will continue for subsequent packets.
                    }
                }
            }

            private bool TryDequeue(out OutboundSendItem item)
            {
                if (highPriority.TryDequeue(out item))
                    return true;

                if (normalPriority.TryDequeue(out item))
                    return true;

                item = default;
                return false;
            }

            private void RequeueFront(OutboundSendItem item)
            {
                // Fallback to enqueueing again; ordering is preserved within priority queues.
                EnqueueInternal(item);
            }

            private void WaitForEnqueue(CancellationToken token)
            {
                queueSignal.Wait(TimeSpan.FromMilliseconds(50), token);
                queueSignal.Reset();
            }

            private bool TryConsumeToken(out TimeSpan waitTime, bool bypassLimiter)
            {
                if (bypassLimiter)
                {
                    waitTime = TimeSpan.Zero;
                    return true;
                }

                lock (rateLock)
                {
                    var now = DateTime.UtcNow;
                    var elapsedSeconds = Math.Max(0.0, (now - lastRefill).TotalSeconds);
                    if (elapsedSeconds > 0)
                    {
                        tokenBalance = Math.Min(
                            maxPacketsPerSecond,
                            tokenBalance + elapsedSeconds * maxPacketsPerSecond);
                        lastRefill = now;
                    }

                    if (tokenBalance >= 1.0)
                    {
                        tokenBalance -= 1.0;
                        waitTime = TimeSpan.Zero;
                        return true;
                    }

                    var deficit = 1.0 - tokenBalance;
                    var secondsUntilNext = deficit / Math.Max(1, maxPacketsPerSecond);
                    waitTime = TimeSpan.FromSeconds(Math.Max(0.001, secondsUntilNext));
                    return false;
                }
            }

            private void RecordSend()
            {
                lock (rateLock)
                {
                    var now = DateTime.UtcNow;
                    sendTimes.Enqueue(now);
                    var cutoff = now - TimeSpan.FromSeconds(1);
                    while (sendTimes.Count > 0 && sendTimes.Peek() < cutoff)
                        sendTimes.Dequeue();
                }
            }

            private double GetSendRate()
            {
                lock (rateLock)
                {
                    if (sendTimes.Count == 0) return 0.0;
                    var now = DateTime.UtcNow;
                    var cutoff = now - TimeSpan.FromSeconds(1);
                    while (sendTimes.Count > 0 && sendTimes.Peek() < cutoff)
                        sendTimes.Dequeue();
                    return sendTimes.Count;
                }
            }
        }

        private readonly struct OutboundSendItem
        {
            private readonly ISendable? packet;
            private readonly string? message;
            private readonly object[] messageArgs;
            private readonly InSimClient client;

            public OutboundSendItem(InSimClient client, PacketPriority priority, bool bypassLimiter, ISendable packet)
            {
                this.client = client;
                Priority = priority;
                this.packet = packet;
                message = null;
                messageArgs = Array.Empty<object>();
                BypassLimiter = bypassLimiter;
            }

            public OutboundSendItem(InSimClient client, PacketPriority priority, bool bypassLimiter, string message, params object[] args)
            {
                this.client = client;
                Priority = priority;
                packet = null;
                this.message = message;
                messageArgs = args ?? Array.Empty<object>();
                BypassLimiter = bypassLimiter;
            }

            public PacketPriority Priority { get; }

            public Task SendAsync(InSimClient? boundClient, CancellationToken token)
            {
                var target = boundClient ?? client;
                if (target == null) return Task.CompletedTask;

                if (packet != null)
                    return target.SendAsync(packet);

                return target.SendAsync(message ?? string.Empty, messageArgs);
            }

            public bool BypassLimiter { get; }
        }

        public struct OutboundQueueStats
        {
            public int HighPriorityDepth { get; set; }
            public int NormalPriorityDepth { get; set; }
            public double SendRatePerSecond { get; set; }
        }
    }
}
