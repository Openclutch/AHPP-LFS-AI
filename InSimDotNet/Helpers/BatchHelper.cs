using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using InSimDotNet.Packets;

namespace InSimDotNet.Helpers
{
    /// <summary>
    ///     Experimental - don't use.
    /// </summary>
    internal class BatchHelper
    {
        private readonly InSim insim;
        private readonly List<ISendable> packets = new List<ISendable>();
        private bool isSending;

        public BatchHelper(InSim insim)
        {
            if (insim == null) throw new ArgumentNullException("insim");

            this.insim = insim;

            BatchSize = 2048; // bytes
            BatchDelay = 100; // milliseconds
        }

        public int BatchSize { get; set; }
        public int BatchDelay { get; }

        public bool IsConnected => insim.IsConnected;

        public void Send(ISendable packet)
        {
            if (!insim.IsConnected) throw new InSimException(StringResources.InSimNotConnectedMessage);

            packets.Add(packet);

            BatchSendAsync();
        }

        public void Send(params ISendable[] packets)
        {
            if (!insim.IsConnected) throw new InSimException(StringResources.InSimNotConnectedMessage);

            this.packets.AddRange(packets);

            BatchSendAsync();
        }

        private async void BatchSendAsync()
        {
            if (isSending)
            {
                Debug.WriteLine("already sending");
                return;
            }

            isSending = true;

            while (true)
            {
                var packets = GetPacketBatch();

                insim.Send(packets.ToArray());

                // wait and see if any more packets added.
                await Task.Delay(BatchDelay);

                Debug.WriteLine("just waited " + BatchDelay + " ms");

                // if not then bye bye
                if (this.packets.Count == 0) break;
            }

            isSending = false;
        }

        private IList<ISendable> GetPacketBatch()
        {
            var batch = new List<ISendable>();

            // get batch of packets up to max byte size.
            var size = 0;
            for (var i = 0; i < packets.Count; i++)
                if (size + packets[i].Size < BatchSize)
                {
                    batch.Add(packets[i]);
                    size += packets[i].Size;
                    break;
                }

            // remove the packets for that last batch.
            packets.RemoveRange(0, batch.Count);

            return batch;
        }
    }
}