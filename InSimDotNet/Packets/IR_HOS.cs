using System.Collections.Generic;
using System.Collections.ObjectModel;

namespace InSimDotNet.Packets
{
    /// <summary>
    ///     Host response packet.
    /// </summary>
    /// <remarks>
    ///     Sent in reply to <see cref="IR_HLR" /> host list request.
    /// </remarks>
    public class IR_HOS : IPacket
    {
        /// <summary>
        ///     Creates a new host response packet.
        /// </summary>
        /// <param name="buffer">A buffer contaning the packet data.</param>
        public IR_HOS(byte[] buffer)
        {
            var reader = new PacketReader(buffer);
            Size = reader.ReadSize();
            Type = (PacketType)reader.ReadByte();
            ReqI = reader.ReadByte();
            NumHosts = reader.ReadByte();

            var info = new List<HInfo>(NumHosts);
            for (var i = 0; i < NumHosts; i++) info.Add(new HInfo(reader));
            Info = new ReadOnlyCollection<HInfo>(info);
        }

        /// <summary>
        ///     Gets the number of hosts described in this packet.
        /// </summary>
        public byte NumHosts { get; }

        /// <summary>
        ///     Gets a collection of up to six <see cref="HInfo" /> packets, one for each host in the relay.
        ///     If more than six hosts are online then multiple <see cref="IR_HOS" /> packets are sent.
        /// </summary>
        public ReadOnlyCollection<HInfo> Info { get; private set; }

        /// <summary>
        ///     Gets the size of the packet.
        /// </summary>
        public int Size { get; }

        /// <summary>
        ///     Gets the type of the packet.
        /// </summary>
        public PacketType Type { get; }

        /// <summary>
        ///     Gets the packet request ID.
        /// </summary>
        public byte ReqI { get; }
    }
}