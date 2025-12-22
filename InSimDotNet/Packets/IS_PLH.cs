using System;
using System.Collections.Generic;

namespace InSimDotNet.Packets
{
    /// <summary>
    /// Player handicap information packet.
    /// </summary>
    public class IS_PLH : IPacket, ISendable
    {
        /// <summary>
        /// Maximum number of player handicap entries allowed in a single packet.
        /// </summary>
        public const int PLH_MAX_PLAYERS = 48;
        /// <summary>
        /// Gets the packet size.
        /// </summary>
        public int Size { get; private set; }

        /// <summary>
        /// Gets the packet type.
        /// </summary>
        public PacketType Type { get; private set; }

        /// <summary>
        /// Gets the request ID.
        /// </summary>
        /// <remarks>
        /// 0 unless this is a reply to a TINY_PLH request.
        /// </remarks>
        public byte ReqI { get; set; }

        /// <summary>
        /// Gets the number of players in this packet. This value is filled in automatically when sending handicaps.
        /// </summary>
        public byte NumP { get; private set; }

        /// <summary>
        /// Gets a collection of <see cref="PlayerHCap"/> sub-packets.
        /// </summary>
        public IList<PlayerHCap> HCaps { get; private set; }

        /// <summary>
        /// Create a new <see cref="IS_PLH"/> object.
        /// </summary>
        public IS_PLH()
        {
            Size = 4;
            Type = PacketType.ISP_PLH;
            HCaps = new List<PlayerHCap>(PLH_MAX_PLAYERS);
        }

        /// <summary>
        /// Creates a new <see cref="IS_PLH"/> object.
        /// </summary>
        /// <param name="hcap">A collection of <see cref="PlayerHCap"/> sub-packets.</param>
        public IS_PLH(IEnumerable<PlayerHCap> hcap)
            : this()
        {
            HCaps = new List<PlayerHCap>(hcap);
        }

        /// <summary>
        /// Creates a new <see cref="IS_PLH"/> object.
        /// </summary>
        /// <param name="buffer">The packet data</param>
        public IS_PLH(byte[] buffer)
        {
            PacketReader reader = new PacketReader(buffer);
            Size = reader.ReadSize();
            Type = (PacketType)reader.ReadByte();
            ReqI = reader.ReadByte();
            NumP = reader.ReadByte();
            HCaps = new List<PlayerHCap>(NumP);

            for (int i = 0; i < NumP; i++)
            {
                HCaps.Add(new PlayerHCap(reader));
            }
        }

        /// <summary>
        /// Gets the packet data.
        /// </summary>
        /// <returns>An array containing the packet data.</returns>
        public byte[] GetBuffer()
        {
            if (HCaps.Count > PLH_MAX_PLAYERS)
                throw new InvalidOperationException("IS_PLH too many player handicaps set");

            NumP = (byte)HCaps.Count;
            Size = 4 + (NumP * 4);
            PacketWriter writer = new PacketWriter(Size);
            writer.WriteSize(Size);
            writer.Write((byte)Type);
            writer.Write(ReqI);
            writer.Write(NumP);
            
            foreach (PlayerHCap hcap in HCaps)
            {
                hcap.GetBuffer(writer);
            }

            return writer.GetBuffer();
        }
    }
}
