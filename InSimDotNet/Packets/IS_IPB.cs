using System;
using System.Collections.Generic;
using System.Net;

namespace InSimDotNet.Packets
{
    /// <summary>
    /// IP ban list information packet.
    /// </summary>
    public class IS_IPB : IPacket, ISendable
    {
        /// <summary>
        /// Maximum number of IP bans allowed in a single packet.
        /// </summary>
        public const int IPB_MAX_BANS = 120;

        /// <summary>
        /// Gets the size of the packet.
        /// </summary>
        public int Size { get; private set; }

        /// <summary>
        /// Gets the type of the packet.
        /// </summary>
        public PacketType Type { get; private set; }

        /// <summary>
        /// Gets the requestID.
        /// </summary>
        /// <remarks>
        /// 0 unless this is a reply to a TINY_IPB request.
        /// </remarks>
        public byte ReqI { get; set; }

        /// <summary>
        /// The number of bans in this packet. This value is filled in automatically when sending bans.
        /// </summary>
        public byte NumB { get; private set; }

        /// <summary>
        /// Reserved byte.
        /// </summary>
        public byte Sp0 { get; private set; }

        /// <summary>
        /// Reserved byte.
        /// </summary>
        public byte Sp1 { get; private set; }

        /// <summary>
        /// Reserved byte.
        /// </summary>
        public byte Sp2 { get; private set; }

        /// <summary>
        /// Reserved byte.
        /// </summary>
        public byte Sp3 { get; private set; }

        /// <summary>
        /// Gets a collection of <see cref="IPAddress"/> entries that are banned.
        /// </summary>
        public IList<IPAddress> BanIPs { get; set; }

        /// <summary>
        /// Creates a new <see cref="IS_IPB"/> object.
        /// </summary>
        public IS_IPB()
        {
            Size = 8;
            Type = PacketType.ISP_IPB;
            BanIPs = new List<IPAddress>(IPB_MAX_BANS);
        }

        /// <summary>
        /// Creates a new <see cref="IS_IPB"/> object.
        /// </summary>
        /// <param name="banIPs">A collection of <see cref="IPAddress"/></param>
        public IS_IPB(IEnumerable<IPAddress> banIPs)
            : this()
        {
            BanIPs = new List<IPAddress>(banIPs);
        }

        /// <summary>
        /// Creates a new <see cref="IS_IPB"/> object.
        /// </summary>
        /// <param name="buffer">The packet data</param>
        public IS_IPB(byte[] buffer)
        {
            PacketReader reader = new PacketReader(buffer);
            Size = reader.ReadSize();
            Type = (PacketType)reader.ReadByte();
            ReqI = reader.ReadByte();
            NumB = reader.ReadByte();
            Sp0 = reader.ReadByte();
            Sp1 = reader.ReadByte();
            Sp2 = reader.ReadByte();
            Sp3 = reader.ReadByte();

            BanIPs = new List<IPAddress>(NumB);

            for (int i = 0; i < NumB; i++)
            {
                BanIPs.Add(new IPAddress(reader.ReadUInt32()));
            }
        }

        /// <summary>
        /// Gets the packet data.
        /// </summary>
        /// <returns>An array containing the packet data.</returns>
        public byte[] GetBuffer()
        {
            if (BanIPs.Count > IPB_MAX_BANS)
                throw new InvalidOperationException("IS_IPB too many bans");

            NumB = (byte)BanIPs.Count;
            Size = 8 + (NumB * 4);
            PacketWriter writer = new PacketWriter(Size);
            writer.WriteSize(Size);
            writer.Write((byte)Type);
            writer.Write(ReqI);
            writer.Write(NumB);
            writer.Skip(4);

            foreach (IPAddress ip in BanIPs)
            {
                byte[] bytes = ip.GetAddressBytes();
                writer.Write(BitConverter.ToUInt32(bytes, 0));
            }
            return writer.GetBuffer();
        }
    }
}
