namespace InSimDotNet.Packets
{
    /// <summary>
    ///     General purpose packet four-byte packet.
    /// </summary>
    public class IS_TINY : IPacket, ISendable
    {
        /// <summary>
        ///     Creates a new general purpose packet.
        /// </summary>
        public IS_TINY()
        {
            Size = 4;
            Type = PacketType.ISP_TINY;
        }

        /// <summary>
        ///     Creates a new general purpose packet.
        /// </summary>
        /// <param name="buffer">A buffer contaning the packet data.</param>
        public IS_TINY(byte[] buffer)
            : this()
        {
            var reader = new PacketReader(buffer);
            Size = reader.ReadSize();
            Type = (PacketType)reader.ReadByte();
            ReqI = reader.ReadByte();
            SubT = (TinyType)reader.ReadByte();
        }

        /// <summary>
        ///     Gets or sets the sub-type.
        /// </summary>
        public TinyType SubT { get; set; }

        /// <summary>
        ///     Gets the packet size.
        /// </summary>
        public int Size { get; }

        /// <summary>
        ///     Gets the packet type.
        /// </summary>
        public PacketType Type { get; }

        /// <summary>
        ///     Gets or sets the request ID.
        /// </summary>
        public byte ReqI { get; set; }

        /// <summary>
        ///     Returns the packet data.
        /// </summary>
        /// <returns>The packet data.</returns>
        public byte[] GetBuffer()
        {
            var writer = new PacketWriter(Size);
            writer.WriteSize(Size);
            writer.Write((byte)Type);
            writer.Write(ReqI);
            writer.Write((byte)SubT);
            return writer.GetBuffer();
        }
    }
}