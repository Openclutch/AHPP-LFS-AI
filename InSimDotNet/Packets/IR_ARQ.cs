namespace InSimDotNet.Packets
{
    /// <summary>
    ///     Admin request packet.
    /// </summary>
    /// <remarks>
    ///     When sent LFS will respond with a <see cref="IR_ARP" /> admin response packet.
    /// </remarks>
    public class IR_ARQ : IPacket, ISendable
    {
        /// <summary>
        ///     Creates a new admin request packet.
        /// </summary>
        public IR_ARQ()
        {
            Size = 4;
            Type = PacketType.IRP_ARQ;
        }

        /// <summary>
        ///     Gets the size of the packet.
        /// </summary>
        public int Size { get; }

        /// <summary>
        ///     Gets the type of the packet.
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
            return writer.GetBuffer();
        }
    }
}