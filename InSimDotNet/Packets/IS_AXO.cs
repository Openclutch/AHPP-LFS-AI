namespace InSimDotNet.Packets
{
    /// <summary>
    ///     AutoX object packet
    /// </summary>
    /// <remarks>
    ///     Sent if an autocross object is hit.
    /// </remarks>
    public class IS_AXO : IPacket
    {
        /// <summary>
        ///     Creates a new AutoX object packet.
        /// </summary>
        public IS_AXO()
        {
            Size = 4;
            Type = PacketType.ISP_AXO;
        }

        /// <summary>
        ///     Creates a new AutoX object packet.
        /// </summary>
        /// <param name="buffer">A buffer contaning the packet data.</param>
        public IS_AXO(byte[] buffer)
            : this()
        {
            var reader = new PacketReader(buffer);
            Size = reader.ReadSize();
            Type = (PacketType)reader.ReadByte();
            ReqI = reader.ReadByte();
            PLID = reader.ReadByte();
        }

        /// <summary>
        ///     Gets the unique ID of the player who hit the object.
        /// </summary>
        public byte PLID { get; private set; }

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