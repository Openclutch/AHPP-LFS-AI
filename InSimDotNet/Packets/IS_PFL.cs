namespace InSimDotNet.Packets
{
    /// <summary>
    ///     Player flags packet.
    /// </summary>
    /// <remarks>
    ///     Sent when player flags have changed.
    /// </remarks>
    public class IS_PFL : IPacket
    {
        /// <summary>
        ///     Creates a new player flags packet.
        /// </summary>
        public IS_PFL()
        {
            Size = 8;
            Type = PacketType.ISP_PFL;
        }

        /// <summary>
        ///     Creates a new player flags packet.
        /// </summary>
        /// <param name="buffer">A buffer contaning the packet data.</param>
        public IS_PFL(byte[] buffer)
            : this()
        {
            var reader = new PacketReader(buffer);
            Size = reader.ReadSize();
            Type = (PacketType)reader.ReadByte();
            ReqI = reader.ReadByte();
            PLID = reader.ReadByte();
            Flags = (PlayerFlags)reader.ReadUInt16();
        }

        /// <summary>
        ///     Gets the unique ID of the player.
        /// </summary>
        public byte PLID { get; private set; }

        /// <summary>
        ///     Gets the new player flags.
        /// </summary>
        public PlayerFlags Flags { get; private set; }

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