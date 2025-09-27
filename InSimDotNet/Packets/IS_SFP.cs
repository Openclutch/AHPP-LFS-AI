namespace InSimDotNet.Packets
{
    /// <summary>
    ///     State flags packet.
    /// </summary>
    /// <remarks>
    ///     Used to set LFS state.
    /// </remarks>
    public class IS_SFP : IPacket, ISendable
    {
        /// <summary>
        ///     Creates a new state flags packet.
        /// </summary>
        public IS_SFP()
        {
            Size = 8;
            Type = PacketType.ISP_SFP;
        }

        /// <summary>
        ///     Gets or sets the state flags.
        /// </summary>
        public StateFlags Flag { get; set; }

        /// <summary>
        ///     Gets or sets if the flag is on or off.
        /// </summary>
        public bool OffOn { get; set; }

        /// <summary>
        ///     Gets the size of the packet.
        /// </summary>
        public int Size { get; }

        /// <summary>
        ///     Gets the type of the packet.
        /// </summary>
        public PacketType Type { get; }

        /// <summary>
        ///     Gets or sets the packet request ID.
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
            writer.Skip(1);
            writer.Write((ushort)Flag);
            writer.Write(OffOn);
            return writer.GetBuffer();
        }
    }
}