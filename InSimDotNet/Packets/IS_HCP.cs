namespace InSimDotNet.Packets
{
    /// <summary>
    ///     HandiCaPs
    /// </summary>
    public class IS_HCP : IPacket, ISendable
    {
        private const int NumCars = 32;

        /// <summary>
        ///     Creates a new IS_HCP packet.
        /// </summary>
        public IS_HCP()
        {
            Size = 68;
            Type = PacketType.ISP_HCP;
            Info = new CarHCP[NumCars];
        }

        /// <summary>
        ///     Gets the handicap info for each car in order (XF GTI first, XR GT second etc..).
        /// </summary>
        public CarHCP[] Info { get; }

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
        ///     Gets the packet buffer.
        /// </summary>
        /// <returns>An array containing the packet data.</returns>
        public byte[] GetBuffer()
        {
            var writer = new PacketWriter(Size);
            writer.WriteSize(Size);
            writer.Write((byte)Type);
            writer.Write(ReqI);
            writer.Skip(1);

            foreach (var info in Info) info.GetBuffer(writer);

            return writer.GetBuffer();
        }
    }
}