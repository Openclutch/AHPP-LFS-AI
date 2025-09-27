namespace InSimDotNet.Packets
{
    /// <summary>
    ///     Single character packet.
    /// </summary>
    /// <remarks>
    ///     Used to send a single character to LFS.
    /// </remarks>
    public class IS_SCH : IPacket, ISendable
    {
        /// <summary>
        ///     Creates a new single character packet.
        /// </summary>
        public IS_SCH()
        {
            Size = 8;
            Type = PacketType.ISP_SCH;
        }

        /// <summary>
        ///     Gets or sets the key to press.
        /// </summary>
        public char CharB { get; set; }

        /// <summary>
        ///     Gets or sets the character modifier flags.
        /// </summary>
        public CharacterModifiers Flags { get; set; }

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
        ///     Gets the packet data.
        /// </summary>
        /// <returns>Returns the packet data.</returns>
        public byte[] GetBuffer()
        {
            var writer = new PacketWriter(Size);
            writer.WriteSize(Size);
            writer.Write((byte)Type);
            writer.Write(ReqI);
            writer.Skip(1);
            writer.Write(CharB);
            writer.Write((byte)Flags);
            return writer.GetBuffer();
        }
    }
}