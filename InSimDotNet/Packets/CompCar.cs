using System;

namespace InSimDotNet.Packets
{
    /// <summary>
    ///     Class for the <see cref="IS_MCI" /> Info collection.
    /// </summary>
    public class CompCar
    {
        /// <summary>
        ///     Creates a new CompCar sub-packet.
        /// </summary>
        /// <param name="reader">A <see cref="PacketReader" /> containing the packet data.</param>
        public CompCar(PacketReader reader)
        {
            if (reader == null) throw new ArgumentNullException("reader");

            Node = reader.ReadUInt16();
            Lap = reader.ReadUInt16();
            PLID = reader.ReadByte();
            Position = reader.ReadByte();
            Info = (CompCarFlags)reader.ReadByte();
            reader.Skip(1);
            X = reader.ReadInt32();
            Y = reader.ReadInt32();
            Z = reader.ReadInt32();
            Speed = reader.ReadUInt16();
            Direction = reader.ReadUInt16();
            Heading = reader.ReadUInt16();
            AngVel = reader.ReadInt16();
        }

        /// <summary>
        ///     Creates a new CompCar sub-packet.
        /// </summary>
        /// <param name="other">An <see cref="CompCar" /> object to construct from.</param>
        public CompCar(CompCar other)
        {
            if (other == null) throw new ArgumentNullException(nameof(other));

            Node = other.Node;
            Lap = other.Lap;
            PLID = other.PLID;
            Position = other.Position;
            Info = other.Info;
            X = other.X;
            Y = other.Y;
            Z = other.Z;
            Speed = other.Speed;
            Direction = other.Direction;
            Heading = other.Heading;
            AngVel = other.AngVel;
        }

        /// <summary>
        ///     Gets the current path node
        /// </summary>
        public int Node { get; }

        /// <summary>
        ///     Gets the current lap
        /// </summary>
        public int Lap { get; }

        /// <summary>
        ///     Gets the unique ID of the player.
        /// </summary>
        public byte PLID { get; }

        /// <summary>
        ///     Gets the current race position : 0 = unknown, 1 = leader, etc...
        /// </summary>
        public byte Position { get; }

        /// <summary>
        ///     Gets the car flags and other info
        /// </summary>
        public CompCarFlags Info { get; }

        /// <summary>
        ///     Gets the cars current X coordinate (65536 = 1 metre)
        /// </summary>
        public int X { get; }

        /// <summary>
        ///     Gets the cars current Y coordinate (65536 = 1 metre)
        /// </summary>
        public int Y { get; }

        /// <summary>
        ///     Gets the cars current Z coordinate (65536 = 1 metre)
        /// </summary>
        public int Z { get; }

        /// <summary>
        ///     Gets the cars current speed (32768 = 100 m/s)
        /// </summary>
        public int Speed { get; }

        /// <summary>
        ///     Gets the direction of car's motion : 0 = world Y direction, 32768 = 180 deg
        /// </summary>
        public int Direction { get; }

        /// <summary>
        ///     Gets the cars current direction of forward axis : 0 = world Y direction, 32768 = 180 deg
        /// </summary>
        public int Heading { get; }

        /// <summary>
        ///     Gets the cars rate of change of heading : (16384 = 360 deg/s)
        /// </summary>
        public short AngVel { get; }
    }
}