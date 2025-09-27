using System;

namespace InSimDotNet.Out
{
    /// <summary>
    ///     Provides data for the OutSim PacketReceived event.
    /// </summary>
    public class OutSimEventArgs : EventArgs
    {
        /// <summary>
        ///     Creates a new OutSimEventArgs object.
        /// </summary>
        /// <param name="packet">The OutSim packet.</param>
        public OutSimEventArgs(OutSimPack packet)
        {
            Packet = packet;
        }

        /// <summary>
        ///     Gets the OutSim packet.
        /// </summary>
        public OutSimPack Packet { get; }

        /// <summary>
        ///     Gets the time in milliseconds (to check order).
        /// </summary>
        public TimeSpan Time => Packet.Time;

        /// <summary>
        ///     Gets the angular velocity.
        /// </summary>
        public Vector AngVel => Packet.AngVel;

        /// <summary>
        ///     Gets the current heading (anticlockwise from above (Z)).
        /// </summary>
        public float Heading => Packet.Heading;

        /// <summary>
        ///     Gets the current pitch (anticlockwise from right (X)).
        /// </summary>
        public float Pitch => Packet.Pitch;

        /// <summary>
        ///     Gets the current roll (anticlockwise from front (Y)).
        /// </summary>
        public float Roll => Packet.Roll;

        /// <summary>
        ///     Gets the current acceleration.
        /// </summary>
        public Vector Accel => Packet.Accel;

        /// <summary>
        ///     Gets the current velocity.
        /// </summary>
        public Vector Vel => Packet.Vel;

        /// <summary>
        ///     Gets the current position (1m = 65536).
        /// </summary>
        public Vec Pos => Packet.Pos;

        /// <summary>
        ///     Gets the optional OutSim ID (if specified in cfg.txt).
        /// </summary>
        public int ID => Packet.ID;
    }
}