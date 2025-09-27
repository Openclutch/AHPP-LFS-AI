using System;

namespace InSimDotNet.Out
{
    /// <summary>
    ///     Provides data for the OutGauge PacketReceived event.
    /// </summary>
    public class OutGaugeEventArgs : EventArgs
    {
        /// <summary>
        ///     Creates a new OutGaugeEventArgs object.
        /// </summary>
        /// <param name="packet">The OutGauge packet.</param>
        public OutGaugeEventArgs(OutGaugePack packet)
        {
            Packet = packet;
        }

        /// <summary>
        ///     Gets the OutGauge packet.
        /// </summary>
        public OutGaugePack Packet { get; }

        /// <summary>
        ///     Gets the time (to check order).
        /// </summary>
        public TimeSpan Time => Packet.Time;

        /// <summary>
        ///     Gets the car name.
        /// </summary>
        public string Car => Packet.Car;

        /// <summary>
        ///     Gets the OutGauge info flags.
        /// </summary>
        public OutGaugeFlags Flags => Packet.Flags;

        /// <summary>
        ///     Gets the current gear (reverse: 0, neutral: 1, first: 2 etc..).
        /// </summary>
        public byte Gear => Packet.Gear;

        /// <summary>
        ///     Gets the PLID of the player.
        /// </summary>
        public byte PLID => Packet.PLID;

        /// <summary>
        ///     Gets the speed in meters per second.
        /// </summary>
        public float Speed => Packet.Speed;

        /// <summary>
        ///     Gets the RPM.
        /// </summary>
        public float RPM => Packet.RPM;

        /// <summary>
        ///     Gets the turbo BAR.
        /// </summary>
        public float Turbo => Packet.Turbo;

        /// <summary>
        ///     Gets the engine temperature in degrees centigrade.
        /// </summary>
        public float EngTemp => Packet.EngTemp;

        /// <summary>
        ///     Gets the fuel (0.0 to 1.0).
        /// </summary>
        public float Fuel => Packet.Fuel;

        /// <summary>
        ///     Gets the oil pressure in BAR.
        /// </summary>
        public float OilPressure => Packet.OilPressure;

        /// <summary>
        ///     Gets the oil temperature in degrees centigrade.
        /// </summary>
        public float OilTemp => Packet.OilTemp;

        /// <summary>
        ///     Gets which dashboard lights available for this car.
        /// </summary>
        public DashLightFlags DashLights => Packet.DashLights;

        /// <summary>
        ///     Gets the dashboard lights currently switched on.
        /// </summary>
        public DashLightFlags ShowLights => Packet.ShowLights;

        /// <summary>
        ///     Gets the throttle position (0.0 to 1.0).
        /// </summary>
        public float Throttle => Packet.Throttle;

        /// <summary>
        ///     Gets the brake position (0.0 to 1.0).
        /// </summary>
        public float Brake => Packet.Brake;

        /// <summary>
        ///     Gets the clutch position (0.0 to 1.0).
        /// </summary>
        public float Clutch => Packet.Clutch;

        /// <summary>
        ///     Gets the first LCD display (usually fuel).
        /// </summary>
        public string Display1 => Packet.Display1;

        /// <summary>
        ///     Gets the second LCD display (usually settings).
        /// </summary>
        public string Display2 => Packet.Display2;

        /// <summary>
        ///     Gets the optional OutGauge ID (if specified in cfg.txt).
        /// </summary>
        public int ID => Packet.ID;
    }
}