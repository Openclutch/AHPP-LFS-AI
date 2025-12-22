using System;

namespace InSimDotNet.Packets
{
    /// <summary>
    /// Switches for CAR LIGHTS <see cref="IS_SMALL"/> SMALL_LCL
    /// </summary>
    public static class LocalCarLights
    {
        /// <summary>
        /// Bit 0
        /// </summary>
        private const int LCL_SET_SIGNALS = 1;  // bit 0

        /// <summary>
        /// Bit 2 (bit 1 reserved)
        /// </summary>
        private const int LCL_SET_LIGHTS = 1 << 2;      // bit 2

        /// <summary>
        /// Bit 4
        /// </summary>
        private const int LCL_SET_FOG_REAR = 1 << 4; // bit 4

        /// <summary>
        /// Bit 5
        /// </summary>
        private const int LCL_SET_FOG_FRONT = 1 << 5;    // bit 5

        /// <summary>
        /// Bit 6
        /// </summary>
        private const int LCL_SET_EXTRA = 1 << 6;	// bit 6

        /// <summary>
        /// Turn signals off
        /// </summary>
        public const int LCL_SIGNALS_OFF = LCL_SET_SIGNALS;

        /// <summary>
        /// Turn signal left
        /// </summary>
        public const int LCL_SIGNALS_LEFT = (1 << 16) | LCL_SET_SIGNALS;

        /// <summary>
        /// Turn signal right
        /// </summary>
        public const int LCL_SIGNALS_RIGHT = (2 << 16) | LCL_SET_SIGNALS;

        /// <summary>
        /// Turn signal hazard
        /// </summary>
        public const int LCL_SIGNALS_HAZARD = (3 << 16) | LCL_SET_SIGNALS;

        /// <summary>
        /// Lights off
        /// </summary>
        public const int LCL_LIGHTS_OFF = LCL_SET_LIGHTS;

        /// <summary>
        /// Side lights on
        /// </summary>
        public const int LCL_SIDE_ON = (1 << 18) | LCL_SET_LIGHTS;
        /// <summary>
        /// Backwards compatible alias for <see cref="LCL_SIDE_ON"/>.
        /// </summary>
        [Obsolete("Use LCL_SIDE_ON instead.")]
        public const int LCL_LIGHTS_SIDE = LCL_SIDE_ON;

        /// <summary>
        /// Low lights on
        /// </summary>
        public const int LCL_LOW_ON = (2 << 18) | LCL_SET_LIGHTS;
        /// <summary>
        /// Backwards compatible alias for <see cref="LCL_LOW_ON"/>.
        /// </summary>
        [Obsolete("Use LCL_LOW_ON instead.")]
        public const int LCL_LIGHTS_LOW = LCL_LOW_ON;

        /// <summary>
        /// High lights on
        /// </summary>
        public const int LCL_HIGH_ON = (3 << 18) | LCL_SET_LIGHTS;
        /// <summary>
        /// Backwards compatible alias for <see cref="LCL_HIGH_ON"/>.
        /// </summary>
        [Obsolete("Use LCL_HIGH_ON instead.")]
        public const int LCL_LIGHTS_HIGH = LCL_HIGH_ON;

        /// <summary>
        /// Fog rear off
        /// </summary>
        public const int LCL_FOG_REAR_OFF = LCL_SET_FOG_REAR;

        /// <summary>
        /// Fog rear on
        /// </summary>
        public const int LCL_FOG_REAR_ON = (1 << 20) | LCL_SET_FOG_REAR;

        /// <summary>
        /// Fog front off
        /// </summary>
        public const int LCL_FOG_FRONT_OFF = LCL_SET_FOG_FRONT;

        /// <summary>
        /// Fog front on
        /// </summary>
        public const int LCL_FOG_FRONT_ON = (1 << 21) | LCL_SET_FOG_FRONT;

        /// <summary>
        /// Extra Light off
        /// </summary>
        public const int LCL_EXTRA_OFF = LCL_SET_EXTRA;

        /// <summary>
        /// Extra Light on
        /// </summary>
        public const int LCL_EXTRA_ON = (1 << 22) | LCL_SET_EXTRA;
    }
}
