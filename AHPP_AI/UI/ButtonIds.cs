using System;

namespace AHPP_AI.UI
{
    /// <summary>
    /// Central registry for InSim button ID ranges so UI elements avoid
    /// collisions with LFS built-in prompts and other AHPP overlays.
    /// </summary>
    public static class ButtonIds
    {
        public const byte ReservedCeiling = 29; // Leave low IDs for LFS prompts
        public const byte MainStart = 30;
        public const byte MainEnd = 119;
        public const byte DynamicStart = 120;
        public const byte DynamicEnd = 179;
        public const byte RemoveStart = 180;
        public const byte RemoveEnd = 219;
        public const byte DebugStart = 220;
        public const byte DebugEnd = 239;

        /// <summary>
        /// Calculate an ID within the primary UI range.
        /// </summary>
        public static byte MainId(byte offset)
        {
            return ValidateRange(MainStart, MainEnd, offset);
        }

        /// <summary>
        /// Calculate an ID for dynamic lists (AI labels, visualization entries).
        /// </summary>
        public static byte Dynamic(byte offset)
        {
            return ValidateRange(DynamicStart, DynamicEnd, offset);
        }

        /// <summary>
        /// Calculate an ID within the remove-button range to keep deletions grouped.
        /// </summary>
        public static byte Remove(byte offset)
        {
            return ValidateRange(RemoveStart, RemoveEnd, offset);
        }

        /// <summary>
        /// Calculate an ID for debug overlays and tooling.
        /// </summary>
        public static byte Debug(byte offset)
        {
            return ValidateRange(DebugStart, DebugEnd, offset);
        }

        private static byte ValidateRange(byte start, byte end, byte offset)
        {
            var value = start + offset;
            if (value > end)
                throw new ArgumentOutOfRangeException(nameof(offset),
                    $"Button ID {value} exceeds reserved range {start}-{end}");
            return (byte)value;
        }
    }
}
