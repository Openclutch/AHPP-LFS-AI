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
        public const byte MainEnd = 109;
        public const byte AiLabelStart = 110;
        public const byte AiLabelEnd = 144;
        public const byte AiModeStart = 145;
        public const byte AiModeEnd = 179;
        public const byte AiRemoveStart = 180;
        public const byte AiRemoveEnd = 214;
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
        /// Calculate an ID for AI label buttons in the right-hand list.
        /// </summary>
        public static byte AiLabel(byte offset)
        {
            return ValidateRange(AiLabelStart, AiLabelEnd, offset);
        }

        /// <summary>
        /// Calculate an ID for AI mode buttons in the right-hand list.
        /// </summary>
        public static byte AiMode(byte offset)
        {
            return ValidateRange(AiModeStart, AiModeEnd, offset);
        }

        /// <summary>
        /// Calculate an ID within the AI remove-button range to keep deletions grouped.
        /// </summary>
        public static byte AiRemove(byte offset)
        {
            return ValidateRange(AiRemoveStart, AiRemoveEnd, offset);
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
