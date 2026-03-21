using System.Collections.Generic;
using InSimDotNet.Packets;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Shared helpers for building AI control packets and formatting control diagnostics.
    /// </summary>
    public static class AIControlDiagnostics
    {
        /// <summary>
        ///     Build the control inputs that should be sent for a single AI update tick.
        /// </summary>
        public static List<AIInputVal> BuildInputPacket(
            int steering,
            int throttle,
            int brake,
            byte gear,
            int clutchValue,
            bool automaticTransmission,
            bool resetInputsBeforeApply)
        {
            var inputs = new List<AIInputVal>();

            if (resetInputsBeforeApply)
            {
                inputs.Add(new AIInputVal
                {
                    Input = AicInputType.CS_RESET_INPUTS,
                    Value = 0
                });
            }

            inputs.Add(new AIInputVal { Input = AicInputType.CS_MSX, Value = steering });
            inputs.Add(new AIInputVal { Input = AicInputType.CS_THROTTLE, Value = throttle });
            inputs.Add(new AIInputVal { Input = AicInputType.CS_BRAKE, Value = brake });

            if (!automaticTransmission)
            {
                inputs.Add(new AIInputVal { Input = AicInputType.CS_GEAR, Value = gear });
                inputs.Add(new AIInputVal { Input = AicInputType.CS_CLUTCH, Value = clutchValue });
            }

            return inputs;
        }

        /// <summary>
        ///     Format the short control string shown in the debug HUD.
        /// </summary>
        public static string FormatCompactControlInfo(
            int steeringCenter,
            int steering,
            int throttle,
            int brake,
            byte gear,
            int clutchValue,
            bool automaticTransmission,
            bool resetInputsBeforeApply)
        {
            var steerOffset = steering - steeringCenter;
            var gearLabel = automaticTransmission ? "A" : gear.ToString();
            var clutchLabel = automaticTransmission ? "A" : $"{clutchValue / 1000}k";
            var resetLabel = resetInputsBeforeApply ? " R:1" : string.Empty;

            return
                $"T:{throttle / 1000}k B:{brake / 1000}k G:{gearLabel} C:{clutchLabel} S:{steerOffset}{resetLabel}";
        }

        /// <summary>
        ///     Build a detailed per-tick trace line for debugging AI driving issues.
        /// </summary>
        public static string BuildTraceLine(
            byte plid,
            int steeringCenter,
            int steering,
            int throttle,
            int brake,
            byte gear,
            int clutchValue,
            bool automaticTransmission,
            bool resetInputsBeforeApply,
            double speedKmh,
            string stateLabel,
            string status,
            bool suspiciousOverlap,
            string overlapReason)
        {
            var gearLabel = automaticTransmission ? "A" : gear.ToString();
            var clutchLabel = automaticTransmission ? "A" : clutchValue.ToString();
            var resetLabel = resetInputsBeforeApply ? "1" : "0";
            var suspiciousLabel = suspiciousOverlap ? $" Suspicious={overlapReason}" : string.Empty;

            return
                $"PLID={plid} CONTROL TRACE: State={stateLabel}, Status={status}, Speed={speedKmh:F1}km/h, " +
                $"Steer={steering} ({steering - steeringCenter:+#;-#;0}), Throttle={throttle}, Brake={brake}, " +
                $"Gear={gearLabel}, Clutch={clutchLabel}, Reset={resetLabel}{suspiciousLabel}";
        }

        /// <summary>
        ///     Flag control combinations that are likely to indicate conflicting throttle/brake or throttle/clutch input.
        /// </summary>
        public static bool HasSuspiciousOverlap(
            int throttle,
            int brake,
            int clutchValue,
            bool automaticTransmission,
            byte gear,
            double speedKmh,
            int brakeBase,
            int clutchFullyPressed,
            bool lowRpmClutchActive,
            out string reason)
        {
            reason = string.Empty;

            if (throttle <= 0)
                return false;

            if (brake >= System.Math.Max(2000, brakeBase))
            {
                reason = "Throttle+brake overlap";
                return true;
            }

            if (automaticTransmission || lowRpmClutchActive)
                return false;

            var clutchEngagedThreshold = clutchFullyPressed / 2;
            var takingOffInFirst = gear == 2 && speedKmh < 5.0;
            if (clutchValue > clutchEngagedThreshold && !takingOffInFirst)
            {
                reason = "Throttle+clutch overlap";
                return true;
            }

            return false;
        }
    }
}
