using System;
using System.Collections.Generic;
using AHPP_AI.Debug;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Represents the current state of the clutch operation
    /// </summary>
    public enum ClutchState
    {
        Released,
        Pressing,
        Pressed,
        Releasing
    }

    /// <summary>
    ///     Handles transmission control including gear shifting and clutch operation
    /// </summary>
    public class GearboxController
    {
        private readonly Dictionary<byte, ClutchState> clutchStates = new Dictionary<byte, ClutchState>();
        private readonly Dictionary<byte, DateTime> clutchTimers = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, bool> brakingClutchActive = new Dictionary<byte, bool>();

        private readonly AIConfig config;
        private readonly Dictionary<byte, int> currentClutchValues = new Dictionary<byte, int>();
        private readonly Dictionary<byte, bool> gearChangeInProgress = new Dictionary<byte, bool>();
        private readonly Dictionary<byte, byte> gearShiftTargets = new Dictionary<byte, byte>();
        private readonly Dictionary<byte, bool> lowRpmClutchActive = new Dictionary<byte, bool>();
        private readonly Dictionary<byte, DateTime> lowRpmClutchTimers = new Dictionary<byte, DateTime>();

        // State tracking per car
        private readonly Dictionary<byte, byte> currentGears = new Dictionary<byte, byte>();
        private readonly Dictionary<byte, DateTime> lastShiftTimes = new Dictionary<byte, DateTime>();
        private readonly Logger logger;

        public GearboxController(AIConfig config, Logger logger)
        {
            this.config = config;
            this.logger = logger;
        }

        /// <summary>
        ///     Initialize a new car's gearbox
        /// </summary>
        public void InitializeGearbox(byte plid)
        {
            // Start aligned with the control inputs we send on spawn (clutch in, first gear selected).
            currentGears[plid] = 2; // LFS gear index 2 = 1st
            clutchStates[plid] = ClutchState.Pressed;
            currentClutchValues[plid] = config.ClutchFullyPressed;
            gearChangeInProgress[plid] = false;
            gearShiftTargets[plid] = currentGears[plid];
            lowRpmClutchActive[plid] = false;
            lowRpmClutchTimers[plid] = DateTime.MinValue;
            clutchTimers[plid] = DateTime.Now;
            lastShiftTimes[plid] = DateTime.MinValue;
            brakingClutchActive[plid] = false;
        }

        /// <summary>
        ///     Update gearbox state including gear selection and clutch operation
        /// </summary>
        public void UpdateGearbox(byte plid, double speedKmh, float engineRpm = 0)
        {
            var desiredGear = ApplyGearHysteresis(plid, config.CalculateDesiredGear(speedKmh), speedKmh);

            // If RPM is falling too low, request a downshift instead of just holding the clutch.
            if (engineRpm > 0 && engineRpm < config.StallPreventionReleaseRpm && currentGears[plid] > 2)
                desiredGear = (byte)Math.Max(2, currentGears[plid] - 1);

            if (HandleLowRpmClutch(plid, engineRpm)) return;

            UpdateGearWithClutch(plid, desiredGear, speedKmh);
        }

        /// <summary>
        ///     Check if the clutch is currently being held in to prevent a stall at low RPM.
        /// </summary>
        public bool IsLowRpmClutchActive(byte plid)
        {
            return lowRpmClutchActive.ContainsKey(plid) && lowRpmClutchActive[plid];
        }

        /// <summary>
        ///     Get current gearbox information for output and debugging
        /// </summary>
        public (byte gear, int clutchValue, ClutchState clutchState) GetGearboxInfo(byte plid)
        {
            return (
                currentGears.ContainsKey(plid) ? currentGears[plid] : (byte)1,
                currentClutchValues.ContainsKey(plid) ? currentClutchValues[plid] : config.ClutchReleased,
                clutchStates.ContainsKey(plid) ? clutchStates[plid] : ClutchState.Released
            );
        }

        /// <summary>
        /// Force-reset gearbox state to 1st gear with clutch held for recovery restart sequences.
        /// </summary>
        public void ResetToFirstGear(byte plid, string reason = "recovery gear reset")
        {
            currentGears[plid] = 2;
            gearShiftTargets[plid] = currentGears[plid];
            gearChangeInProgress[plid] = false;
            clutchStates[plid] = ClutchState.Pressed;
            currentClutchValues[plid] = config.ClutchFullyPressed;
            clutchTimers[plid] = DateTime.Now;
            lastShiftTimes[plid] = DateTime.Now;
            brakingClutchActive[plid] = false;
            lowRpmClutchActive[plid] = false;
            lowRpmClutchTimers[plid] = DateTime.MinValue;
            logger.Log($"PLID={plid} GEAR RESET: set to 1st ({reason})");
        }

        /// <summary>
        ///     Update gear with appropriate clutch control
        /// </summary>
        private void UpdateGearWithClutch(byte plid, byte desiredGear, double speedKmh)
        {
            if (!gearShiftTargets.ContainsKey(plid)) gearShiftTargets[plid] = currentGears[plid];
            if (!gearChangeInProgress.ContainsKey(plid)) gearChangeInProgress[plid] = false;

            // If we're mid-shift, keep pursuing the original target instead of re-evaluating every tick.
            var targetGear = gearChangeInProgress[plid] ? gearShiftTargets[plid] : desiredGear;
            var needGearChange = targetGear != currentGears[plid];

            // Check if we need to change gear
            if (needGearChange)
            {
                // Respect minimum time between shifts to prevent hunting
                if (DateTime.Now.Subtract(lastShiftTimes[plid]).TotalMilliseconds < config.GearShiftMinIntervalMs &&
                    clutchStates[plid] == ClutchState.Released)
                    return;

                // Handle clutch state machine for gear changes
                switch (clutchStates[plid])
                {
                    case ClutchState.Released:
                        // Start gear change process by fully pressing clutch
                        if (DateTime.Now.Subtract(lastShiftTimes[plid]).TotalMilliseconds >= config.ShiftDelayMs)
                        {
                            gearChangeInProgress[plid] = true;
                            gearShiftTargets[plid] = targetGear;
                            currentClutchValues[plid] = config.ClutchFullyPressed;
                            clutchStates[plid] = ClutchState.Pressing;
                            clutchTimers[plid] = DateTime.Now;
                            logger.Log(
                                $"PLID={plid} CLUTCH: Pressing for gear change from {currentGears[plid]} to {targetGear}");
                        }

                        break;

                    case ClutchState.Pressing:
                        // Wait for clutch press delay, then change gear
                        if (DateTime.Now.Subtract(clutchTimers[plid]).TotalMilliseconds >= config.ClutchPressDelayMs)
                        {
                            // Lock onto the current target for this shift cycle
                            gearShiftTargets[plid] = targetGear;
                            currentGears[plid] = gearShiftTargets[plid];
                            clutchStates[plid] = ClutchState.Pressed;
                            clutchTimers[plid] = DateTime.Now;
                            logger.Log($"PLID={plid} GEAR: Changed to {gearShiftTargets[plid]}");
                        }

                        break;

                    case ClutchState.Pressed:
                        // Ensure the requested gear is latched while the clutch stays fully pressed
                        if (currentGears[plid] != gearShiftTargets[plid])
                        {
                            currentGears[plid] = gearShiftTargets[plid];
                            logger.Log($"PLID={plid} GEAR: Changed to {gearShiftTargets[plid]} (latched)");
                        }

                        // Stay fully pressed until the shift is confirmed, then start releasing
                        var shiftConfirmed = !gearChangeInProgress[plid] ||
                                             currentGears[plid] == gearShiftTargets[plid];
                        var holdElapsed = DateTime.Now.Subtract(clutchTimers[plid]).TotalMilliseconds;
                        var holdSatisfied = clutchTimers[plid] == DateTime.MinValue ||
                                            holdElapsed >= config.ClutchHoldAfterShiftMs;

                        if (shiftConfirmed && holdSatisfied)
                        {
                            gearChangeInProgress[plid] = false;
                            clutchStates[plid] = ClutchState.Releasing;
                            clutchTimers[plid] = DateTime.Now;
                        }
                        else
                        {
                            currentClutchValues[plid] = config.ClutchFullyPressed;
                        }

                        break;

                    case ClutchState.Releasing:
                        // Gradually release clutch over time
                        var elapsedMs = DateTime.Now.Subtract(clutchTimers[plid]).TotalMilliseconds;
                        var releaseStep = (int)(elapsedMs / config.ClutchReleaseIntervalMs);

                        if (releaseStep >= config.ClutchReleaseSteps)
                        {
                            // Fully released
                            currentClutchValues[plid] = config.ClutchReleased;
                            clutchStates[plid] = ClutchState.Released;
                            lastShiftTimes[plid] = DateTime.Now;
                            gearShiftTargets[plid] = currentGears[plid];
                            gearChangeInProgress[plid] = false;
                            logger.Log($"PLID={plid} CLUTCH: Fully released after gear change to {currentGears[plid]}");
                        }
                        else
                        {
                            // Calculate intermediate clutch value (gradually releasing)
                            var releaseProgress = (double)releaseStep / config.ClutchReleaseSteps;
                            currentClutchValues[plid] = (int)(config.ClutchFullyPressed * (1 - releaseProgress));
                        }

                        break;
                }
            }
            else if (clutchStates[plid] != ClutchState.Released)
            {
                // If clutch is still engaged but no gear change is needed, continue hold/release process
                switch (clutchStates[plid])
                {
                    case ClutchState.Releasing:
                        var elapsedMs = DateTime.Now.Subtract(clutchTimers[plid]).TotalMilliseconds;
                        var releaseStep = (int)(elapsedMs / config.ClutchReleaseIntervalMs);

                        if (releaseStep >= config.ClutchReleaseSteps)
                        {
                            // Fully released
                            currentClutchValues[plid] = config.ClutchReleased;
                            clutchStates[plid] = ClutchState.Released;
                            lastShiftTimes[plid] = DateTime.Now;
                            gearShiftTargets[plid] = currentGears[plid];
                            gearChangeInProgress[plid] = false;
                        }
                        else
                        {
                            // Calculate intermediate clutch value
                            var releaseProgress = (double)releaseStep / config.ClutchReleaseSteps;
                            currentClutchValues[plid] = (int)(config.ClutchFullyPressed * (1 - releaseProgress));
                        }

                        break;

                    case ClutchState.Pressed:
                        var holdElapsedMs = DateTime.Now.Subtract(clutchTimers[plid]).TotalMilliseconds;
                        var shiftConfirmed = !gearChangeInProgress[plid] ||
                                             currentGears[plid] == gearShiftTargets[plid];
                        var holdSatisfied = clutchTimers[plid] == DateTime.MinValue ||
                                            holdElapsedMs >= config.ClutchHoldAfterShiftMs;

                        if (shiftConfirmed && holdSatisfied)
                        {
                            gearChangeInProgress[plid] = false;
                            clutchStates[plid] = ClutchState.Releasing;
                            clutchTimers[plid] = DateTime.Now;
                        }
                        else
                        {
                            currentClutchValues[plid] = config.ClutchFullyPressed;
                        }

                        break;

                    default:
                        // Skip to release phase
                        clutchStates[plid] = ClutchState.Releasing;
                        clutchTimers[plid] = DateTime.Now;
                        gearChangeInProgress[plid] = false;
                        break;
                }
            }

            // Special case: If speed is very low and we're starting from a standstill in first gear
            if (speedKmh < 2 && currentGears[plid] == 2 && clutchStates[plid] == ClutchState.Released)
                // Slightly engage clutch to prevent stalling
                currentClutchValues[plid] = config.ClutchFullyPressed / 3;
        }

        /// <summary>
        ///     Apply appropriate clutch value during braking
        /// </summary>
        public void ApplyBrakingClutch(byte plid, int brakeValue)
        {
            if (!brakingClutchActive.ContainsKey(plid)) brakingClutchActive[plid] = false;

            var hardBraking = brakeValue > config.BrakeBase * 2;

            // Apply clutch when braking hard
            if (hardBraking)
            {
                if (!brakingClutchActive[plid])
                {
                    brakingClutchActive[plid] = true;

                    // Press clutch when hard braking to prevent stalling
                    if (clutchStates[plid] != ClutchState.Pressed)
                    {
                        clutchStates[plid] = ClutchState.Pressing;
                        clutchTimers[plid] = DateTime.Now;
                    }
                }

                currentClutchValues[plid] = config.ClutchFullyPressed;
            }
            else if (brakingClutchActive[plid] &&
                     (clutchStates[plid] == ClutchState.Pressed || clutchStates[plid] == ClutchState.Pressing))
            {
                brakingClutchActive[plid] = false;

                // Begin releasing once braking intensity drops
                clutchStates[plid] = ClutchState.Releasing;
                clutchTimers[plid] = DateTime.Now;
                currentClutchValues[plid] = config.ClutchFullyPressed;
            }
        }

        /// <summary>
        ///     Check if throttle should be applied based on clutch state
        /// </summary>
        public bool ShouldApplyThrottle(byte plid, double speedKmh)
        {
            if (IsLowRpmClutchActive(plid)) return true;

            var isLowSpeedFirstGear = currentGears[plid] == 2 && speedKmh < 5;
            var isClutchEngaged = currentClutchValues[plid] > config.ClutchFullyPressed / 2;

            // Only allow throttle when clutch is in if we're in 1st gear at low speed (taking off)
            return !isClutchEngaged || isLowSpeedFirstGear;
        }

        /// <summary>
        ///     Engage and hold the clutch if RPM dips too low to prevent stalling, releasing once recovered.
        /// </summary>
        private bool HandleLowRpmClutch(byte plid, float engineRpm)
        {
            if (!lowRpmClutchActive.ContainsKey(plid))
            {
                lowRpmClutchActive[plid] = false;
                lowRpmClutchTimers[plid] = DateTime.MinValue;
            }

            var protectionActive = lowRpmClutchActive[plid];

            if (engineRpm > 0 && engineRpm < config.StallPreventionRpm)
            {
                lowRpmClutchActive[plid] = true;
                lowRpmClutchTimers[plid] = DateTime.Now;
                currentClutchValues[plid] = config.ClutchFullyPressed;
                clutchStates[plid] = ClutchState.Pressed;
                return true;
            }

            if (protectionActive)
            {
                var holdElapsed = DateTime.Now - lowRpmClutchTimers[plid];
                var recoveredRpm = engineRpm >= config.StallPreventionReleaseRpm;
                var heldLongEnough = holdElapsed.TotalMilliseconds >= config.StallPreventionHoldMs;

                if (!recoveredRpm && !heldLongEnough)
                {
                    currentClutchValues[plid] = config.ClutchFullyPressed;
                    clutchStates[plid] = ClutchState.Pressed;
                    return true;
                }

                lowRpmClutchActive[plid] = false;
                clutchStates[plid] = ClutchState.Releasing;
                clutchTimers[plid] = DateTime.Now;
            }

            return false;
        }

        /// <summary>
        ///     Apply basic hysteresis so small speed oscillations do not trigger rapid gear hunting.
        /// </summary>
        private byte ApplyGearHysteresis(byte plid, byte desiredGear, double speedKmh)
        {
            if (!currentGears.ContainsKey(plid)) return desiredGear;

            var currentGear = currentGears[plid];
            var upHys = Math.Max(0, config.GearUpshiftHysteresisKmh);
            var downHys = Math.Max(0, config.GearDownshiftHysteresisKmh);

            // Prevent array bounds issues
            var thresholds = config.GearSpeedThresholds ?? Array.Empty<double>();

            // Upshift check (current gear -> next)
            var upIndex = currentGear - 2;
            if (upIndex >= 0 && upIndex < thresholds.Length)
            {
                var upThreshold = thresholds[upIndex];
                if (desiredGear > currentGear && speedKmh < upThreshold + upHys)
                    desiredGear = currentGear;
            }

            // Downshift check (current gear -> previous)
            var downIndex = currentGear - 3;
            if (downIndex >= 0 && downIndex < thresholds.Length)
            {
                var downThreshold = thresholds[downIndex];
                if (desiredGear < currentGear && speedKmh > downThreshold - downHys)
                    desiredGear = currentGear;
            }

            return desiredGear;
        }
    }
}
