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

        private readonly AIConfig config;
        private readonly Dictionary<byte, int> currentClutchValues = new Dictionary<byte, int>();
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
            currentGears[plid] = 1; // Neutral
            clutchStates[plid] = ClutchState.Released;
            currentClutchValues[plid] = config.ClutchReleased;
            lowRpmClutchActive[plid] = false;
            lowRpmClutchTimers[plid] = DateTime.MinValue;
            clutchTimers[plid] = DateTime.MinValue;
            lastShiftTimes[plid] = DateTime.MinValue;
        }

        /// <summary>
        ///     Update gearbox state including gear selection and clutch operation
        /// </summary>
        public void UpdateGearbox(byte plid, double speedKmh, float engineRpm = 0)
        {
            if (HandleLowRpmClutch(plid, engineRpm)) return;

            var desiredGear = config.CalculateDesiredGear(speedKmh);
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
        ///     Update gear with appropriate clutch control
        /// </summary>
        private void UpdateGearWithClutch(byte plid, byte desiredGear, double speedKmh)
        {
            // Check if we need to change gear
            if (desiredGear != currentGears[plid])
            {
                // Handle clutch state machine for gear changes
                switch (clutchStates[plid])
                {
                    case ClutchState.Released:
                        // Start gear change process by fully pressing clutch
                        if (DateTime.Now.Subtract(lastShiftTimes[plid]).TotalMilliseconds >= config.ShiftDelayMs)
                        {
                            currentClutchValues[plid] = config.ClutchFullyPressed;
                            clutchStates[plid] = ClutchState.Pressing;
                            clutchTimers[plid] = DateTime.Now;
                            logger.Log(
                                $"PLID={plid} CLUTCH: Pressing for gear change from {currentGears[plid]} to {desiredGear}");
                        }

                        break;

                    case ClutchState.Pressing:
                        // Wait for clutch press delay, then change gear
                        if (DateTime.Now.Subtract(clutchTimers[plid]).TotalMilliseconds >= config.ClutchPressDelayMs)
                        {
                            currentGears[plid] = desiredGear;
                            clutchStates[plid] = ClutchState.Pressed;
                            clutchTimers[plid] = DateTime.Now;
                            logger.Log($"PLID={plid} GEAR: Changed to {desiredGear}");
                        }

                        break;

                    case ClutchState.Pressed:
                        // Start releasing clutch gradually
                        clutchStates[plid] = ClutchState.Releasing;
                        clutchTimers[plid] = DateTime.Now;
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
                // If clutch is still engaged but no gear change is needed, continue release process
                if (clutchStates[plid] == ClutchState.Releasing)
                {
                    var elapsedMs = DateTime.Now.Subtract(clutchTimers[plid]).TotalMilliseconds;
                    var releaseStep = (int)(elapsedMs / config.ClutchReleaseIntervalMs);

                    if (releaseStep >= config.ClutchReleaseSteps)
                    {
                        // Fully released
                        currentClutchValues[plid] = config.ClutchReleased;
                        clutchStates[plid] = ClutchState.Released;
                        lastShiftTimes[plid] = DateTime.Now;
                    }
                    else
                    {
                        // Calculate intermediate clutch value
                        var releaseProgress = (double)releaseStep / config.ClutchReleaseSteps;
                        currentClutchValues[plid] = (int)(config.ClutchFullyPressed * (1 - releaseProgress));
                    }
                }
                else
                {
                    // Skip to release phase
                    clutchStates[plid] = ClutchState.Releasing;
                    clutchTimers[plid] = DateTime.Now;
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
            // Apply clutch when braking hard
            if (brakeValue > config.BrakeBase * 2)
                // Press clutch when hard braking to prevent stalling
                currentClutchValues[plid] = config.ClutchFullyPressed;
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
    }
}
