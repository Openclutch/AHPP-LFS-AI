using System;
using System.Collections.Generic;
using AHPP_AI.Debug;
using InSimDotNet.Packets;

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
    ///     Represents the simplified transmission phases used while commanding and verifying a shift.
    /// </summary>
    internal enum GearShiftPhase
    {
        Idle,
        SpawnLaunch,        // Initial takeoff from stopped with engine already running
        PressClutch,
        WaitForGearConfirmation,
        HoldAfterConfirmation,
        ReleaseClutch
    }

    /// <summary>
    ///     Detailed gearbox state used for diagnostics and launch debugging.
    /// </summary>
    internal readonly struct GearboxDebugInfo
    {
        public GearboxDebugInfo(
            byte commandedGear,
            byte targetGear,
            byte actualGear,
            bool actualGearFresh,
            int clutchValue,
            ClutchState clutchState,
            GearShiftPhase shiftPhase)
        {
            CommandedGear = commandedGear;
            TargetGear = targetGear;
            ActualGear = actualGear;
            ActualGearFresh = actualGearFresh;
            ClutchValue = clutchValue;
            ClutchState = clutchState;
            ShiftPhase = shiftPhase;
        }

        public byte CommandedGear { get; }
        public byte TargetGear { get; }
        public byte ActualGear { get; }
        public bool ActualGearFresh { get; }
        public int ClutchValue { get; }
        public ClutchState ClutchState { get; }
        public GearShiftPhase ShiftPhase { get; }
    }

    /// <summary>
    ///     Handles transmission control by verifying each requested shift against live LFS AI info.
    /// </summary>
    public class GearboxController
    {
        private static readonly TimeSpan ActualGearFreshnessWindow = TimeSpan.FromSeconds(3);

        private readonly Dictionary<byte, byte> actualGears = new Dictionary<byte, byte>();
        private readonly Dictionary<byte, DateTime> actualGearTimestamps = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, bool> brakingClutchActive = new Dictionary<byte, bool>();
        private readonly Dictionary<byte, ClutchState> clutchStates = new Dictionary<byte, ClutchState>();
        private readonly Dictionary<byte, DateTime> clutchTimers = new Dictionary<byte, DateTime>();
        private readonly AIConfig config;
        private readonly Dictionary<byte, int> currentClutchValues = new Dictionary<byte, int>();
        private readonly Dictionary<byte, byte> currentGears = new Dictionary<byte, byte>();
        private readonly Dictionary<byte, byte> gearShiftTargets = new Dictionary<byte, byte>();
        private readonly Dictionary<byte, DateTime> lastShiftTimes = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, bool> lowRpmClutchActive = new Dictionary<byte, bool>();
        private readonly Dictionary<byte, DateTime> lowRpmClutchTimers = new Dictionary<byte, DateTime>();
        private readonly Logger logger;
        private readonly Dictionary<byte, GearShiftPhase> shiftPhases = new Dictionary<byte, GearShiftPhase>();
        private readonly Dictionary<byte, DateTime> shiftPhaseStartedAt = new Dictionary<byte, DateTime>();

        public GearboxController(AIConfig config, Logger logger)
        {
            this.config = config;
            this.logger = logger;
        }

        /// <summary>
        ///     Initialize a new car's gearbox and enter the spawn launch phase to smoothly release the clutch.
        /// </summary>
        public void InitializeGearbox(byte plid)
        {
            currentGears[plid] = 2; // LFS gear index 2 = 1st
            gearShiftTargets[plid] = 2;
            clutchStates[plid] = ClutchState.Pressed;
            currentClutchValues[plid] = config.ClutchFullyPressed;
            lowRpmClutchActive[plid] = false;
            lowRpmClutchTimers[plid] = DateTime.MinValue;
            brakingClutchActive[plid] = false;
            shiftPhases[plid] = GearShiftPhase.SpawnLaunch;
            shiftPhaseStartedAt[plid] = DateTime.UtcNow;
            clutchTimers[plid] = DateTime.UtcNow;
            lastShiftTimes[plid] = DateTime.MinValue;
            logger.Log($"PLID={plid} GEARBOX INIT: entering SpawnLaunch (hold={config.LaunchHoldMs}ms release={config.LaunchClutchReleaseMs}ms throttle={config.LaunchThrottleValue})");
        }

        /// <summary>
        ///     Store live AI info so later control ticks can verify that LFS actually selected the requested gear.
        /// </summary>
        public void UpdateActualGear(byte plid, byte actualGear)
        {
            actualGears[plid] = actualGear;
            actualGearTimestamps[plid] = DateTime.UtcNow;
        }

        /// <summary>
        ///     Update gearbox state including gear selection and clutch operation.
        /// </summary>
        public void UpdateGearbox(
            byte plid,
            double speedKmh,
            float engineRpm = 0,
            AIConfig.DrivingMode drivingMode = AIConfig.DrivingMode.Cruise)
        {
            EnsureStateInitialized(plid);

            if (drivingMode == AIConfig.DrivingMode.Race && config.RaceUseAutomaticTransmission)
                return;

            var desiredGear = ApplyGearHysteresis(plid, config.CalculateDesiredGear(speedKmh), speedKmh, drivingMode);

            // If RPM is falling too low, request a downshift instead of just holding the clutch.
            if (engineRpm > 0 && engineRpm < config.StallPreventionReleaseRpm && GetReferenceGear(plid) > 2)
                desiredGear = (byte)Math.Max(2, GetReferenceGear(plid) - 1);

            // Don't let low-RPM clutch protection interrupt the spawn launch sequence —
            // SpawnLaunch already manages the clutch release; the protection would fight it
            // by pressing the clutch back in and then overwriting the phase to ReleaseClutch.
            if (shiftPhases[plid] != GearShiftPhase.SpawnLaunch && HandleLowRpmClutch(plid, engineRpm))
                return;

            AdvanceShiftState(plid, desiredGear);
        }

        /// <summary>
        ///     Check if the clutch is currently being held in to prevent a stall at low RPM.
        /// </summary>
        public bool IsLowRpmClutchActive(byte plid)
        {
            return lowRpmClutchActive.ContainsKey(plid) && lowRpmClutchActive[plid];
        }

        /// <summary>
        ///     Check if the gearbox is still in the initial spawn launch phase.
        /// </summary>
        public bool IsSpawnLaunchActive(byte plid)
        {
            return shiftPhases.TryGetValue(plid, out var phase) && phase == GearShiftPhase.SpawnLaunch;
        }

        /// <summary>
        ///     Get the current commanded gearbox values used when sending the next control frame.
        /// </summary>
        public (byte gear, int clutchValue, ClutchState clutchState) GetGearboxInfo(byte plid)
        {
            EnsureStateInitialized(plid);
            return (currentGears[plid], currentClutchValues[plid], clutchStates[plid]);
        }

        /// <summary>
        ///     Get the current gearbox debug state including the most recent confirmed LFS gear.
        /// </summary>
        internal GearboxDebugInfo GetGearboxDebugInfo(byte plid)
        {
            EnsureStateInitialized(plid);
            var actualGearFresh = TryGetFreshActualGear(plid, out var actualGear);
            var shiftPhase = shiftPhases.TryGetValue(plid, out var phase) ? phase : GearShiftPhase.Idle;

            return new GearboxDebugInfo(
                currentGears[plid],
                gearShiftTargets[plid],
                actualGearFresh ? actualGear : (byte)255,
                actualGearFresh,
                currentClutchValues[plid],
                clutchStates[plid],
                shiftPhase);
        }

        /// <summary>
        ///     Provide a short verification-focused status string for debug logging and HUD output.
        /// </summary>
        public string GetShiftStatus(byte plid)
        {
            EnsureStateInitialized(plid);

            var phase = shiftPhases[plid];
            if (phase == GearShiftPhase.Idle)
                return string.Empty;

            if (phase == GearShiftPhase.SpawnLaunch)
            {
                var elapsed = (DateTime.UtcNow - shiftPhaseStartedAt[plid]).TotalMilliseconds;
                var clutch = currentClutchValues[plid];
                return $"SPAWN LAUNCH: clutch={clutch} elapsed={elapsed:F0}ms";
            }

            var actualGearFresh = TryGetFreshActualGear(plid, out var actualGear);
            var actualLabel = actualGearFresh ? FormatGear(actualGear) : "?";
            return $"SHIFT {phase}: target={FormatGear(gearShiftTargets[plid])} actual={actualLabel}";
        }

        /// <summary>
        ///     Force-reset gearbox state to 1st gear with clutch held for recovery restart sequences.
        /// </summary>
        public void ResetToFirstGear(byte plid, string reason = "recovery gear reset")
        {
            EnsureStateInitialized(plid);

            currentGears[plid] = 2;
            gearShiftTargets[plid] = 2;
            clutchStates[plid] = ClutchState.Pressed;
            currentClutchValues[plid] = config.ClutchFullyPressed;
            clutchTimers[plid] = DateTime.UtcNow;
            shiftPhases[plid] = GearShiftPhase.WaitForGearConfirmation;
            shiftPhaseStartedAt[plid] = DateTime.UtcNow;
            lastShiftTimes[plid] = DateTime.UtcNow;
            brakingClutchActive[plid] = false;
            lowRpmClutchActive[plid] = false;
            lowRpmClutchTimers[plid] = DateTime.MinValue;
            logger.Log($"PLID={plid} GEAR RESET: waiting for 1st ({reason})");
        }

        /// <summary>
        ///     Apply appropriate clutch value during braking.
        /// </summary>
        public void ApplyBrakingClutch(byte plid, int brakeValue)
        {
            EnsureStateInitialized(plid);

            var hardBraking = brakeValue > config.BrakeBase * 2;

            if (hardBraking)
            {
                brakingClutchActive[plid] = true;
                currentClutchValues[plid] = config.ClutchFullyPressed;
                clutchStates[plid] = ClutchState.Pressed;
                return;
            }

            if (!brakingClutchActive[plid])
                return;

            brakingClutchActive[plid] = false;

            if (shiftPhases[plid] == GearShiftPhase.Idle)
            {
                clutchStates[plid] = ClutchState.Releasing;
                clutchTimers[plid] = DateTime.UtcNow;
                shiftPhases[plid] = GearShiftPhase.ReleaseClutch;
                shiftPhaseStartedAt[plid] = DateTime.UtcNow;
            }
        }

        /// <summary>
        ///     Check if throttle should be applied based on the verified gear and clutch state.
        /// </summary>
        public bool ShouldApplyThrottle(byte plid, double speedKmh)
        {
            EnsureStateInitialized(plid);

            if (IsLowRpmClutchActive(plid))
                return true;

            var shiftPhase = shiftPhases.TryGetValue(plid, out var phase) ? phase : GearShiftPhase.Idle;

            // Always allow throttle during spawn launch so the launch throttle minimum can be applied.
            if (shiftPhase == GearShiftPhase.SpawnLaunch)
                return true;

            var isClutchEngaged = currentClutchValues[plid] > config.ClutchFullyPressed / 2;
            var actualGear = TryGetFreshActualGear(plid, out var freshActualGear) ? freshActualGear : currentGears[plid];
            var takingOffInVerifiedFirst = actualGear == 2 &&
                                           speedKmh < 5.0 &&
                                           (shiftPhase == GearShiftPhase.ReleaseClutch || shiftPhase == GearShiftPhase.Idle);

            if (shiftPhase == GearShiftPhase.WaitForGearConfirmation && actualGear != gearShiftTargets[plid])
                return false;

            return !isClutchEngaged || takingOffInVerifiedFirst;
        }

        /// <summary>
        ///     Engage and hold the clutch if RPM dips too low to prevent stalling, releasing once recovered.
        /// </summary>
        private bool HandleLowRpmClutch(byte plid, float engineRpm)
        {
            var protectionActive = lowRpmClutchActive[plid];

            if (engineRpm <= 0 && !protectionActive)
                return false;

            if (engineRpm > 0 && engineRpm < config.StallPreventionRpm)
            {
                lowRpmClutchActive[plid] = true;
                lowRpmClutchTimers[plid] = DateTime.UtcNow;
                currentClutchValues[plid] = config.ClutchFullyPressed;
                clutchStates[plid] = ClutchState.Pressed;
                return true;
            }

            if (!protectionActive)
                return false;

            var holdElapsed = DateTime.UtcNow - lowRpmClutchTimers[plid];
            var recoveredRpm = engineRpm > 0 && engineRpm >= config.StallPreventionReleaseRpm;
            var heldLongEnough = holdElapsed.TotalMilliseconds >= config.StallPreventionHoldMs;

            if (!recoveredRpm && !heldLongEnough)
            {
                currentClutchValues[plid] = config.ClutchFullyPressed;
                clutchStates[plid] = ClutchState.Pressed;
                return true;
            }

            lowRpmClutchActive[plid] = false;
            clutchStates[plid] = ClutchState.Releasing;
            clutchTimers[plid] = DateTime.UtcNow;
            shiftPhases[plid] = GearShiftPhase.ReleaseClutch;
            shiftPhaseStartedAt[plid] = DateTime.UtcNow;
            return false;
        }

        /// <summary>
        ///     Advance the simplified shift/verification state machine using the latest desired gear and AII feedback.
        /// </summary>
        private void AdvanceShiftState(byte plid, byte desiredGear)
        {
            var referenceGear = GetReferenceGear(plid);
            if (shiftPhases[plid] == GearShiftPhase.Idle &&
                desiredGear != referenceGear &&
                (DateTime.UtcNow - lastShiftTimes[plid]).TotalMilliseconds >= config.GearShiftMinIntervalMs)
                BeginShift(plid, desiredGear);

            var phase = shiftPhases[plid];
            var now = DateTime.UtcNow;
            var elapsedMs = (now - shiftPhaseStartedAt[plid]).TotalMilliseconds;

            switch (phase)
            {
                case GearShiftPhase.SpawnLaunch:
                    // Hold clutch fully in for LaunchHoldMs, then slowly release over LaunchClutchReleaseMs.
                    currentGears[plid] = 2; // 1st gear
                    gearShiftTargets[plid] = 2;
                    if (elapsedMs < config.LaunchHoldMs)
                    {
                        currentClutchValues[plid] = config.ClutchFullyPressed;
                        clutchStates[plid] = ClutchState.Pressed;
                    }
                    else
                    {
                        var releaseElapsed = elapsedMs - config.LaunchHoldMs;
                        var releaseProgress = Math.Min(1.0, releaseElapsed / Math.Max(1, config.LaunchClutchReleaseMs));
                        // Smooth ease-out curve: faster initial release to build momentum, gentle finish
                        var curve = Math.Sqrt(releaseProgress);
                        currentClutchValues[plid] = (int)(config.ClutchFullyPressed * (1.0 - curve));
                        clutchStates[plid] = currentClutchValues[plid] > 0 ? ClutchState.Releasing : ClutchState.Released;

                        if (curve >= 1.0)
                        {
                            currentClutchValues[plid] = config.ClutchReleased;
                            clutchStates[plid] = ClutchState.Released;
                            shiftPhases[plid] = GearShiftPhase.Idle;
                            lastShiftTimes[plid] = DateTime.UtcNow;
                            logger.Log($"PLID={plid} SPAWN LAUNCH: clutch release complete, handing off to normal driving");
                        }
                    }
                    break;

                case GearShiftPhase.Idle:
                    currentGears[plid] = referenceGear;
                    gearShiftTargets[plid] = referenceGear;
                    currentClutchValues[plid] = config.ClutchReleased;
                    clutchStates[plid] = ClutchState.Released;
                    break;

                case GearShiftPhase.PressClutch:
                    currentClutchValues[plid] = config.ClutchFullyPressed;
                    clutchStates[plid] = ClutchState.Pressing;
                    currentGears[plid] = gearShiftTargets[plid];

                    if (elapsedMs >= config.ClutchPressDelayMs)
                    {
                        shiftPhases[plid] = GearShiftPhase.WaitForGearConfirmation;
                        shiftPhaseStartedAt[plid] = now;
                        clutchStates[plid] = ClutchState.Pressed;
                    }

                    break;

                case GearShiftPhase.WaitForGearConfirmation:
                    currentClutchValues[plid] = config.ClutchFullyPressed;
                    clutchStates[plid] = ClutchState.Pressed;
                    currentGears[plid] = gearShiftTargets[plid];

                    var gearConfirmed = TryGetFreshActualGear(plid, out var actualGear) && actualGear == gearShiftTargets[plid];
                    var confirmTimedOut = config.GearConfirmationTimeoutMs > 0 && elapsedMs >= config.GearConfirmationTimeoutMs;

                    if (gearConfirmed)
                    {
                        shiftPhases[plid] = GearShiftPhase.HoldAfterConfirmation;
                        shiftPhaseStartedAt[plid] = now;
                        logger.Log($"PLID={plid} GEAR VERIFIED: {FormatGear(actualGear)} acknowledged by LFS");
                    }
                    else if (confirmTimedOut)
                    {
                        shiftPhases[plid] = GearShiftPhase.HoldAfterConfirmation;
                        shiftPhaseStartedAt[plid] = now;
                        logger.Log($"PLID={plid} GEAR CONFIRMATION TIMEOUT after {elapsedMs:F0}ms: proceeding with {FormatGear(gearShiftTargets[plid])} unconfirmed");
                    }

                    break;

                case GearShiftPhase.HoldAfterConfirmation:
                    currentClutchValues[plid] = config.ClutchFullyPressed;
                    clutchStates[plid] = ClutchState.Pressed;
                    currentGears[plid] = gearShiftTargets[plid];

                    if (elapsedMs >= config.ClutchHoldAfterShiftMs)
                    {
                        shiftPhases[plid] = GearShiftPhase.ReleaseClutch;
                        shiftPhaseStartedAt[plid] = now;
                        clutchTimers[plid] = now;
                        clutchStates[plid] = ClutchState.Releasing;
                    }

                    break;

                case GearShiftPhase.ReleaseClutch:
                    currentGears[plid] = gearShiftTargets[plid];
                    clutchStates[plid] = ClutchState.Releasing;
                    ApplyClutchRelease(plid, elapsedMs);
                    break;
            }
        }

        /// <summary>
        ///     Start a new verified shift toward the requested gear.
        /// </summary>
        private void BeginShift(byte plid, byte targetGear)
        {
            gearShiftTargets[plid] = targetGear;
            currentGears[plid] = targetGear;
            currentClutchValues[plid] = config.ClutchFullyPressed;
            clutchStates[plid] = ClutchState.Pressing;
            shiftPhases[plid] = GearShiftPhase.PressClutch;
            shiftPhaseStartedAt[plid] = DateTime.UtcNow;
            clutchTimers[plid] = DateTime.UtcNow;

            logger.Log(
                $"PLID={plid} SHIFT START: target={FormatGear(targetGear)}, actual={FormatGear(GetReferenceGear(plid))}");
        }

        /// <summary>
        ///     Release the clutch over the configured number of steps and return to idle once complete.
        /// </summary>
        private void ApplyClutchRelease(byte plid, double elapsedMs)
        {
            var releaseStep = (int)(elapsedMs / config.ClutchReleaseIntervalMs);

            if (releaseStep >= config.ClutchReleaseSteps)
            {
                currentClutchValues[plid] = config.ClutchReleased;
                clutchStates[plid] = ClutchState.Released;
                shiftPhases[plid] = GearShiftPhase.Idle;
                lastShiftTimes[plid] = DateTime.UtcNow;
                logger.Log(
                    $"PLID={plid} CLUTCH: Released after verified shift to {FormatGear(gearShiftTargets[plid])}");
                return;
            }

            var releaseProgress = (double)releaseStep / config.ClutchReleaseSteps;
            currentClutchValues[plid] = (int)(config.ClutchFullyPressed * (1 - releaseProgress));
        }

        /// <summary>
        ///     Return the latest trustworthy LFS gear when available, otherwise fall back to the commanded gear.
        /// </summary>
        private byte GetReferenceGear(byte plid)
        {
            return TryGetFreshActualGear(plid, out var actualGear) ? actualGear : currentGears[plid];
        }

        /// <summary>
        ///     Try to read a recent LFS-confirmed gear value from cached AII telemetry.
        /// </summary>
        private bool TryGetFreshActualGear(byte plid, out byte actualGear)
        {
            actualGear = 1;

            if (!actualGears.TryGetValue(plid, out actualGear))
                return false;

            if (!actualGearTimestamps.TryGetValue(plid, out var timestamp))
                return false;

            return DateTime.UtcNow - timestamp <= ActualGearFreshnessWindow;
        }

        /// <summary>
        ///     Initialize any missing state containers so external callers can safely query gearbox info at any time.
        /// </summary>
        private void EnsureStateInitialized(byte plid)
        {
            if (!currentGears.ContainsKey(plid))
                InitializeGearbox(plid);
        }

        /// <summary>
        ///     Apply basic hysteresis so small speed oscillations do not trigger rapid gear hunting.
        /// </summary>
        private byte ApplyGearHysteresis(byte plid, byte desiredGear, double speedKmh, AIConfig.DrivingMode drivingMode)
        {
            var currentGear = GetReferenceGear(plid);
            var upHys = Math.Max(0, config.GearUpshiftHysteresisKmh);
            var downHys = Math.Max(0, config.GearDownshiftHysteresisKmh);
            var upshiftMultiplier = drivingMode == AIConfig.DrivingMode.Race
                ? Math.Max(1.0, config.RaceUpshiftThresholdMultiplier)
                : 1.0;
            var thresholds = config.GearSpeedThresholds ?? Array.Empty<double>();

            var upIndex = currentGear - 2;
            if (upIndex >= 0 && upIndex < thresholds.Length)
            {
                var upThreshold = thresholds[upIndex] * upshiftMultiplier;
                if (desiredGear > currentGear && speedKmh < upThreshold + upHys)
                    desiredGear = currentGear;
            }

            var downIndex = currentGear - 3;
            if (downIndex >= 0 && downIndex < thresholds.Length)
            {
                var downThreshold = thresholds[downIndex];
                if (desiredGear < currentGear && speedKmh > downThreshold - downHys)
                    desiredGear = currentGear;
            }

            return desiredGear;
        }

        /// <summary>
        ///     Convert an LFS gear index into a readable debug label.
        /// </summary>
        private static string FormatGear(byte gear)
        {
            return gear switch
            {
                0 => "R",
                1 => "N",
                255 => "?",
                _ => (gear - 1).ToString()
            };
        }
    }
}
