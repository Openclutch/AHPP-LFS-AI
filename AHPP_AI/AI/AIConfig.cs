using System;
using System.Collections.Generic;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Configuration parameters for the AI controller
    /// </summary>
    public class AIConfig
    {
        public enum RouteMode
        {
            Recorded
        }

        public enum PassByReactionMode
        {
            Flash,
            Horn,
            FlashAndHorn
        }

        // Basic AI configuration
        public int NumberOfAIs { get; set; } = 0;

        public int WaitTimeToSpawn { get; set; } = 10000;

        // Population manager settings
        public int MaxPlayers { get; set; } = 48;
        public int ReservedSlots { get; set; } = 10;
        public double AiFillRatio { get; set; } = 0.80;
        public int MinAIs { get; set; } = 0;
        public int MaxAIs { get; set; } = 38;
        public int AdjustIntervalMs { get; set; } = 10000;
        public int SpawnBatchSize { get; set; } = 1;
        public int RemoveBatchSize { get; set; } = 1;
        public bool AutoManagePopulation { get; set; } = true;

        public RouteMode WaypointSource { get; set; } = RouteMode.Recorded;

        public string TrafficRouteName { get; set; } = "main_loop";

        // Advanced route settings
        public string SpawnRouteName { get; set; } = "pit_entry";
        public string MainRouteName { get; set; } = "main_loop";
        public string MainAlternateRouteName { get; set; } = string.Empty;
        public List<string> BranchRouteNames { get; set; } = new List<string>();

        // Lane change settings
        public double LaneChangeInitialCheckIntervalSeconds { get; set; } = 30.0;
        public double LaneChangePostCooldownIntervalSeconds { get; set; } = 10.0;
        public double LaneChangeMergeChance { get; set; } = 0.30;
        public double LaneChangeCooldownSeconds { get; set; } = 180.0;
        public double LaneChangeSafetyCheckDistanceMeters { get; set; } = 25.0;
        public double LaneChangeSafetyCheckHalfWidthMeters { get; set; } = 3.5;
        public double LaneChangeMaxParallelDistanceMeters { get; set; } = 12.0;
        public double LaneChangeMaxParallelHeadingDegrees { get; set; } = 45.0;
        public double LaneChangeParallelLookaheadSeconds { get; set; } = 1.5;
        public double LaneChangeParallelLookaheadMinMeters { get; set; } = 20.0;
        public double LaneChangeMaxMergeSpeedKmh { get; set; } = 60.0;
        public double LaneChangeSignalLeadTimeSeconds { get; set; } = 3.0;
        public double LaneChangeSignalMinimumDurationSeconds { get; set; } = 5.0;
        public double LaneChangeTransitionLengthMeters { get; set; } = 25.0;
        public int LaneChangeTransitionPointCount { get; set; } = 12;
        public int LaneChangeTargetAheadWaypoints { get; set; } = 4;

        // Player interaction settings
        public bool PassByReactionEnabled { get; set; } = true;
        public double PassByReactionChance { get; set; } = 0.10;
        public double PassBySpeedThresholdKmh { get; set; } = 120.0;
        public double PassByReactionDurationSeconds { get; set; } = 2.0;
        public double PassByReactionDistanceMeters { get; set; } = 25.0;
        public PassByReactionMode PassByMode { get; set; } = PassByReactionMode.FlashAndHorn;

        // Throttle and brake settings
        public int ThrottleBase { get; set; } = 0;
        public int BrakeBase { get; set; } = 10000;

        // Steering settings
        public int MaxSteering { get; set; } = 65535;
        public int MinSteering { get; set; } = 1;
        public int SteeringCenter { get; set; } = 32768;
        public double SteeringResponseDamping { get; set; } = 1.0;
        public double SteeringDeadzoneDegrees { get; set; } = 0.0;
        public bool UsePurePursuitSteering { get; set; } = true;
        public double PurePursuitLookaheadMinMeters { get; set; } = 6.0;
        public double PurePursuitLookaheadMaxMeters { get; set; } = 25.0;
        public double PurePursuitLookaheadSpeedFactor { get; set; } = 0.35;
        public double PurePursuitWheelbaseMeters { get; set; } = 2.5;
        public double PurePursuitSteeringGain { get; set; } = 1.0;
        public double PurePursuitMaxSteerDegrees { get; set; } = 25.0;

        // Gearbox settings
        public int ShiftDelayMs { get; set; } = 500;
        public int GearUpshiftHysteresisKmh { get; set; } = 5;
        public int GearDownshiftHysteresisKmh { get; set; } = 5;
        public int GearShiftMinIntervalMs { get; set; } = 900;
        public string BuildVersion { get; set; } = "dev";

        // Clutch settings
        public int ClutchFullyPressed { get; set; } = 65535;
        public int ClutchReleased { get; set; } = 0;
        public int ClutchPressDelayMs { get; set; } = 100;
        public int ClutchHoldAfterShiftMs { get; set; } = 75;
        public int ClutchReleaseSteps { get; set; } = 5;
        public int ClutchReleaseIntervalMs { get; set; } = 100;
        public int StallPreventionRpm { get; set; } = 500;
        public int StallPreventionReleaseRpm { get; set; } = 900;
        public int StallPreventionHoldMs { get; set; } = 750;

        // Waypoint and recovery settings
        public bool WallRecoveryEnabled { get; set; } = true;
        public int WaypointTimeoutSeconds { get; set; } = 30;
        public int ProgressCheckIntervalMs { get; set; } = 5000;
        public double MinRequiredProgress { get; set; } = 5.0;
        public int MaxRecoveryAttempts { get; set; } = 5;
        public int MaxFailedRecoveryCycles { get; set; } = 2;
        public double ProgressAdvanceResetDistanceMeters { get; set; } = 10.0;
        public int RecoveryShortReverseMs { get; set; } = 1200;
        public int RecoveryLongReverseMs { get; set; } = 2000;
        public int RecoveryCooldownMs { get; set; } = 750;
        public int RecoveryValidationWindowMs { get; set; } = 2000;
        public double RecoverySuccessDistanceMeters { get; set; } = 3.0;
        public double RecoverySuccessSpeedKmh { get; set; } = 8.0;
        public double RecoveryPositionChangeThreshold { get; set; } = 2.0;
        public double RecoveryProgressStallThreshold { get; set; } = 0.2;
        public int RecoveryStuckCheckIntervalMs { get; set; } = 1000;
        public double RecoveryLowSpeedThresholdKmh { get; set; } = 5.0;
        public int RecoveryDetectionsBeforeAction { get; set; } = 2;
        public int RecoveryMaxFailureCount { get; set; } = 3;
        public int RecoveryStallReverseTrigger { get; set; } = 3;
        public double MinSpeedThreshold { get; set; } = 0.5;
        public int StationaryCheckCount { get; set; } = 3;
        public int LookaheadWaypoints { get; set; } = 2;
        public double WaypointProximityMultiplier { get; set; } = 1.0;

        // Speed thresholds for gear changes
        public double[] GearSpeedThresholds { get; set; } =
        {
            20.0, // 1st gear below km/h
            40.0, // 2nd gear below km/h
            60.0, // 3rd gear below km/h
            80.0, // 4th gear below km/h
            100.0 // 5th gear below km/h
        };

        // Collision avoidance settings
        public double CollisionDetectionRangeM { get; set; } = 30.0; // Detection range in meters
        public double CollisionDetectionAngle { get; set; } = 45.0; // Half-angle in degrees (total 180° arc)
        public double MinimumSafetyDistanceM { get; set; } = 10.0; // Minimum safe distance in meters
        public double CollisionDetectionHalfWidthM { get; set; } = 2.5; // Half-width corridor to consider a car "ahead"

        // Waypoint approach
        public double WaypointMinThreshold { get; set; } = 1.5;
        public double WaypointMaxThreshold { get; set; } = 5.0;
        public double WaypointThresholdSpeedFactor { get; set; } = 0.1;

        // Debug settings
        public bool DebugEnabled { get; set; } = false;

        /// <summary>
        ///     Calculate waypoint threshold based on speed
        /// </summary>
        /// <param name="speedKmh">Current speed in km/h</param>
        /// <returns>Appropriate waypoint threshold in meters</returns>
        public double CalculateWaypointThreshold(double speedKmh)
        {
            var baseThreshold = Math.Max(
                WaypointMinThreshold,
                Math.Min(WaypointMaxThreshold, speedKmh * WaypointThresholdSpeedFactor + WaypointMinThreshold)
            );

            return Math.Max(0.1, baseThreshold * Math.Max(0.1, WaypointProximityMultiplier));
        }

        /// <summary>
        ///     Determine appropriate gear for current speed
        /// </summary>
        /// <param name="speedKmh">Current speed in km/h</param>
        /// <returns>Appropriate gear (2=1st, 3=2nd, etc.)</returns>
        public byte CalculateDesiredGear(double speedKmh)
        {
            for (var i = 0; i < GearSpeedThresholds.Length; i++)
                if (speedKmh < GearSpeedThresholds[i])
                    return (byte)(i + 2); // +2 because 2=1st gear in LFS

            return 7; // 6th gear for high speeds
        }
    }
}
