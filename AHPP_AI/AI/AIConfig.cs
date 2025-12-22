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

        // Basic AI configuration
        public int NumberOfAIs { get; set; } = 0;

        public int WaitTimeToSpawn { get; set; } = 10000;

        public RouteMode WaypointSource { get; set; } = RouteMode.Recorded;

        public string TrafficRouteName { get; set; } = "main_loop";

        // Advanced route settings
        public string SpawnRouteName { get; set; } = "pit_entry";
        public string MainRouteName { get; set; } = "main_loop";
        public List<string> BranchRouteNames { get; set; } = new List<string> { "detour1", "detour2", "detour3" };

        // Throttle and brake settings
        public int ThrottleBase { get; set; } = 0;
        public int BrakeBase { get; set; } = 10000;

        // Steering settings
        public int MaxSteering { get; set; } = 65535;
        public int MinSteering { get; set; } = 1;
        public int SteeringCenter { get; set; } = 32768;

        // Gearbox settings
        public int ShiftDelayMs { get; set; } = 500;

        // Clutch settings
        public int ClutchFullyPressed { get; set; } = 65535;
        public int ClutchReleased { get; set; } = 0;
        public int ClutchPressDelayMs { get; set; } = 100;
        public int ClutchReleaseSteps { get; set; } = 5;
        public int ClutchReleaseIntervalMs { get; set; } = 100;

        // Waypoint and recovery settings
        public bool WallRecoveryEnabled { get; set; } = false;
        public int WaypointTimeoutSeconds { get; set; } = 30;
        public int ProgressCheckIntervalMs { get; set; } = 5000;
        public double MinRequiredProgress { get; set; } = 5.0;
        public int MaxRecoveryAttempts { get; set; } = 5;
        public double MinSpeedThreshold { get; set; } = 0.5;
        public int StationaryCheckCount { get; set; } = 3;

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
            return Math.Max(
                WaypointMinThreshold,
                Math.Min(WaypointMaxThreshold, speedKmh * WaypointThresholdSpeedFactor + WaypointMinThreshold)
            );
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
