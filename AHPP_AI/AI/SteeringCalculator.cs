using System;
using System.Collections.Generic;
using AHPP_AI.Debug;
using AHPP_AI.Util;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Handles steering calculations for AI vehicles
    /// </summary>
    public class SteeringCalculator
    {
        // Constants for steering behavior
        private const int STEERING_CENTER = 32768; // Center of steering range (straight)
        private const int MAX_STEERING = 65535; // Maximum steering value
        private const int MIN_STEERING = 1; // Minimum steering value

        // Constants for steering smoothing
        private const double SHARP_TURN_THRESHOLD = 16384; // 90 degrees in LFS units
        private const double MEDIUM_TURN_THRESHOLD = 8192; // 45 degrees in LFS units
        private const double GENTLE_TURN_THRESHOLD = 4096; // 22.5 degrees in LFS units

        // Memory of previous steering directions to prevent oscillation
        private readonly Dictionary<byte, SteeringMemory> carSteeringMemory = new Dictionary<byte, SteeringMemory>();
        private readonly Logger logger;

        /// <summary>
        ///     Initializes a new instance of the SteeringCalculator
        /// </summary>
        /// <param name="logger">Logger for debugging information</param>
        public SteeringCalculator(Logger logger = null)
        {
            this.logger = logger;
        }

        /// <summary>
        ///     Calculate steering value based on target position and current heading
        /// </summary>
        /// <param name="plid">Player ID for the car</param>
        /// <param name="targetX">X coordinate of target relative to car</param>
        /// <param name="targetY">Y coordinate of target relative to car</param>
        /// <param name="currentHeading">Current heading of the car (LFS format)</param>
        /// <param name="currentSpeed">Current speed in km/h for adaptive steering</param>
        /// <returns>Steering information including value, desired heading, and error</returns>
        public SteeringInfo CalculateSteering(byte plid, double targetX, double targetY, double currentHeading,
            double currentSpeed = 0)
        {
            // Initialize memory for this car if not already present
            if (!carSteeringMemory.ContainsKey(plid)) carSteeringMemory[plid] = new SteeringMemory();

            // Calculate desired heading to target using the utility method
            var desiredHeading = CoordinateUtils.CalculateHeadingToTarget(targetX, targetY);

            // Make sure current heading is normalized to 0-65535 range
            var normalizedCurrentHeading = CoordinateUtils.NormalizeHeading((int)currentHeading);

            // Calculate heading error with proper wrap-around
            var headingError = CoordinateUtils.CalculateHeadingError(normalizedCurrentHeading, desiredHeading);

            // Get car's memory of steering direction
            var memory = carSteeringMemory[plid];

            // Handle near-180 degree case (common when car has to do a U-turn) by using memory
            // to maintain consistent turn direction and avoid oscillation
            if (Math.Abs(headingError) > CoordinateUtils.HALF_CIRCLE - 1000)
                // Use the previously established turn direction if it exists
                if (memory.LastTurnDirection != 0)
                {
                    // Force error to be in the same direction as previous turns
                    headingError = memory.LastTurnDirection > 0
                        ? CoordinateUtils.HALF_CIRCLE - 500
                        : -CoordinateUtils.HALF_CIRCLE + 500;

                    if (logger != null)
                        logger.Log($"Near 180 case - Using memory direction: {headingError}");
                }

            // Update steering history for more stable control
            memory.UpdateSteeringHistory(headingError);

            // Calculate steering value
            var steeringValue = CalculateSteeringValue(headingError, memory, currentSpeed);

            // For debug logging if enabled
            if (logger != null && Math.Abs(headingError) > 500)
                logger.Log(
                    $"Steering: Error={headingError}, Desired={desiredHeading}, Current={normalizedCurrentHeading}, Value={steeringValue}");

            return new SteeringInfo
            {
                SteeringValue = steeringValue,
                DesiredHeading = desiredHeading,
                HeadingError = headingError
            };
        }

        /// <summary>
        ///     Calculates the steering value based on the heading error and steering memory
        /// </summary>
        private int CalculateSteeringValue(int headingError, SteeringMemory memory, double currentSpeed)
        {
            int steeringValue;
            var errorMagnitude = Math.Abs(headingError);

            // Adaptive steering - less sensitive at higher speeds
            var speedFactor = Math.Max(0.5, Math.Min(1.0, 50.0 / (currentSpeed + 10.0)));

            // For large errors (near 180 degrees), use memory to avoid oscillation
            if (errorMagnitude > SHARP_TURN_THRESHOLD)
            {
                int turnDirection;

                // If nearly 180° (within 10° of it), use previous direction to avoid oscillation
                if (Math.Abs(Math.Abs(headingError) - CoordinateUtils.HALF_CIRCLE) < 1820)
                    turnDirection = memory.GetConsistentTurnDirection();
                else
                    // Otherwise use the shortest turn direction
                    turnDirection = Math.Sign(headingError);

                // Calculate intensity based on how far we need to turn
                var turnStrength = Math.Min(1.0, (errorMagnitude - GENTLE_TURN_THRESHOLD) / 24576.0);

                // Apply speed factor and smooth turn curve
                turnStrength *= speedFactor;
                turnStrength = Math.Pow(turnStrength, 0.85); // Less aggressive at lower error

                // Calculate steering value - full at 180°, gradually less as we approach target
                steeringValue = STEERING_CENTER + (int)(turnDirection * turnStrength * (STEERING_CENTER - 1));
            }
            else
            {
                // Reset direction memory when error is small enough
                memory.ResetDirectionMemory();

                // Proportional control for smaller errors
                var factor = Math.Min(1.0, errorMagnitude / SHARP_TURN_THRESHOLD);

                // Apply speed factor and use progressive curve for smoother handling
                factor *= speedFactor;
                var smoothFactor = Math.Pow(factor, 0.7);

                // Calculate steering value with progressive control
                steeringValue = STEERING_CENTER + (int)(Math.Sign(headingError) * smoothFactor * (STEERING_CENTER - 1));
            }

            // Ensure steering is within valid range
            steeringValue = Math.Max(MIN_STEERING, Math.Min(MAX_STEERING, steeringValue));

            // Update memory with new steering value
            memory.LastSteeringValue = steeringValue;

            return steeringValue;
        }

        /// <summary>
        ///     Holds memory of steering behavior for a car to prevent oscillation
        /// </summary>
        private class SteeringMemory
        {
            private const int MAX_HISTORY_SIZE = 5;

            // Error history for smoothing
            private readonly Queue<int> errorHistory = new Queue<int>();

            // Previous steering direction and values
            public int LastTurnDirection { get; set; }
            public int DirectionChangeCounter { get; set; }
            public int LastSteeringValue { get; set; } = STEERING_CENTER;

            /// <summary>
            ///     Updates the steering error history
            /// </summary>
            public void UpdateSteeringHistory(int headingError)
            {
                // Add new error to history
                errorHistory.Enqueue(headingError);

                // Keep history at maximum size
                if (errorHistory.Count > MAX_HISTORY_SIZE) errorHistory.Dequeue();

                // Track direction changes for oscillation detection
                var newDirection = Math.Sign(headingError);
                if (newDirection != 0 && LastTurnDirection != 0 && newDirection != LastTurnDirection)
                    DirectionChangeCounter++;
                else if (newDirection != 0) DirectionChangeCounter = Math.Max(0, DirectionChangeCounter - 1);

                LastTurnDirection = newDirection;
            }

            /// <summary>
            ///     Gets a consistent turning direction based on history to prevent oscillation
            /// </summary>
            public int GetConsistentTurnDirection()
            {
                // If oscillating rapidly, use the most consistent recent direction
                if (DirectionChangeCounter >= 3)
                {
                    // Count positive and negative errors in history
                    int positiveCount = 0, negativeCount = 0;
                    foreach (var error in errorHistory)
                        if (error > 0) positiveCount++;
                        else if (error < 0) negativeCount++;

                    // Use the predominant direction
                    if (positiveCount > negativeCount) return 1;
                    if (negativeCount > positiveCount) return -1;
                }

                // Default to last direction, or calculate from last steering value if no direction yet
                return LastTurnDirection != 0 ? LastTurnDirection :
                    LastSteeringValue > STEERING_CENTER ? 1 :
                    LastSteeringValue < STEERING_CENTER ? -1 : 1; // Default to right turn if center
            }

            /// <summary>
            ///     Resets the direction memory when error is small enough
            /// </summary>
            public void ResetDirectionMemory()
            {
                DirectionChangeCounter = 0;
            }
        }
    }

    /// <summary>
    ///     Contains information about steering calculations
    /// </summary>
    public struct SteeringInfo
    {
        /// <summary>
        ///     The calculated steering value (1-65535, 32768 is center)
        /// </summary>
        public int SteeringValue;

        /// <summary>
        ///     The desired heading in LFS format (0-65535)
        /// </summary>
        public double DesiredHeading;

        /// <summary>
        ///     The heading error (-32768 to 32768)
        /// </summary>
        public double HeadingError;
    }
}