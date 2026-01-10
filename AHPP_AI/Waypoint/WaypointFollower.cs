using System;
using System.Collections.Generic;
using AHPP_AI.AI;
using AHPP_AI.Debug;
using AHPP_AI.Util;
using InSimDotNet.Packets;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    ///     Handles AI navigation along waypoints, including target selection and progress tracking
    /// </summary>
    public class WaypointFollower
    {
        private enum ProgressState
        {
            Unknown,
            Progressing,
            Slow,
            MovingAway,
            ReverseRecoveryPending
        }

        // Constants for approach curve
        private const double APPROACH_DISTANCE = 50.0; // Distance to generate the approach curve in meters
        private const double INITIAL_APPROACH_SPEED = 10.0; // Speed in km/h for initial approach
        private const double APPROACH_CURVE_RADIUS = 25.0; // Radius of approach curve in meters
        private const int APPROACH_CURVE_POINTS = 10; // Number of points in approach curve

        // Waypoint tracking
        private readonly Dictionary<byte, List<Util.Waypoint>> aiPaths = new Dictionary<byte, List<Util.Waypoint>>();
        private readonly Dictionary<byte, List<Vec>> approachCurvePoints = new Dictionary<byte, List<Vec>>();
        private readonly AIConfig config;
        private readonly Dictionary<byte, int> currentApproachPointIndex = new Dictionary<byte, int>();
        private readonly Dictionary<byte, Queue<int>> headingErrorHistory = new Dictionary<byte, Queue<int>>();
        private readonly Dictionary<byte, bool> inRecoveryMode = new Dictionary<byte, bool>();
        private readonly Dictionary<byte, int> failedRecoveryCycles = new Dictionary<byte, int>();
        private readonly Dictionary<byte, ProgressState> progressStates = new Dictionary<byte, ProgressState>();
        private readonly Dictionary<byte, bool> approachFinished = new Dictionary<byte, bool>();

        // Path entry handling
        private readonly Dictionary<byte, bool> isFirstApproach = new Dictionary<byte, bool>();

        // Progress tracking
        private readonly Dictionary<byte, double> lastDistanceToWaypoint = new Dictionary<byte, double>();
        private readonly Dictionary<byte, DateTime> lastProgressCheckTimes = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, int> movingAwayCounts = new Dictionary<byte, int>();
        private readonly Logger logger;
        private readonly Dictionary<byte, int> lookAheadWaypointIndices = new Dictionary<byte, int>();
        private readonly Dictionary<byte, int> recoveryAttempts = new Dictionary<byte, int>();
        private readonly Dictionary<byte, int> stationaryChecks = new Dictionary<byte, int>();
        private readonly SteeringCalculator steeringCalculator;
        private readonly Dictionary<byte, double> targetSpeeds = new Dictionary<byte, double>();
        private readonly Dictionary<byte, double> manualTargetSpeeds = new Dictionary<byte, double>();
        private readonly Dictionary<byte, int> targetWaypointIndices = new Dictionary<byte, int>();
        private readonly Dictionary<byte, DateTime> waypointTimers = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, int> lastBrakeValues = new Dictionary<byte, int>();

        public WaypointFollower(AIConfig config, Logger logger, SteeringCalculator steeringCalculator)
        {
            this.config = config;
            this.logger = logger;
            this.steeringCalculator = steeringCalculator;
        }

        public void SetManualTargetSpeed(byte plid, double speed)
        {
            manualTargetSpeeds[plid] = speed;
        }

        public void ClearManualTargetSpeed(byte plid)
        {
            if (manualTargetSpeeds.ContainsKey(plid))
                manualTargetSpeeds.Remove(plid);
        }

        /// <summary>
        ///     Initialize path for a car
        /// </summary>
        public bool InitializePath(byte plid, CompCar car, WaypointManager waypointManager, AIConfig config,
            List<ObjectInfo> layoutObjects)
        {
            // If a non-empty path already exists, just return. Empty paths should be rebuilt.
            if (aiPaths.TryGetValue(plid, out var existingPath))
            {
                if (existingPath != null && existingPath.Count > 0)
                    return true;

                logger.LogWarning($"PLID={plid} Existing path is empty, rebuilding from recorded route or fallback");
            }

            var currentPos = new Vec(car.X, car.Y, car.Z);

            // Get path from waypoint manager
            aiPaths[plid] = waypointManager.GetPathForCar(currentPos, config);

            if (aiPaths[plid] == null || aiPaths[plid].Count == 0)
            {
                logger.LogError($"PLID={plid} Failed to get valid path for car");
                return false;
            }

            // Find best starting waypoint based on car position and heading
            var isLoop = pathIsLoop.TryGetValue(plid, out var loopFlag) ? loopFlag : true;
            var (bestIndex, clockwise) = waypointManager.FindBestWaypointForDirection(
                currentPos, car.Heading, aiPaths[plid], isLoop);

            if (!clockwise)
            {
                aiPaths[plid].Reverse();
                bestIndex = aiPaths[plid].Count - 1 - bestIndex;
            }

            // Initialize waypoint indices
            targetWaypointIndices[plid] = bestIndex;
            var lookahead = Math.Max(1, config.LookaheadWaypoints);
            lookAheadWaypointIndices[plid] = GetAdvanceIndex(plid, bestIndex, lookahead);

            // IMPORTANT: Default to not using approach curve
            isFirstApproach[plid] = false;

            // Initialize tracking data
            targetSpeeds[plid] = INITIAL_APPROACH_SPEED;
            waypointTimers[plid] = DateTime.Now;
            lastDistanceToWaypoint[plid] = double.MaxValue;
            recoveryAttempts[plid] = 0;
            failedRecoveryCycles[plid] = 0;
            inRecoveryMode[plid] = false;
            progressStates[plid] = ProgressState.Progressing;
            headingErrorHistory[plid] = new Queue<int>();
            stationaryChecks[plid] = 0;
            lastProgressCheckTimes[plid] = DateTime.Now;
            movingAwayCounts[plid] = 0;
            currentApproachPointIndex[plid] = 0;
            lastBrakeValues[plid] = 0;

            return true;
        }

        public void CheckAndUpdateApproachCurve(byte plid, CompCar car)
        {
            if (approachFinished.TryGetValue(plid, out var finished) && finished)
                return;

            if (!aiPaths.TryGetValue(plid, out var path) || path == null || path.Count == 0)
                return;

            if (!targetWaypointIndices.TryGetValue(plid, out var storedIndex))
                return;

            var currentPos = new Vec(car.X, car.Y, car.Z);
            var targetIndex = ClampIndex(plid, storedIndex);
            targetWaypointIndices[plid] = targetIndex;
            var targetWaypoint = path[targetIndex];
            var waypointPos = new Vec(targetWaypoint.Position.X, targetWaypoint.Position.Y);
            var distance = CoordinateUtils.CalculateDistance(currentPos, waypointPos);

            var hasApproachFlag = isFirstApproach.TryGetValue(plid, out var firstApproach);
            // Only generate approach curve when we're close to the track (e.g., within 20m)
            if (distance < APPROACH_DISTANCE)
            {
                if (!hasApproachFlag || !firstApproach || !approachCurvePoints.ContainsKey(plid))
                {
                    isFirstApproach[plid] = true;
                    approachFinished[plid] = false;
                    GenerateApproachCurve(plid, car);
                    logger.Log($"PLID={plid} Approach curve activated at distance: {distance:F1}m");
                }
            }
            else
            {
                // Too far from track, don't use approach curve
                isFirstApproach[plid] = false;
                logger.Log($"PLID={plid} Too far from waypoint ({distance:F1}m), not using approach curve");
            }
        }

        /// <summary>
        ///     Generate a smooth approach curve to join the path
        /// </summary>
        private void GenerateApproachCurve(byte plid, CompCar car)
        {
            if (!aiPaths.TryGetValue(plid, out var path) || path == null || path.Count < 2)
            {
                logger.LogWarning($"PLID={plid} Cannot generate approach curve: Missing path or indices");
                return;
            }

            if (!targetWaypointIndices.TryGetValue(plid, out var storedIndex))
            {
                logger.LogWarning($"PLID={plid} Cannot generate approach curve: Missing target index");
                return;
            }

            // Current car position
            var carPos = new Vec(car.X, car.Y, car.Z);

            // Target waypoint
            var targetIndex = ClampIndex(plid, storedIndex);
            targetWaypointIndices[plid] = targetIndex;
            var targetWaypoint = path[targetIndex];

            // Look-ahead waypoint (use 3 waypoints ahead for smoother approach)
            var lookAheadIndex = GetAdvanceIndex(plid, targetIndex, 3);
            lookAheadWaypointIndices[plid] = lookAheadIndex;
            var lookAheadWaypoint = path[lookAheadIndex];

            // Calculate approach curve using a Bezier curve
            var curvePoints = new List<Vec>();

            // Start point is current car position
            var p0 = carPos;

            // End point is the target waypoint
            var p2 = new Vec(targetWaypoint.Position.X, targetWaypoint.Position.Y, carPos.Z);

            // Calculate path direction vector (converting from int to double)
            double pathDx = lookAheadWaypoint.Position.X - targetWaypoint.Position.X;
            double pathDy = lookAheadWaypoint.Position.Y - targetWaypoint.Position.Y;
            var pathLength = Math.Sqrt(pathDx * pathDx + pathDy * pathDy);

            if (pathLength > 0)
            {
                // Normalize path direction
                pathDx /= pathLength;
                pathDy /= pathLength;

                // Calculate control point position (use a larger value for APPROACH_CURVE_RADIUS)
                var controlPointDistance = APPROACH_CURVE_RADIUS * 2.0;

                // Calculate offset from target waypoint for smoother entry
                // Use backwards direction from path to create a smooth entry
                var p1X = targetWaypoint.Position.X - (int)(pathDx * controlPointDistance * 65536.0);
                var p1Y = targetWaypoint.Position.Y - (int)(pathDy * controlPointDistance * 65536.0);
                var p1 = new Vec(p1X, p1Y, carPos.Z);

                // Generate points along the curve (use more points for smoother approach)
                for (var i = 0; i <= APPROACH_CURVE_POINTS * 2; i++)
                {
                    var t = i / (double)(APPROACH_CURVE_POINTS * 2);

                    // Quadratic Bezier curve formula: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
                    var mt = 1.0 - t;
                    var mt2 = mt * mt;
                    var t2 = t * t;

                    var x = (int)(mt2 * p0.X + 2 * mt * t * p1.X + t2 * p2.X);
                    var y = (int)(mt2 * p0.Y + 2 * mt * t * p1.Y + t2 * p2.Y);
                    var z = (int)(mt2 * p0.Z + 2 * mt * t * p1.Z + t2 * p2.Z);

                curvePoints.Add(new Vec(x, y, z));
            }

            // Store the curve points
            approachCurvePoints[plid] = curvePoints;

            logger.Log($"PLID={plid} Generated {curvePoints.Count} curve points for smooth approach");
        }
            else
            {
                // Fallback if path is too short - use direct line
                curvePoints.Add(carPos);
                curvePoints.Add(new Vec(targetWaypoint.Position.X, targetWaypoint.Position.Y, carPos.Z));
                approachCurvePoints[plid] = curvePoints;

                logger.Log($"PLID={plid} Using direct approach (path too short for curve)");
            }
        }

        /// <summary>
        ///     Get target waypoint for a car
        /// </summary>
        public Util.Waypoint GetTargetWaypoint(byte plid)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count == 0)
                return new Util.Waypoint(); // Empty waypoint

            if (!targetWaypointIndices.ContainsKey(plid))
            {
                targetWaypointIndices[plid] = 0;
                logger.LogWarning($"PLID={plid} Target waypoint index not found, defaulting to 0");
            }

            var targetIndex = ClampIndex(plid, targetWaypointIndices[plid]);
            targetWaypointIndices[plid] = targetIndex;

            return aiPaths[plid][targetIndex];
        }

        /// <summary>
        ///     Get a lookahead waypoint based on the configured offset.
        /// </summary>
        public Util.Waypoint GetLookaheadWaypoint(byte plid)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count == 0)
                return new Util.Waypoint();

            if (!targetWaypointIndices.ContainsKey(plid))
                targetWaypointIndices[plid] = 0;

            var path = aiPaths[plid];
            var currentIndex = ClampIndex(plid, targetWaypointIndices[plid]);
            var lookaheadOffset = Math.Max(1, config.LookaheadWaypoints);
            var lookaheadIndex = GetAdvanceIndex(plid, currentIndex, lookaheadOffset);

            return path[lookaheadIndex];
        }

        /// <summary>
        ///     Get next waypoint index
        /// </summary>
        public int GetNextWaypointIndex(byte plid)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count == 0)
                return 0;

            if (!targetWaypointIndices.ContainsKey(plid))
                return 0;

            var targetIndex = targetWaypointIndices[plid];
            return GetAdvanceIndex(plid, targetIndex, 1);
        }

        /// <summary>
        ///     Result of a progress evaluation, used to drive unified recovery handling.
        /// </summary>
        public class ProgressEvaluationResult
        {
            public bool ShouldReverse { get; set; }
            public bool ShouldReset { get; set; }
            public bool IsMovingAway { get; set; }
            public bool IsSlowProgress { get; set; }
            public double ProgressDelta { get; set; }
        }

        /// <summary>
        ///     Check progress toward waypoint and surface recovery recommendations.
        /// </summary>
        public ProgressEvaluationResult EvaluateProgress(byte plid, double currentDistance, double speedKmh)
        {
            var result = new ProgressEvaluationResult
            {
                ProgressDelta = 0,
                ShouldReset = false,
                ShouldReverse = false,
                IsMovingAway = false,
                IsSlowProgress = false
            };

            // Ensure dictionaries have the necessary keys
            if (!lastDistanceToWaypoint.ContainsKey(plid))
                lastDistanceToWaypoint[plid] = currentDistance;

            if (!lastProgressCheckTimes.ContainsKey(plid))
                lastProgressCheckTimes[plid] = DateTime.Now;

            if (!recoveryAttempts.ContainsKey(plid))
                recoveryAttempts[plid] = 0;

            if (!movingAwayCounts.ContainsKey(plid))
                movingAwayCounts[plid] = 0;

            if (!failedRecoveryCycles.ContainsKey(plid))
                failedRecoveryCycles[plid] = 0;

            if (!progressStates.ContainsKey(plid))
                progressStates[plid] = ProgressState.Progressing;

            // If the stored distance is an uninitialized sentinel value, seed it with the current reading to avoid
            // overflow-sized progress deltas on the first check.
            var previousDistance = lastDistanceToWaypoint[plid];
            if (double.IsNaN(previousDistance) || double.IsInfinity(previousDistance) ||
                previousDistance == double.MaxValue)
            {
                lastDistanceToWaypoint[plid] = currentDistance;
                lastProgressCheckTimes[plid] = DateTime.Now;
                return result;
            }

            // Check progress at regular intervals
            if ((DateTime.Now - lastProgressCheckTimes[plid]).TotalMilliseconds >= config.ProgressCheckIntervalMs)
            {
                var progress = previousDistance - currentDistance;
                result.ProgressDelta = progress;

                // If we just advanced a waypoint, distances can jump up; ignore negative progress within a small window.
                if (progress < 0 &&
                    previousDistance <= config.ProgressAdvanceResetDistanceMeters &&
                    currentDistance <= config.ProgressAdvanceResetDistanceMeters * 2)
                {
                    movingAwayCounts[plid] = 0;
                    recoveryAttempts[plid] = 0;
                    failedRecoveryCycles[plid] = 0;
                    progressStates[plid] = ProgressState.Progressing;

                    lastDistanceToWaypoint[plid] = currentDistance;
                    lastProgressCheckTimes[plid] = DateTime.Now;
                    return result;
                }

                logger.Log(
                    $"PLID={plid} PROGRESS CHECK: Previous={previousDistance:F1}m, Current={currentDistance:F1}m, Progress={progress:F1}m");

                // If too many failed attempts, signal reset
                if (recoveryAttempts[plid] >= config.MaxRecoveryAttempts)
                {
                    progressStates[plid] = ProgressState.ReverseRecoveryPending;
                    recoveryAttempts[plid] = 0;
                    failedRecoveryCycles[plid] = failedRecoveryCycles.TryGetValue(plid, out var cycles)
                        ? cycles + 1
                        : 1;

                    logger.Log(
                        $"PLID={plid} PROGRESS STALLED: Reverse recovery cycle {failedRecoveryCycles[plid]} of {config.MaxFailedRecoveryCycles}");

                    if (failedRecoveryCycles[plid] >= config.MaxFailedRecoveryCycles)
                    {
                        logger.LogWarning(
                            $"PLID={plid} RECOVERY FAILED: Exceeded recovery cycles, requesting pit or spectate.");
                        result.ShouldReset = true;
                    }
                    else
                    {
                        result.ShouldReverse = true;
                    }

                    lastDistanceToWaypoint[plid] = currentDistance;
                    lastProgressCheckTimes[plid] = DateTime.Now;
                    movingAwayCounts[plid] = 0;
                }
                else if (progress < 0)
                {
                    // Moving away from waypoint; only trigger a reverse when speed is low or the issue persists.
                    var currentCount = movingAwayCounts.TryGetValue(plid, out var count) ? count + 1 : 1;
                    movingAwayCounts[plid] = currentCount;

                    var lowSpeed = speedKmh <= config.RecoveryLowSpeedThresholdKmh * 1.5;
                    var persistent = currentCount >= 2;

                    if (lowSpeed || persistent)
                    {
                        recoveryAttempts[plid]++;
                        progressStates[plid] = ProgressState.MovingAway;
                        result.IsMovingAway = true;
                        result.ShouldReverse = true;
                        logger.Log(
                            $"PLID={plid} MOVING AWAY: Recovery attempt {recoveryAttempts[plid]} of {config.MaxRecoveryAttempts} (speed={speedKmh:F1}, count={currentCount})");
                    }
                    else
                    {
                        result.IsMovingAway = true;
                        progressStates[plid] = ProgressState.MovingAway;
                        logger.Log(
                            $"PLID={plid} MOVING AWAY: Deferring recovery while coasting (speed={speedKmh:F1}, count={currentCount})");
                    }
                }
                else if (progress < config.MinRequiredProgress)
                {
                    progressStates[plid] = ProgressState.Slow;
                    result.IsSlowProgress = true;
                }
                else if (progress > config.MinRequiredProgress)
                {
                    // Good progress, reset recovery attempts
                    movingAwayCounts[plid] = 0;
                    recoveryAttempts[plid] = 0;
                    failedRecoveryCycles[plid] = 0;
                    progressStates[plid] = ProgressState.Progressing;
                }

                // Update for next check
                lastDistanceToWaypoint[plid] = currentDistance;
                lastProgressCheckTimes[plid] = DateTime.Now;
            }
            else
            {
                // If we're not due for a full progress check, still clear moving-away streak when distance improves.
                if (lastDistanceToWaypoint[plid] - currentDistance > 0)
                    movingAwayCounts[plid] = 0;
            }

            result.IsMovingAway |= progressStates[plid] == ProgressState.MovingAway;
            result.IsSlowProgress |= progressStates[plid] == ProgressState.Slow;

            return result;
        }

        /// <summary>
        ///     Check if car has reached waypoint threshold and advance to next waypoint if needed
        /// </summary>
        public void CheckWaypointReached(byte plid, double distanceCurrent, double speedKmh)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null)
                return;

            // Initialize dictionary values if missing
            if (!isFirstApproach.ContainsKey(plid))
                isFirstApproach[plid] = false;

            if (!waypointTimers.ContainsKey(plid))
                waypointTimers[plid] = DateTime.Now;

            // Handle initial approach differently
            if (isFirstApproach[plid])
            {
                // Use a larger threshold for the first waypoint
                if (distanceCurrent < 5.0)
                {
                    isFirstApproach[plid] = false;
                    approachFinished[plid] = true;
                    var waypoint = GetTargetWaypoint(plid);

                    if (!targetSpeeds.ContainsKey(plid))
                        targetSpeeds[plid] = INITIAL_APPROACH_SPEED;

                    // Gradually transition to waypoint speed limit
                    targetSpeeds[plid] = Math.Min(waypoint.SpeedLimit, INITIAL_APPROACH_SPEED * 1.5);
                    logger.Log(
                        $"PLID={plid} FIRST WAYPOINT REACHED: Switching to normal navigation with speed={targetSpeeds[plid]:F1}km/h");

                    // Reset approach point index to ensure we don't try to continue using curve points
                    if (currentApproachPointIndex.ContainsKey(plid))
                        currentApproachPointIndex[plid] = 0;
                }

                return;
            }

            // Rest of the method remains unchanged
            var waypointTimeout = TimeSpan.FromSeconds(config.WaypointTimeoutSeconds);
            if (DateTime.Now - waypointTimers[plid] > waypointTimeout)
            {
                AdvanceToNextWaypoint(plid, "WAYPOINT TIMEOUT");
                return;
            }

            var threshold = config.CalculateWaypointThreshold(speedKmh);

            if (distanceCurrent < threshold)
                AdvanceToNextWaypoint(plid,
                    $"WAYPOINT REACHED: Distance={distanceCurrent:F1}m, Threshold={threshold:F1}m");
        }

        /// <summary>
        ///     Advance to the next waypoint
        /// </summary>
        private void AdvanceToNextWaypoint(byte plid, string reason)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count == 0)
                return;

            // Initialize dictionary values if missing
            if (!targetWaypointIndices.ContainsKey(plid))
                targetWaypointIndices[plid] = 0;

            if (!waypointTimers.ContainsKey(plid))
                waypointTimers[plid] = DateTime.Now;

            if (!recoveryAttempts.ContainsKey(plid))
                recoveryAttempts[plid] = 0;

            if (!inRecoveryMode.ContainsKey(plid))
                inRecoveryMode[plid] = false;

            if (!approachFinished.ContainsKey(plid))
                approachFinished[plid] = false;

            if (!headingErrorHistory.ContainsKey(plid))
                headingErrorHistory[plid] = new Queue<int>();

            if (!stationaryChecks.ContainsKey(plid))
                stationaryChecks[plid] = 0;

            if (!progressStates.ContainsKey(plid))
                progressStates[plid] = ProgressState.Progressing;

            var currentIndex = targetWaypointIndices[plid];
            var nextIndex = GetAdvanceIndex(plid, currentIndex, 1);

            logger.Log($"PLID={plid} {reason}: Moving from {currentIndex} to {nextIndex}");

            targetWaypointIndices[plid] = nextIndex;
            UpdateLookaheadIndex(plid);

            if (!targetSpeeds.ContainsKey(plid))
                targetSpeeds[plid] = 30.0; // Default speed

            targetSpeeds[plid] = aiPaths[plid][nextIndex].SpeedLimit;
            waypointTimers[plid] = DateTime.Now;
            recoveryAttempts[plid] = 0;
            failedRecoveryCycles[plid] = 0;
            inRecoveryMode[plid] = false;
            headingErrorHistory[plid].Clear();
            stationaryChecks[plid] = 0;
            progressStates[plid] = ProgressState.Progressing;
        }

        /// <summary>
        ///     Calculate throttle and brake values based on current speed and target speed
        /// </summary>
        public (int throttle, int brake) CalculateThrottleAndBrake(byte plid, double speedKmh, double headingError)
        {
            // Initialize dictionary values if missing
            if (!targetSpeeds.ContainsKey(plid))
                targetSpeeds[plid] = INITIAL_APPROACH_SPEED;

            if (!isFirstApproach.ContainsKey(plid))
                isFirstApproach[plid] = false;

            var targetSpeed = manualTargetSpeeds.ContainsKey(plid)
                ? manualTargetSpeeds[plid]
                : targetSpeeds[plid];

            // Apply more aggressive speed reduction during first approach
            if (isFirstApproach[plid])
            {
                targetSpeed = INITIAL_APPROACH_SPEED;
                logger.Log($"PLID={plid} FIRST APPROACH: Target speed={targetSpeed:F1}km/h");
            }

            // Adjust target speed based on heading error
            var headingErrorDeg = Math.Abs(headingError) * 360.0 / 65536.0;

            if (headingErrorDeg > 45)
                targetSpeed = Math.Min(targetSpeed, 15);
            else if (headingErrorDeg > 30)
                targetSpeed = Math.Min(targetSpeed, 30);
            else if (headingErrorDeg > 15)
                targetSpeed = Math.Min(targetSpeed, 50);

            var speedError = targetSpeed - speedKmh;

            int throttle, brake;

            if (speedError > config.BrakeCoastSpeedErrorKmh)
            {
                var factor = Math.Min(1.0, speedError / 20.0);
                throttle = config.ThrottleBase + (int)(factor * (65535 - config.ThrottleBase));
                throttle = Math.Max(config.ThrottleBase, Math.Min(65535, throttle));
                brake = SmoothBrake(plid, 0);
            }
            else if (speedError < -config.BrakeApplySpeedErrorKmh)
            {
                var factor = Math.Min(1.0, -speedError / 20.0);
                throttle = 0;
                var desiredBrake = config.BrakeBase + (int)(factor * (65535 - config.BrakeBase));
                desiredBrake = Math.Max(config.BrakeBase, Math.Min(65535, desiredBrake));
                brake = SmoothBrake(plid, desiredBrake);
            }
            else if (speedError < -config.BrakeCoastSpeedErrorKmh)
            {
                throttle = 0;
                brake = SmoothBrake(plid, 0);
            }
            else
            {
                throttle = speedError >= 0 ? config.ThrottleBase / 2 : 0;
                brake = SmoothBrake(plid, 0);
            }

            return (throttle, brake);
        }

        /// <summary>
        ///     Smooth brake application and release to avoid rapid taps.
        /// </summary>
        private int SmoothBrake(byte plid, int desiredBrake)
        {
            if (!lastBrakeValues.ContainsKey(plid))
                lastBrakeValues[plid] = 0;

            var currentBrake = lastBrakeValues[plid];
            int updatedBrake;

            if (desiredBrake > currentBrake)
                updatedBrake = Math.Min(desiredBrake, currentBrake + config.BrakeRiseStep);
            else
                updatedBrake = Math.Max(desiredBrake, currentBrake - config.BrakeReleaseStep);

            lastBrakeValues[plid] = updatedBrake;
            return updatedBrake;
        }

        /// <summary>
        ///     Get debug information about the current waypoint state
        /// </summary>
        public (int targetIndex, int waypointCount, double targetSpeed, bool inRecovery) GetFollowerInfo(byte plid)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null)
                return (0, 0, 0, false);

            // Initialize dictionary values if missing
            if (!targetWaypointIndices.ContainsKey(plid))
                targetWaypointIndices[plid] = 0;

            if (!targetSpeeds.ContainsKey(plid))
                targetSpeeds[plid] = INITIAL_APPROACH_SPEED;

            if (!inRecoveryMode.ContainsKey(plid))
                inRecoveryMode[plid] = false;

            var speed = manualTargetSpeeds.ContainsKey(plid)
                ? manualTargetSpeeds[plid]
                : targetSpeeds[plid];

            return (
                targetWaypointIndices[plid],
                aiPaths[plid].Count,
                speed,
                inRecoveryMode[plid]
            );
        }

        /// <summary>
        ///     Get the list of waypoints for a specific car
        /// </summary>
        public List<Util.Waypoint> GetPath(byte plid)
        {
            if (!aiPaths.ContainsKey(plid))
                return new List<Util.Waypoint>();

            return aiPaths[plid];
        }

        /// <summary>
        ///     Provide a human-friendly label describing recent progress toward the target.
        /// </summary>
        public string GetProgressStateLabel(byte plid)
        {
            if (!progressStates.TryGetValue(plid, out var state))
                return string.Empty;

            return state switch
            {
                ProgressState.Slow => "Slow progress",
                ProgressState.MovingAway => "Moving away",
                ProgressState.ReverseRecoveryPending => "Stuck - reversing",
                _ => string.Empty
            };
        }

        private readonly Dictionary<byte, bool> pathIsLoop = new Dictionary<byte, bool>();

        public void SetPath(byte plid, List<Util.Waypoint> path, int? startIndex = null, bool isLoop = true)
        {
            aiPaths[plid] = path;
            pathIsLoop[plid] = isLoop;
            if (path == null || path.Count == 0)
            {
                targetWaypointIndices[plid] = 0;
                lookAheadWaypointIndices[plid] = 0;
                progressStates[plid] = ProgressState.Unknown;
                recoveryAttempts[plid] = 0;
                failedRecoveryCycles[plid] = 0;
                approachFinished[plid] = false;
                isFirstApproach[plid] = false;
                currentApproachPointIndex[plid] = 0;
                return;
            }

            var clampedIndex = Math.Max(0, Math.Min(path.Count - 1, startIndex ?? 0));
            targetWaypointIndices[plid] = clampedIndex;
            UpdateLookaheadIndex(plid);
            recoveryAttempts[plid] = 0;
            failedRecoveryCycles[plid] = 0;
            progressStates[plid] = ProgressState.Progressing;
            lastDistanceToWaypoint[plid] = double.MaxValue;
            lastProgressCheckTimes[plid] = DateTime.Now;
            movingAwayCounts[plid] = 0;
            approachFinished[plid] = false;
            isFirstApproach[plid] = false;
            currentApproachPointIndex[plid] = 0;
        }

        private void UpdateLookaheadIndex(byte plid)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count == 0)
                return;

            if (!targetWaypointIndices.ContainsKey(plid))
                targetWaypointIndices[plid] = 0;

            var lookaheadOffset = Math.Max(1, config.LookaheadWaypoints);
            lookAheadWaypointIndices[plid] = GetAdvanceIndex(plid, targetWaypointIndices[plid], lookaheadOffset);
        }

        /// <summary>
        /// Advance an index by offset, wrapping only if the current path is looped; otherwise clamp at the end.
        /// </summary>
        private int GetAdvanceIndex(byte plid, int currentIndex, int offset)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count == 0) return 0;
            var path = aiPaths[plid];
            var loop = pathIsLoop.TryGetValue(plid, out var isLoop) ? isLoop : true;
            if (path.Count == 0) return 0;

            if (loop)
                return (currentIndex + offset) % path.Count;

            return Math.Min(path.Count - 1, Math.Max(0, currentIndex + offset));
        }

        /// <summary>
        /// Clamp an index to the bounds of the path for non-looped routes.
        /// </summary>
        private int ClampIndex(byte plid, int index)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count == 0) return 0;
            var path = aiPaths[plid];
            var loop = pathIsLoop.TryGetValue(plid, out var isLoop) ? isLoop : true;
            if (path.Count == 0) return 0;

            if (loop) return ((index % path.Count) + path.Count) % path.Count;

            return Math.Max(0, Math.Min(path.Count - 1, index));
        }

        /// <summary>
        ///     Calculate a lookahead point for pure pursuit steering based on speed and route geometry.
        /// </summary>
        public (double targetX, double targetY, double lookaheadDistanceMeters) CalculatePurePursuitTarget(
            byte plid, double carX, double carY, double speedKmh)
        {
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count == 0)
                return (carX, carY, 0);

            if (!targetWaypointIndices.ContainsKey(plid))
                targetWaypointIndices[plid] = 0;

            var lookaheadDistance = CalculateLookaheadDistance(speedKmh);
            var path = aiPaths[plid];
            var startIndex = ClampIndex(plid, targetWaypointIndices[plid]);
            var remaining = lookaheadDistance;

            var previousX = carX;
            var previousY = carY;

            for (var i = 0; i < path.Count; i++)
            {
                var idx = GetAdvanceIndex(plid, startIndex, i);
                var waypoint = path[idx];
                var wpX = waypoint.Position.X / 65536.0;
                var wpY = waypoint.Position.Y / 65536.0;

                var dx = wpX - previousX;
                var dy = wpY - previousY;
                var segmentLength = Math.Sqrt(dx * dx + dy * dy);

                if (segmentLength >= remaining && segmentLength > 0.001)
                {
                    var t = remaining / segmentLength;
                    var targetX = previousX + dx * t;
                    var targetY = previousY + dy * t;

                    return (targetX, targetY, lookaheadDistance);
                }

                remaining -= segmentLength;
                previousX = wpX;
                previousY = wpY;
            }

            return (previousX, previousY, lookaheadDistance);
        }

        /// <summary>
        ///     Calculate a clamped lookahead distance using speed-based scaling.
        /// </summary>
        private double CalculateLookaheadDistance(double speedKmh)
        {
            var baseDistance = config.PurePursuitLookaheadMinMeters +
                               speedKmh * config.PurePursuitLookaheadSpeedFactor;

            return Math.Max(
                config.PurePursuitLookaheadMinMeters,
                Math.Min(config.PurePursuitLookaheadMaxMeters, baseDistance));
        }

        /// <summary>
        ///     Calculate heading and distance to current target waypoint or approach curve point
        /// </summary>
        public (double distance, int desiredHeading, int headingError) CalculateTargetData(byte plid, double carX,
            double carY, int currentHeading)
        {
            // Initialize dictionary values if missing
            if (!isFirstApproach.ContainsKey(plid))
                isFirstApproach[plid] = false;

            if (!currentApproachPointIndex.ContainsKey(plid))
                currentApproachPointIndex[plid] = 0;

            // If in first approach mode and we have curve points, target the current curve point
            if (isFirstApproach[plid] &&
                approachCurvePoints.ContainsKey(plid) &&
                approachCurvePoints[plid].Count > 0 &&
                currentApproachPointIndex[plid] < approachCurvePoints[plid].Count)
            {
                var curvePoint = approachCurvePoints[plid][currentApproachPointIndex[plid]];
                var pointX = curvePoint.X / 65536.0;
                var pointY = curvePoint.Y / 65536.0;

                // Calculate distance and direction to curve point
                var dxToPoint = pointX - carX;
                var dyToPoint = pointY - carY;
                var distance = Math.Sqrt(dxToPoint * dxToPoint + dyToPoint * dyToPoint);

                // Calculate desired heading to target
                var normalizedHeading = CoordinateUtils.NormalizeHeading(currentHeading);
                var desiredHeading = CoordinateUtils.CalculateHeadingToTarget(dxToPoint, dyToPoint);
                var headingError = CoordinateUtils.CalculateHeadingError(normalizedHeading, desiredHeading);

                // Check if we've reached the current curve point and should advance to the next
                if (distance < 2.0 && currentApproachPointIndex[plid] < approachCurvePoints[plid].Count - 1)
                {
                    currentApproachPointIndex[plid]++;

                    // Gradually increase target speed as we progress through the curve
                    var progressFactor = (double)currentApproachPointIndex[plid] / approachCurvePoints[plid].Count;
                    targetSpeeds[plid] = INITIAL_APPROACH_SPEED + progressFactor * 15; // Gradually increase speed

                    logger.Log(
                        $"PLID={plid} CURVE POINT REACHED: Moving to curve point {currentApproachPointIndex[plid]} of {approachCurvePoints[plid].Count}, Speed={targetSpeeds[plid]:F1}km/h");
                }
                else if (distance < 2.0 && currentApproachPointIndex[plid] >= approachCurvePoints[plid].Count - 1)
                {
                    // Finished the approach curve; hand over to normal waypoint following without reactivating.
                    approachFinished[plid] = true;
                    isFirstApproach[plid] = false;
                    currentApproachPointIndex[plid] = 0;
                    logger.Log($"PLID={plid} APPROACH COMPLETE: Switching to normal waypoint targeting");
                }

                logger.Log(
                    $"PLID={plid} APPROACH CURVE: Point={currentApproachPointIndex[plid]}, Pos=({pointX:F1},{pointY:F1}), Distance={distance:F1}m, Heading Error={headingError}");

                return (distance, desiredHeading, headingError);
            }

            // Normal waypoint targeting
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count == 0)
            {
                logger.LogError($"PLID={plid} CalculateTargetData called without a valid path");
                return (double.MaxValue, currentHeading, 0);
            }

            if (!targetWaypointIndices.ContainsKey(plid))
                targetWaypointIndices[plid] = 0;

            var path = aiPaths[plid];
            var loop = pathIsLoop.TryGetValue(plid, out var loopFlag) ? loopFlag : true;
            var currentIndex = ClampIndex(plid, targetWaypointIndices[plid]);
            var lookaheadCount = Math.Max(1, config.LookaheadWaypoints);

            var currentWaypoint = path[currentIndex];
            var primaryLookaheadIndex = GetAdvanceIndex(plid, currentIndex, lookaheadCount);
            var primaryLookaheadWaypoint = path[primaryLookaheadIndex];

            // Distance to current waypoint drives progress/advancement.
            var currentX = currentWaypoint.Position.X / 65536.0;
            var currentY = currentWaypoint.Position.Y / 65536.0;
            var dxCurrent = currentX - carX;
            var dyCurrent = currentY - carY;
            var distanceToCurrent = Math.Sqrt(dxCurrent * dxCurrent + dyCurrent * dyCurrent);

            // Heading aims toward the lookahead waypoint by default to give more reaction time.
            // Blend several upcoming waypoints so steering anticipates the path and smooths oscillations.
            double blendedDx = 0, blendedDy = 0, totalWeight = 0;
            for (var i = 1; i <= lookaheadCount; i++)
            {
                var idx = GetAdvanceIndex(plid, currentIndex, i);
                var wp = path[idx];
                var dx = wp.Position.X / 65536.0 - carX;
                var dy = wp.Position.Y / 65536.0 - carY;

                // Weight nearer lookahead points a little higher to keep responsiveness.
                var weight = (lookaheadCount - i + 1);
                blendedDx += dx * weight;
                blendedDy += dy * weight;
                totalWeight += weight;
            }

            if (totalWeight > 0)
            {
                blendedDx /= totalWeight;
                blendedDy /= totalWeight;
            }
            else
            {
                blendedDx = primaryLookaheadWaypoint.Position.X / 65536.0 - carX;
                blendedDy = primaryLookaheadWaypoint.Position.Y / 65536.0 - carY;
            }

            var normalizedHeadingToWaypoint = CoordinateUtils.NormalizeHeading(currentHeading);

            var desiredHeadingToCurrent = CoordinateUtils.CalculateHeadingToTarget(dxCurrent, dyCurrent);
            var headingErrorToCurrent =
                CoordinateUtils.CalculateHeadingError(normalizedHeadingToWaypoint, desiredHeadingToCurrent);

            var desiredHeadingToWaypoint = CoordinateUtils.CalculateHeadingToTarget(blendedDx, blendedDy);
            var headingErrorToWaypoint =
                CoordinateUtils.CalculateHeadingError(normalizedHeadingToWaypoint, desiredHeadingToWaypoint);

            // Drop to the current waypoint for very tight turns or when we're already close so we don't orbit
            // around the target because the lookahead point is too far away to hit.
            var headingErrorDegrees = Math.Abs(CoordinateUtils.HeadingToDegrees(Math.Abs(headingErrorToWaypoint)));
            if (headingErrorDegrees > 180) headingErrorDegrees = 360 - headingErrorDegrees;

            var closeRangeMeters = Math.Max(8.0, config.WaypointMaxThreshold * 2.0);
            var useCurrentWaypoint = distanceToCurrent < closeRangeMeters || headingErrorDegrees > 110.0;

            if (useCurrentWaypoint)
                return (distanceToCurrent, desiredHeadingToCurrent, headingErrorToCurrent);

            return (distanceToCurrent, desiredHeadingToWaypoint, headingErrorToWaypoint);
        }
    }
}
