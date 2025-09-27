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

        // Path entry handling
        private readonly Dictionary<byte, bool> isFirstApproach = new Dictionary<byte, bool>();

        // Progress tracking
        private readonly Dictionary<byte, double> lastDistanceToWaypoint = new Dictionary<byte, double>();
        private readonly Dictionary<byte, DateTime> lastProgressCheckTimes = new Dictionary<byte, DateTime>();
        private readonly Logger logger;
        private readonly Dictionary<byte, int> lookAheadWaypointIndices = new Dictionary<byte, int>();
        private readonly Dictionary<byte, int> recoveryAttempts = new Dictionary<byte, int>();
        private readonly Dictionary<byte, int> stationaryChecks = new Dictionary<byte, int>();
        private readonly SteeringCalculator steeringCalculator;
        private readonly Dictionary<byte, double> targetSpeeds = new Dictionary<byte, double>();
        private readonly Dictionary<byte, double> manualTargetSpeeds = new Dictionary<byte, double>();
        private readonly Dictionary<byte, int> targetWaypointIndices = new Dictionary<byte, int>();
        private readonly Dictionary<byte, DateTime> waypointTimers = new Dictionary<byte, DateTime>();

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
            // If path already exists, just return
            if (aiPaths.ContainsKey(plid) && aiPaths[plid] != null)
                return true;

            var currentPos = new Vec(car.X, car.Y, car.Z);

            // Get path from waypoint manager
            aiPaths[plid] = waypointManager.GetPathForCar(currentPos, config);

            if (aiPaths[plid] == null || aiPaths[plid].Count == 0)
            {
                logger.LogError($"PLID={plid} Failed to get valid path for car");
                return false;
            }

            // Find best starting waypoint based on car position and heading
            var (bestIndex, clockwise) = waypointManager.FindBestWaypointForDirection(
                currentPos, car.Heading, aiPaths[plid]);

            if (!clockwise)
            {
                aiPaths[plid].Reverse();
                bestIndex = aiPaths[plid].Count - 1 - bestIndex;
            }

            // Initialize waypoint indices
            targetWaypointIndices[plid] = bestIndex;
            lookAheadWaypointIndices[plid] = (bestIndex + 2) % aiPaths[plid].Count;

            // IMPORTANT: Default to not using approach curve
            isFirstApproach[plid] = false;

            // Initialize tracking data
            targetSpeeds[plid] = INITIAL_APPROACH_SPEED;
            waypointTimers[plid] = DateTime.Now;
            lastDistanceToWaypoint[plid] = double.MaxValue;
            recoveryAttempts[plid] = 0;
            inRecoveryMode[plid] = false;
            headingErrorHistory[plid] = new Queue<int>();
            stationaryChecks[plid] = 0;
            lastProgressCheckTimes[plid] = DateTime.Now;
            currentApproachPointIndex[plid] = 0;

            return true;
        }

        public void CheckAndUpdateApproachCurve(byte plid, CompCar car)
        {
            if (!aiPaths.ContainsKey(plid) || !targetWaypointIndices.ContainsKey(plid))
                return;

            var currentPos = new Vec(car.X, car.Y, car.Z);
            var targetWaypoint = aiPaths[plid][targetWaypointIndices[plid]];
            var waypointPos = new Vec(targetWaypoint.Position.X, targetWaypoint.Position.Y);
            var distance = CoordinateUtils.CalculateDistance(currentPos, waypointPos);

            // Only generate approach curve when we're close to the track (e.g., within 20m)
            if (distance < APPROACH_DISTANCE)
            {
                if (!isFirstApproach[plid] || !approachCurvePoints.ContainsKey(plid))
                {
                    isFirstApproach[plid] = true;
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
            if (!aiPaths.ContainsKey(plid) || aiPaths[plid] == null || aiPaths[plid].Count < 2 ||
                !targetWaypointIndices.ContainsKey(plid) || !lookAheadWaypointIndices.ContainsKey(plid))
            {
                logger.LogWarning($"PLID={plid} Cannot generate approach curve: Missing path or indices");
                return;
            }

            // Current car position
            var carPos = new Vec(car.X, car.Y, car.Z);

            // Target waypoint
            var targetIndex = targetWaypointIndices[plid];
            var targetWaypoint = aiPaths[plid][targetIndex];

            // Look-ahead waypoint (use 3 waypoints ahead for smoother approach)
            var lookAheadIndex = (targetIndex + 3) % aiPaths[plid].Count;
            lookAheadWaypointIndices[plid] = lookAheadIndex;
            var lookAheadWaypoint = aiPaths[plid][lookAheadIndex];

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

            var targetIndex = targetWaypointIndices[plid];
            if (targetIndex >= aiPaths[plid].Count)
            {
                targetIndex = 0;
                targetWaypointIndices[plid] = targetIndex;
            }

            return aiPaths[plid][targetIndex];
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
            return (targetIndex + 1) % aiPaths[plid].Count;
        }

        /// <summary>
        ///     Check progress toward waypoint and handle recovery if needed
        /// </summary>
        public void CheckWaypointProgress(byte plid, double currentDistance)
        {
            // Ensure dictionaries have the necessary keys
            if (!lastDistanceToWaypoint.ContainsKey(plid))
                lastDistanceToWaypoint[plid] = currentDistance;

            if (!lastProgressCheckTimes.ContainsKey(plid))
                lastProgressCheckTimes[plid] = DateTime.Now;

            if (!recoveryAttempts.ContainsKey(plid))
                recoveryAttempts[plid] = 0;

            // Check progress at regular intervals
            if ((DateTime.Now - lastProgressCheckTimes[plid]).TotalMilliseconds >= config.ProgressCheckIntervalMs)
            {
                var previousDistance = lastDistanceToWaypoint[plid];
                var progress = previousDistance - currentDistance;

                logger.Log(
                    $"PLID={plid} PROGRESS CHECK: Previous={previousDistance:F1}m, Current={currentDistance:F1}m, Progress={progress:F1}m");

                // If too many failed attempts, skip to next waypoint
                if (recoveryAttempts[plid] >= config.MaxRecoveryAttempts)
                {
                    AdvanceToNextWaypoint(plid, "TOO MANY RECOVERY ATTEMPTS");
                    recoveryAttempts[plid] = 0;
                }
                else if (progress < 0)
                {
                    // Moving away from waypoint, increment recovery attempt counter
                    recoveryAttempts[plid]++;
                    logger.Log(
                        $"PLID={plid} MOVING AWAY: Recovery attempt {recoveryAttempts[plid]} of {config.MaxRecoveryAttempts}");
                }
                else if (progress > config.MinRequiredProgress)
                {
                    // Good progress, reset recovery attempts
                    recoveryAttempts[plid] = 0;
                }

                // Update for next check
                lastDistanceToWaypoint[plid] = currentDistance;
                lastProgressCheckTimes[plid] = DateTime.Now;
            }
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

            if (!headingErrorHistory.ContainsKey(plid))
                headingErrorHistory[plid] = new Queue<int>();

            if (!stationaryChecks.ContainsKey(plid))
                stationaryChecks[plid] = 0;

            var currentIndex = targetWaypointIndices[plid];
            var nextIndex = (currentIndex + 1) % aiPaths[plid].Count;

            logger.Log($"PLID={plid} {reason}: Moving from {currentIndex} to {nextIndex}");

            targetWaypointIndices[plid] = nextIndex;

            if (!targetSpeeds.ContainsKey(plid))
                targetSpeeds[plid] = 30.0; // Default speed

            targetSpeeds[plid] = aiPaths[plid][nextIndex].SpeedLimit;
            waypointTimers[plid] = DateTime.Now;
            recoveryAttempts[plid] = 0;
            inRecoveryMode[plid] = false;
            headingErrorHistory[plid].Clear();
            stationaryChecks[plid] = 0;
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

            if (speedError > 1)
            {
                var factor = Math.Min(1.0, speedError / 20.0);
                throttle = config.ThrottleBase + (int)(factor * (65535 - config.ThrottleBase));
                throttle = Math.Max(config.ThrottleBase, Math.Min(65535, throttle));
                brake = 0;
            }
            else if (speedError < -1)
            {
                var factor = Math.Min(1.0, -speedError / 20.0);
                throttle = 0;
                brake = config.BrakeBase + (int)(factor * (65535 - config.BrakeBase));
                brake = Math.Max(config.BrakeBase, Math.Min(65535, brake));
            }
            else
            {
                throttle = config.ThrottleBase / 2;
                brake = 0;
            }

            return (throttle, brake);
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

        public void SetPath(byte plid, List<Util.Waypoint> path)
        {
            aiPaths[plid] = path;
            targetWaypointIndices[plid] = 0;
            lookAheadWaypointIndices[plid] = path != null && path.Count > 2 ? 2 : 0;
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

                logger.Log(
                    $"PLID={plid} APPROACH CURVE: Point={currentApproachPointIndex[plid]}, Pos=({pointX:F1},{pointY:F1}), Distance={distance:F1}m, Heading Error={headingError}");

                return (distance, desiredHeading, headingError);
            }

            // Normal waypoint targeting
            var waypoint = GetTargetWaypoint(plid);

            // Calculate distance and direction to waypoint
            var waypointX = waypoint.Position.X / 65536.0;
            var waypointY = waypoint.Position.Y / 65536.0;
            var dxToWaypoint = waypointX - carX;
            var dyToWaypoint = waypointY - carY;
            var distanceToWaypoint = Math.Sqrt(dxToWaypoint * dxToWaypoint + dyToWaypoint * dyToWaypoint);

            // Calculate desired heading to target
            var normalizedHeadingToWaypoint = CoordinateUtils.NormalizeHeading(currentHeading);
            var desiredHeadingToWaypoint = CoordinateUtils.CalculateHeadingToTarget(dxToWaypoint, dyToWaypoint);
            var headingErrorToWaypoint =
                CoordinateUtils.CalculateHeadingError(normalizedHeadingToWaypoint, desiredHeadingToWaypoint);

            return (distanceToWaypoint, desiredHeadingToWaypoint, headingErrorToWaypoint);
        }
    }
}