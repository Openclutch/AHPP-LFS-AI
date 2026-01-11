using System;
using System.Collections.Generic;
using AHPP_AI.Debug;
using AHPP_AI.Util;
using AHPP_AI.Waypoint;
using InSimDotNet;
using InSimDotNet.Packets;
using InSimClient = InSimDotNet.InSimClient;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Core driving logic for AI cars, handling steering, throttle, braking and waypoint following
    /// </summary>
    public class AIDriver
    {
        // Enumeration for engine start state machine
        public enum EngineStartState
        {
            Normal, // Normal operation, no stall detected
            StallDetected, // Engine stall detected
            PressClutch, // Step 1: Press clutch
            TurnIgnition, // Step 2: Turn ignition
            ReleaseClutch, // Step 3: Gradually release clutch
            Recovery // Recovery completed, resume normal operation
        }

        private enum RecoveryState
        {
            Driving,
            EngineRestart,
            ReverseShort,
            ReverseLong,
            Cooldown
        }

        private enum MovementIssue
        {
            None,
            SlowProgress,
            StuckLowSpeed
        }

        private struct TrafficLeadInfo
        {
            public byte TargetPlid;
            public bool TargetIsAi;
            public double GapMeters;
            public double ClosingSpeedMps;
            public double TtcSeconds;
            public double DesiredGapMeters;
        }

        private class RecoveryContext
        {
            public RecoveryState State = RecoveryState.Driving;
            public EngineStartState EngineState = EngineStartState.Normal;
            public DateTime StateStarted = DateTime.Now;
            public DateTime ActionEnds = DateTime.Now;
            public int FailureCount;
            public int AttemptCount;
            public double BaselineDistance = double.MaxValue;
            public double BaselineX;
            public double BaselineY;
            public int BaselineHeadingError;
            public int SteerDirection = 1;
            public string Reason = string.Empty;
            public bool ValidationActive;
        }

        private readonly AIConfig config;

        // Control state
        private readonly Dictionary<byte, string> controlInputs = new Dictionary<byte, string>();
        private readonly Dictionary<byte, string> controlStatuses = new Dictionary<byte, string>();

        // Engine state tracking
        private readonly Dictionary<byte, bool> engineRunning = new Dictionary<byte, bool>();
        private readonly Dictionary<byte, DateTime> warmupUntilUtc = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, DateTime> warmupHoldUntilUtc = new Dictionary<byte, DateTime>();

        private readonly Dictionary<byte, EngineStartState>
            engineStartStates = new Dictionary<byte, EngineStartState>();

        private readonly Dictionary<byte, DateTime> engineStateTimers = new Dictionary<byte, DateTime>();
        private readonly GearboxController gearboxController;
        private readonly AILightController lightController;
        private readonly InSimClient insim;
        private readonly Dictionary<byte, DateTime> lastCollisionLogTime = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, double> lastProgressDistance = new Dictionary<byte, double>();
        private readonly Dictionary<byte, DateTime> lastStuckCheckTime = new Dictionary<byte, DateTime>();
        private Action<byte>? recoveryFailedHandler;
        private Action<byte, string>? recoveryStartedHandler;
        private readonly Logger logger;
        private readonly Dictionary<byte, double> stuckPositionX = new Dictionary<byte, double>();
        private readonly Dictionary<byte, double> stuckPositionY = new Dictionary<byte, double>();
        private readonly Dictionary<byte, MovementIssue> movementIssues = new Dictionary<byte, MovementIssue>();
        private readonly Dictionary<byte, int> consecutiveStalls = new Dictionary<byte, int>();
        private readonly Dictionary<byte, RecoveryContext> recoveryContexts = new Dictionary<byte, RecoveryContext>();
        private readonly Dictionary<byte, int> movementIssueCounts = new Dictionary<byte, int>();
        private readonly WaypointFollower waypointFollower;

        public AIDriver(
            AIConfig config,
            Logger logger,
            WaypointFollower waypointFollower,
            GearboxController gearboxController,
            InSimClient insim,
            AILightController lightController)
        {
            this.config = config;
            this.logger = logger;
            this.waypointFollower = waypointFollower;
            this.gearboxController = gearboxController;
            this.insim = insim;
            this.lightController = lightController;
        }

        /// <summary>
        /// Register a callback that is invoked when recovery fails and a reset is required.
        /// </summary>
        public void SetRecoveryFailedHandler(Action<byte> handler)
        {
            recoveryFailedHandler = handler;
        }

        /// <summary>
        /// Register a callback that is invoked whenever recovery begins.
        /// </summary>
        public void SetRecoveryStartedHandler(Action<byte, string> handler)
        {
            recoveryStartedHandler = handler;
        }

        /// <summary>
        /// Start a warmup window where recovery and stall logic are suppressed.
        /// </summary>
        public void StartWarmup(byte plid, int warmupDurationMs, int brakeHoldMs)
        {
            var now = DateTime.UtcNow;
            warmupUntilUtc[plid] = now.AddMilliseconds(Math.Max(0, warmupDurationMs));
            warmupHoldUntilUtc[plid] = now.AddMilliseconds(Math.Max(0, brakeHoldMs));
        }

        /// <summary>
        /// Determine whether an AI is still within its warmup window.
        /// </summary>
        private bool IsWarmup(byte plid)
        {
            return warmupUntilUtc.TryGetValue(plid, out var until) && DateTime.UtcNow < until;
        }

        /// <summary>
        /// Determine whether the warmup brake hold is still active.
        /// </summary>
        private bool IsWarmupHoldActive(byte plid)
        {
            return warmupHoldUntilUtc.TryGetValue(plid, out var until) && DateTime.UtcNow < until;
        }

        /// <summary>
        ///     Initialize a new AI driver
        /// </summary>
        public void InitializeDriver(byte plid)
        {
            gearboxController.InitializeGearbox(plid);
            controlInputs[plid] = "T:0k B:0k G:0 C:0 S:0";
            controlStatuses[plid] = "Initializing";
            engineRunning[plid] = false; // Always start with engine "not running" to trigger start procedure
            engineStartStates[plid] = EngineStartState.PressClutch; // Start in clutch pressed state
            engineStateTimers[plid] = DateTime.Now;

            // Reset lights and set a sensible default
            lightController?.Reset(plid);
            lightController?.SetHeadlights(plid, AILightController.HeadlightMode.LowBeam);
            lightController?.CancelIndicators(plid);

            // Initialize recovery tracking
            recoveryContexts[plid] = new RecoveryContext();
            stuckPositionX[plid] = 0;
            stuckPositionY[plid] = 0;
            lastStuckCheckTime[plid] = DateTime.Now;
            lastProgressDistance[plid] = 0;
            movementIssues[plid] = MovementIssue.None;
            consecutiveStalls[plid] = 0;
            movementIssueCounts[plid] = 0;

            // Send initial controls to configure AI car
            SendAIControls(plid);
        }

        /// <summary>
        ///     Update AI controls based on telemetry data
        /// </summary>
        public void UpdateControls(
            byte plid,
            CompCar car,
            CompCar[] allCars,
            TrafficCarSnapshot[] trafficSnapshot,
            WaypointManager waypointManager,
            AIConfig config,
            List<ObjectInfo> layoutObjects,
            float currentRpm,
            bool holdForMerge)
        {
            try
            {
                lightController?.Update(plid);

                // Initialize path if needed
                if (!waypointFollower.InitializePath(plid, car, waypointManager, config, layoutObjects))
                    return;

                // Check if we should activate approach curve based on distance to track
                // TODO commented out for now, causing AI to never get onto the track
                // waypointFollower.CheckAndUpdateApproachCurve(plid, car);

                // Calculate position in meters
                var carX = car.X / 65536.0;
                var carY = car.Y / 65536.0;

                // Get current speed in km/h
                var speedKmh = 360.0 * car.Speed / 32768.0;
                double currentHeading = car.Heading;


                // Calculate distance and heading to target waypoint early for recovery steering.
                var (distance, desiredHeading, headingError) = waypointFollower.CalculateTargetData(
                    plid, carX, carY, (int)currentHeading);

                // Calculate steering early so merge holds can keep the car aligned to the lane.
                var steering = CalculateSteering(plid, carX, carY, currentHeading, speedKmh, headingError);

                if (IsWarmupHoldActive(plid))
                {
                    ApplyWarmupHold(plid, steering, speedKmh, currentRpm);
                    return;
                }

                if (holdForMerge)
                {
                    ApplyMergeHold(plid, steering, speedKmh, currentRpm);
                    return;
                }

                var progressEvaluation = waypointFollower.EvaluateProgress(plid, distance, speedKmh);
                var movementIssue = DetectMovementIssue(plid, carX, carY, (int)currentHeading, speedKmh, distance);

                if (progressEvaluation.ShouldReset)
                {
                    controlStatuses[plid] = "Recovery failed - resetting";
                    logger.LogWarning($"PLID={plid} RECOVERY FAILED: Pitting/spectating AI to clear track");
                    recoveryFailedHandler?.Invoke(plid);
                    return;
                }

                // Evaluate unified recovery state machine; returns true when recovery handled control inputs.
                if (HandleRecovery(
                        plid,
                        carX,
                        carY,
                        speedKmh,
                        headingError,
                        distance,
                        progressEvaluation,
                        movementIssue))
                {
                    return;
                }

                // Log debugging info periodically
                if (DateTime.Now.Millisecond < 50 && DateTime.Now.Second % 2 == 0)
                {
                    var waypoint = waypointFollower.GetTargetWaypoint(plid);
                    var waypointX = waypoint.Position.X / 65536.0;
                    var waypointY = waypoint.Position.Y / 65536.0;

                    logger.Log(
                        $"PLID={plid} STEERING: CurrentH={CoordinateUtils.NormalizeHeading((int)currentHeading)}, " +
                        $"DesiredH={desiredHeading}, Diff={headingError}, Steering={steering}, " +
                        $"TargetPos=({waypointX:F1},{waypointY:F1})");
                }

                // Calculate throttle and brake based on speed and heading error
                int throttle;
                int brake;

                // Reduce throttle for sharp turns, apply more throttle for straight driving
                var errorMagnitude = Math.Abs(headingError) / (double)CoordinateUtils.HALF_CIRCLE;
                if (errorMagnitude > 0.5) // More than 90 degrees off
                {
                    // We're facing more away than toward - reduce speed for sharper turn
                    throttle = 20000;
                    brake = 0;

                    // Log severe heading errors
                    if (DateTime.Now.Millisecond < 100) // Throttle logging
                        logger.Log(
                            $"PLID={plid} SHARP TURN: Heading={CoordinateUtils.NormalizeHeading((int)currentHeading)}, Target={desiredHeading}, Diff={headingError}, Steering={steering}");
                }
                else if (errorMagnitude > 0.25) // 45-90 degrees off
                {
                    throttle = 30000;
                    brake = 0;
                }
                else
                {
                    // Normal driving, use throttle based on desired speed
                    (throttle, brake) = waypointFollower.CalculateThrottleAndBrake(plid, speedKmh, headingError);
                }

                // Handle stationary vehicle 
                if (speedKmh < 0.5)
                {
                    // Apply full throttle to escape if stuck
                    throttle = 65000;

                    // Log when stuck
                    if (DateTime.Now.Millisecond < 100) // Throttle logging
                        logger.Log($"PLID={plid} SLOW/STUCK: Applying full throttle");
                }

                // Log status periodically
                if (DateTime.Now.Second % 5 == 0 && DateTime.Now.Millisecond < 100)
                {
                    var waypoint = waypointFollower.GetTargetWaypoint(plid);
                    var waypointX = waypoint.Position.X / 65536.0;
                    var waypointY = waypoint.Position.Y / 65536.0;
                    var (targetIndex, waypointCount, targetSpeed, inRecovery) = waypointFollower.GetFollowerInfo(plid);

                    logger.Log(
                        $"PLID={plid} AI STATUS: Pos=({carX:F1},{carY:F1}), TargetWaypoint=({waypointX:F1},{waypointY:F1}), " +
                        $"Distance={distance:F1}m, Speed={speedKmh:F1}km/h, Recovery={inRecovery}, Engine={engineRunning[plid]}");
                }

                // Check if we've reached the waypoint
                waypointFollower.CheckWaypointReached(plid, distance, speedKmh);

                // Update gearbox (gears and clutch)
                gearboxController.UpdateGearbox(plid, speedKmh, currentRpm);

                var lowRpmClutchActive = gearboxController.IsLowRpmClutchActive(plid);

                // Check if we should apply throttle based on clutch state
                if (!gearboxController.ShouldApplyThrottle(plid, speedKmh)) throttle = 0;

                var forceClutch = false;
                if (ApplyTrafficSpacing(plid, carX, carY, speedKmh, car.Direction, trafficSnapshot,
                        ref throttle, ref brake, out var trafficStatus, ref forceClutch))
                {
                    controlStatuses[plid] = trafficStatus;
                }

                // Check for potential collisions
                var collisionDanger = DetectPotentialCollision(plid, car, allCars);
                if (collisionDanger)
                {
                    // Override controls - stop the car
                    throttle = 0;
                    brake = 65535; // Full brake
                    forceClutch = true;
                    controlStatuses[plid] = "COLLISION AVOIDANCE: Stopping";
                }

                gearboxController.ApplyBrakingClutch(plid, brake);

                // Get current gearbox state after braking adjustments
                var (gear, clutchValue, clutchState) = gearboxController.GetGearboxInfo(plid);
                if (forceClutch) clutchValue = 65535;

                // Update control info for debug display
                controlInputs[plid] =
                    $"T:{throttle / 1000}k B:{brake / 1000}k G:{gear} C:{clutchValue / 1000}k S:{(steering - config.SteeringCenter)}";
                if (lowRpmClutchActive) controlStatuses[plid] = "Low RPM clutch";

                // Send AI controls
                SendControlInputs(plid, steering, throttle, brake, gear, clutchValue);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Error processing AI control for PLID={plid}");
            }
        }

        /// <summary>
        /// Apply full braking to hold position while waiting for a safe merge.
        /// </summary>
        private void ApplyMergeHold(byte plid, int steering, double speedKmh, float currentRpm)
        {
            const int throttle = 0;
            const int brake = 65535;

            gearboxController.UpdateGearbox(plid, speedKmh, currentRpm);
            gearboxController.ApplyBrakingClutch(plid, brake);

            var (gear, clutchValue, _) = gearboxController.GetGearboxInfo(plid);
            clutchValue = Math.Max(clutchValue, config.ClutchFullyPressed);

            controlInputs[plid] =
                $"T:{throttle / 1000}k B:{brake / 1000}k G:{gear} C:{clutchValue / 1000}k S:{(steering - config.SteeringCenter)}";
            controlStatuses[plid] = "MERGE YIELD: Waiting";

            SendControlInputs(plid, steering, throttle, brake, gear, clutchValue);
        }

        /// <summary>
        /// Apply a short brake hold during spawn warmup to stabilize the AI.
        /// </summary>
        private void ApplyWarmupHold(byte plid, int steering, double speedKmh, float currentRpm)
        {
            const int throttle = 0;
            const int brake = 65535;

            gearboxController.UpdateGearbox(plid, speedKmh, currentRpm);
            gearboxController.ApplyBrakingClutch(plid, brake);

            var (gear, clutchValue, _) = gearboxController.GetGearboxInfo(plid);
            clutchValue = Math.Max(clutchValue, config.ClutchFullyPressed);

            controlInputs[plid] =
                $"T:{throttle / 1000}k B:{brake / 1000}k G:{gear} C:{clutchValue / 1000}k S:{(steering - config.SteeringCenter)}";
            controlStatuses[plid] = "WARMUP HOLD: Stabilizing";

            SendControlInputs(plid, steering, throttle, brake, gear, clutchValue);
        }

        /// <summary>
        /// Adjust throttle and braking to maintain safe spacing on the current lane.
        /// </summary>
        private bool ApplyTrafficSpacing(
            byte plid,
            double carX,
            double carY,
            double speedKmh,
            int carDirection,
            TrafficCarSnapshot[] trafficSnapshot,
            ref int throttle,
            ref int brake,
            out string status,
            ref bool forceClutch)
        {
            status = string.Empty;
            if (trafficSnapshot == null || trafficSnapshot.Length == 0) return false;

            var path = waypointFollower.GetPath(plid);
            if (path == null || path.Count < 2) return false;

            var speedMps = speedKmh / 3.6;
            if (!TryGetLeadInfo(plid, carX, carY, speedMps, carDirection, trafficSnapshot, path, out var leadInfo))
                return false;

            var spacingFactor = GetSpacingFactor(leadInfo.TargetIsAi);
            var emergencyTtc = Math.Max(0.1, config.TrafficEmergencyTtcSeconds * spacingFactor);
            var brakeTtc = Math.Max(0.1, config.TrafficBrakeTtcSeconds * spacingFactor);

            if (leadInfo.GapMeters <= config.MinimumSafetyDistanceM ||
                (leadInfo.ClosingSpeedMps > 0 && leadInfo.TtcSeconds <= emergencyTtc))
            {
                throttle = 0;
                brake = 65535;
                forceClutch = true;
                status = $"TRAFFIC STOP: PLID {leadInfo.TargetPlid}";
                LogTrafficWarning(plid,
                    $"TRAFFIC EMERGENCY: lead {leadInfo.TargetPlid} gap={leadInfo.GapMeters:F1}m ttc={leadInfo.TtcSeconds:F1}s");
                return true;
            }

            var updated = false;
            if (leadInfo.ClosingSpeedMps > 0 && leadInfo.TtcSeconds <= brakeTtc)
            {
                var intensity = (brakeTtc - leadInfo.TtcSeconds) / brakeTtc;
                intensity = Math.Max(0.0, Math.Min(1.0, intensity));
                var targetBrake = config.BrakeBase + (int)Math.Round(intensity * (65535 - config.BrakeBase));
                brake = Math.Max(brake, targetBrake);
                throttle = 0;
                status = $"TRAFFIC BRAKE: PLID {leadInfo.TargetPlid}";
                LogTrafficWarning(plid,
                    $"TRAFFIC BRAKE: lead {leadInfo.TargetPlid} gap={leadInfo.GapMeters:F1}m ttc={leadInfo.TtcSeconds:F1}s");
                updated = true;
            }
            else if (leadInfo.GapMeters < leadInfo.DesiredGapMeters)
            {
                var ratio = leadInfo.GapMeters / Math.Max(0.1, leadInfo.DesiredGapMeters);
                ratio = Math.Max(0.0, Math.Min(1.0, ratio));
                var throttleLimit = (int)Math.Round(throttle * ratio);
                if (throttleLimit < throttle)
                {
                    throttle = throttleLimit;
                    status = $"TRAFFIC FOLLOW: PLID {leadInfo.TargetPlid}";
                    updated = true;
                }
            }

            return updated;
        }

        /// <summary>
        /// Find the nearest lead car on the current path and compute spacing metrics.
        /// </summary>
        private bool TryGetLeadInfo(
            byte plid,
            double carX,
            double carY,
            double speedMps,
            int carDirection,
            TrafficCarSnapshot[] trafficSnapshot,
            List<Util.Waypoint> path,
            out TrafficLeadInfo leadInfo)
        {
            leadInfo = default;

            var geometry = PathProjection.GetGeometry(path, config.PathLoopClosureDistanceMeters);
            if (geometry == null) return false;

            if (!PathProjection.TryProjectToPath(path, geometry, carX, carY, out var selfProjection))
                return false;

            var selfDir = GetDirectionVector(carDirection);
            var selfForwardSpeed =
                speedMps * (selfDir.x * selfProjection.DirectionX + selfDir.y * selfProjection.DirectionY);
            if (selfForwardSpeed < 0) selfForwardSpeed = 0;

            var lookahead = Math.Max(config.TrafficLookaheadMinMeters, speedMps * config.TrafficLookaheadSeconds);
            var laneHalfWidth = Math.Max(0.1, config.TrafficLaneHalfWidthMeters);

            var bestGap = double.MaxValue;

            foreach (var snapshot in trafficSnapshot)
            {
                if (snapshot.PLID == 0 || snapshot.PLID == plid) continue;

                if (!PathProjection.TryProjectToPath(path, geometry, snapshot.XMeters, snapshot.YMeters,
                        out var otherProjection))
                    continue;

                if (Math.Abs(otherProjection.LateralOffsetMeters) > laneHalfWidth) continue;

                var forwardDistance = PathProjection.GetForwardDistance(
                    geometry,
                    selfProjection.DistanceAlongPathMeters,
                    otherProjection.DistanceAlongPathMeters);

                if (!geometry.IsLoop && forwardDistance < 0) continue;
                if (forwardDistance <= 0.1 || forwardDistance > lookahead) continue;

                if (forwardDistance < bestGap)
                {
                    var otherDir = GetDirectionVector(snapshot.Direction);
                    var otherForwardSpeed =
                        snapshot.SpeedMps *
                        (otherDir.x * otherProjection.DirectionX + otherDir.y * otherProjection.DirectionY);
                    if (otherForwardSpeed < 0) otherForwardSpeed = 0;

                    var closingSpeed = selfForwardSpeed - otherForwardSpeed;
                    var ttc = closingSpeed > 0.1 ? forwardDistance / closingSpeed : double.PositiveInfinity;
                    var spacingFactor = GetSpacingFactor(snapshot.IsAi);
                    var desiredGap =
                        (config.TrafficBaseGapMeters + speedMps * config.TrafficTimeHeadwaySeconds) * spacingFactor;

                    bestGap = forwardDistance;
                    leadInfo = new TrafficLeadInfo
                    {
                        TargetPlid = snapshot.PLID,
                        TargetIsAi = snapshot.IsAi,
                        GapMeters = forwardDistance,
                        ClosingSpeedMps = closingSpeed,
                        TtcSeconds = ttc,
                        DesiredGapMeters = desiredGap
                    };
                }
            }

            return bestGap < double.MaxValue;
        }

        /// <summary>
        /// Resolve a spacing multiplier based on whether the target is AI or human.
        /// </summary>
        private double GetSpacingFactor(bool targetIsAi)
        {
            return targetIsAi ? config.TrafficAiSpacingFactor : config.TrafficHumanSpacingFactor;
        }

        /// <summary>
        /// Convert an LFS heading to a normalized 2D direction vector.
        /// </summary>
        private (double x, double y) GetDirectionVector(int heading)
        {
            var radians = (heading + CoordinateUtils.QUARTER_CIRCLE) * 2 * Math.PI / CoordinateUtils.FULL_CIRCLE;
            return (Math.Cos(radians), Math.Sin(radians));
        }

        /// <summary>
        /// Emit throttled traffic warnings to the log.
        /// </summary>
        private void LogTrafficWarning(byte plid, string message)
        {
            if (!lastCollisionLogTime.ContainsKey(plid) ||
                (DateTime.Now - lastCollisionLogTime[plid]).TotalSeconds >= 2)
            {
                logger.Log($"PLID={plid} {message}");
                lastCollisionLogTime[plid] = DateTime.Now;
            }
        }

        /// <summary>
        ///     Inspect recent movement to see if we are likely stuck or making no progress.
        /// </summary>
        private MovementIssue DetectMovementIssue(
            byte plid,
            double carX,
            double carY,
            int currentHeading,
            double speedKmh,
            double currentDistance)
        {
            if (!movementIssues.ContainsKey(plid))
                movementIssues[plid] = MovementIssue.None;

            if (!movementIssueCounts.ContainsKey(plid))
                movementIssueCounts[plid] = 0;

            if (!lastStuckCheckTime.ContainsKey(plid))
                lastStuckCheckTime[plid] = DateTime.Now;

            // Only check at configured intervals
            if ((DateTime.Now - lastStuckCheckTime[plid]).TotalMilliseconds <
                config.RecoveryStuckCheckIntervalMs)
                return movementIssues[plid];

            lastStuckCheckTime[plid] = DateTime.Now;

            // Initialize baseline if needed
            stuckPositionX.TryGetValue(plid, out var previousX);
            stuckPositionY.TryGetValue(plid, out var previousY);

            if (Math.Abs(previousX) < double.Epsilon &&
                Math.Abs(previousY) < double.Epsilon)
            {
                stuckPositionX[plid] = carX;
                stuckPositionY[plid] = carY;
                lastProgressDistance[plid] = currentDistance;
                movementIssues[plid] = MovementIssue.None;
                return MovementIssue.None;
            }

            var dx = carX - stuckPositionX[plid];
            var dy = carY - stuckPositionY[plid];
            var distanceMoved = Math.Sqrt(dx * dx + dy * dy);
            var distanceProgress = lastProgressDistance[plid] - currentDistance;

            var issue = MovementIssue.None;

            if (distanceMoved < config.RecoveryPositionChangeThreshold &&
                speedKmh < config.RecoveryLowSpeedThresholdKmh)
            {
                issue = MovementIssue.StuckLowSpeed;
                logger.Log(
                    $"PLID={plid} STUCK DETECTION: Moved {distanceMoved:F2}m @ {speedKmh:F1}km/h (threshold {config.RecoveryPositionChangeThreshold:F1}m)");
            }
            else if (distanceProgress <= config.RecoveryProgressStallThreshold &&
                     speedKmh < config.RecoveryLowSpeedThresholdKmh * 2)
            {
                issue = MovementIssue.SlowProgress;
                logger.Log(
                    $"PLID={plid} PROGRESS STALL: Distance to target not improving (Δ={distanceProgress:F2}m)");
            }

            if (issue != MovementIssue.None)
            {
                movementIssues[plid] = issue;
                movementIssueCounts[plid] = movementIssueCounts.TryGetValue(plid, out var count) ? count + 1 : 1;
                var steerDirection = waypointFollower.CalculateTargetData(plid, carX, carY, currentHeading).headingError;
                logger.Log(
                    $"PLID={plid} MOVEMENT ISSUE: {issue} (count {movementIssueCounts[plid]} of {config.RecoveryDetectionsBeforeAction}), steerDir={Math.Sign(steerDirection)}");
                if (movementIssueCounts[plid] >= config.RecoveryDetectionsBeforeAction)
                {
                    stuckPositionX[plid] = carX;
                    stuckPositionY[plid] = carY;
                    lastProgressDistance[plid] = currentDistance;
                    return issue;
                }
            }
            else
            {
                movementIssueCounts[plid] = 0;
                movementIssues[plid] = MovementIssue.None;
            }

            stuckPositionX[plid] = carX;
            stuckPositionY[plid] = carY;
            lastProgressDistance[plid] = currentDistance;

            return MovementIssue.None;
        }


        /// <summary>
        ///     Main unified recovery handler. Returns true when recovery controls were sent and normal driving should pause.
        /// </summary>
        private bool HandleRecovery(
            byte plid,
            double carX,
            double carY,
            double speedKmh,
            int headingError,
            double distanceToWaypoint,
            WaypointFollower.ProgressEvaluationResult progressEvaluation,
            MovementIssue movementIssue)
        {
            var context = GetRecoveryContext(plid);
            if (IsWarmup(plid))
            {
                if (context.State != RecoveryState.Driving)
                {
                    context.State = RecoveryState.Driving;
                    context.ValidationActive = false;
                }

                return false;
            }

            var isEngineRunning = engineRunning.TryGetValue(plid, out var running) && running;
            if (!isEngineRunning &&
                context.State == RecoveryState.Driving)
            {
                BeginEngineRecovery(plid, headingError, distanceToWaypoint, carX, carY, "Engine stalled");
            }

            if (context.State == RecoveryState.Driving)
            {
                if (progressEvaluation.ShouldReverse)
                {
                    BeginReverseRecovery(
                        plid,
                        headingError,
                        distanceToWaypoint,
                        carX,
                        carY,
                        context.AttemptCount > 0,
                        progressEvaluation.IsMovingAway ? "Moving away from waypoint" : "Progress stalled");
                }
                else if (movementIssue != MovementIssue.None)
                {
                    BeginReverseRecovery(
                        plid,
                        headingError,
                        distanceToWaypoint,
                        carX,
                        carY,
                        context.AttemptCount > 0,
                        $"Movement issue: {movementIssue}");
                }
            }

            switch (context.State)
            {
                case RecoveryState.EngineRestart:
                    return ExecuteEngineRestart(plid, headingError, distanceToWaypoint);
                case RecoveryState.ReverseShort:
                case RecoveryState.ReverseLong:
                    return ExecuteReverseRecovery(plid);
                case RecoveryState.Cooldown:
                    EvaluateRecoveryCooldown(plid, distanceToWaypoint, speedKmh, headingError);
                    return false;
                default:
                    return false;
            }
        }

        /// <summary>
        ///     Ensure a recovery context exists for the given PLID.
        /// </summary>
        private RecoveryContext GetRecoveryContext(byte plid)
        {
            if (!recoveryContexts.TryGetValue(plid, out var context))
            {
                context = new RecoveryContext();
                recoveryContexts[plid] = context;
            }

            return context;
        }

        /// <summary>
        ///     Start a reverse recovery maneuver.
        /// </summary>
        private void BeginReverseRecovery(
            byte plid,
            int headingError,
            double distanceToWaypoint,
            double carX,
            double carY,
            bool longRecovery,
            string reason)
        {
            var context = GetRecoveryContext(plid);
            context.State = longRecovery ? RecoveryState.ReverseLong : RecoveryState.ReverseShort;
            context.StateStarted = DateTime.Now;
            context.ActionEnds = DateTime.Now.AddMilliseconds(
                longRecovery ? config.RecoveryLongReverseMs : config.RecoveryShortReverseMs);
            context.BaselineDistance = distanceToWaypoint;
            context.BaselineX = carX;
            context.BaselineY = carY;
            context.BaselineHeadingError = headingError;
            context.SteerDirection = Math.Sign(headingError);
            if (context.SteerDirection == 0) context.SteerDirection = 1;
            context.Reason = reason;
            context.ValidationActive = false;

            controlStatuses[plid] = $"Recovery: {(longRecovery ? "ReverseLong" : "ReverseShort")}";
            logger.Log(
                $"PLID={plid} RECOVERY START [{context.State}]: {reason}, steerDir={context.SteerDirection}, duration={(longRecovery ? config.RecoveryLongReverseMs : config.RecoveryShortReverseMs)}ms");
            recoveryStartedHandler?.Invoke(plid, reason);
        }

        /// <summary>
        ///     Execute the reverse maneuver until the configured duration completes.
        /// </summary>
        private bool ExecuteReverseRecovery(byte plid)
        {
            var context = GetRecoveryContext(plid);

            if (DateTime.Now <= context.ActionEnds)
            {
                var steerMagnitude = context.State == RecoveryState.ReverseLong ? 18000 : 12000;
                var throttle = context.State == RecoveryState.ReverseLong ? 28000 : 20000;

                var steer = config.SteeringCenter + context.SteerDirection * steerMagnitude;
                SendControlInputs(plid, steer, throttle, 4000, 0, config.ClutchReleased);
                controlStatuses[plid] = $"Recovery: {context.State}";
                return true;
            }

            EnterRecoveryCooldown(plid, "Reverse maneuver complete");
            return false;
        }

        /// <summary>
        ///     Initialize the engine restart flow and baseline for validation.
        /// </summary>
        private void BeginEngineRecovery(
            byte plid,
            int headingError,
            double distanceToWaypoint,
            double carX,
            double carY,
            string reason)
        {
            var context = GetRecoveryContext(plid);
            context.State = RecoveryState.EngineRestart;
            context.StateStarted = DateTime.Now;
            context.BaselineDistance = distanceToWaypoint;
            context.BaselineX = carX;
            context.BaselineY = carY;
            context.BaselineHeadingError = headingError;
            context.Reason = reason;
            context.ValidationActive = false;

            engineStartStates[plid] = EngineStartState.StallDetected;
            engineStateTimers[plid] = DateTime.Now;

            controlStatuses[plid] = "Engine restart";
            logger.Log($"PLID={plid} RECOVERY START [EngineRestart]: {reason}");
            recoveryStartedHandler?.Invoke(plid, reason);
        }

        /// <summary>
        ///     Drive the engine restart state machine. Returns true while sending recovery controls.
        /// </summary>
        private bool ExecuteEngineRestart(byte plid, int headingError, double distanceToWaypoint)
        {
            var context = GetRecoveryContext(plid);

            var isRunning = engineRunning.TryGetValue(plid, out var running) && running;
            if (isRunning && engineStartStates[plid] == EngineStartState.Normal)
            {
                    EnterRecoveryCooldown(plid, "Engine restarted", true, true);
                    return false;
                }

            var steering = CalculateHeadingBasedSteering(headingError);
            var throttleBoost = CalculateStallRecoveryThrottleBoost(steering);
            var elapsed = DateTime.Now.Subtract(engineStateTimers[plid]).TotalMilliseconds;

            switch (engineStartStates[plid])
            {
                case EngineStartState.StallDetected:
                    if (consecutiveStalls.TryGetValue(plid, out var stalls) &&
                        stalls >= config.RecoveryStallReverseTrigger)
                    {
                        consecutiveStalls[plid] = 0;
                        BeginReverseRecovery(
                            plid,
                            headingError,
                            distanceToWaypoint,
                            context.BaselineX,
                            context.BaselineY,
                            true,
                            "Repeated stalls - reversing");
                        return true;
                    }

                    engineStartStates[plid] = EngineStartState.PressClutch;
                    engineStateTimers[plid] = DateTime.Now;
                    logger.Log($"PLID={plid} STALL: Pressing clutch");
                    return true;

                case EngineStartState.PressClutch:
                    SendControlInputs(plid, steering, 3000 + throttleBoost, 0, 2, config.ClutchFullyPressed);

                    if (elapsed >= 500)
                    {
                        engineStartStates[plid] = EngineStartState.TurnIgnition;
                        engineStateTimers[plid] = DateTime.Now;
                        logger.Log($"PLID={plid} STALL: Turning ignition");
                    }

                    return true;

                case EngineStartState.TurnIgnition:
                    SendControlInputs(plid, steering, 15000 + throttleBoost, 0, 2, config.ClutchFullyPressed);

                    if (elapsed < 300)
                    {
                        insim.Send(new IS_AIC(new List<AIInputVal>
                                { new AIInputVal { Input = AicInputType.CS_IGNITION, Time = 0, Value = 2 } })
                            { PLID = plid }, InSimClientExtensions.PacketPriority.High);
                    }
                    else if (elapsed >= 600)
                    {
                        insim.Send(new IS_AIC(new List<AIInputVal>
                                { new AIInputVal { Input = AicInputType.CS_IGNITION, Time = 0, Value = 3 } })
                            { PLID = plid }, InSimClientExtensions.PacketPriority.High);

                        if (elapsed >= 2000)
                        {
                            engineStartStates[plid] = EngineStartState.ReleaseClutch;
                            engineStateTimers[plid] = DateTime.Now;
                            logger.Log($"PLID={plid} STALL: Preparing for clutch release with throttle");
                        }
                    }

                    return true;

                case EngineStartState.ReleaseClutch:
                    var totalReleaseTime =
                        config.ClutchReleaseSteps * config.ClutchReleaseIntervalMs * 5;
                    var releaseProgress = Math.Min(1.0, elapsed / totalReleaseTime);
                    var progressCurve = Math.Pow(releaseProgress, 3) * (3 - 2 * Math.Pow(releaseProgress, 1.5));

                    var clutchValue = (int)(config.ClutchFullyPressed * (1 - progressCurve));
                    var throttle = Math.Min(30000, 20000 + throttleBoost);
                    byte gear = 2;

                    SendControlInputs(plid, steering, throttle, 0, gear, clutchValue);

                    if ((int)(elapsed / 1000) != (int)((elapsed - 20) / 1000))
                        logger.Log(
                            $"PLID={plid} CLUTCH: Release={progressCurve:F2}, Value={clutchValue}, Throttle={throttle}");

                    if (progressCurve >= 1.0)
                    {
                        engineStartStates[plid] = EngineStartState.Recovery;
                        engineStateTimers[plid] = DateTime.Now;
                        logger.Log($"PLID={plid} STALL: Clutch release complete");
                    }

                    return true;

                case EngineStartState.Recovery:
                    if (elapsed >= 100)
                    {
                        engineStartStates[plid] = EngineStartState.Normal;
                        EnterRecoveryCooldown(plid, "Engine restart sequence complete", true, true);
                        return false;
                    }

                    SendControlInputs(plid, steering, 15000 + throttleBoost, 0, 2, 0);
                    return true;
                default:
                    engineStartStates[plid] = EngineStartState.Normal;
                    EnterRecoveryCooldown(plid, "Engine restart reset", true, true);
                    return false;
            }
        }

        /// <summary>
        ///     Enter cooldown/validation after a recovery attempt.
        /// </summary>
        private void EnterRecoveryCooldown(byte plid, string reason, bool resetToFirstGear = false,
            bool markEngineRunning = false)
        {
            var context = GetRecoveryContext(plid);
            context.State = RecoveryState.Cooldown;
            context.StateStarted = DateTime.Now;
            context.ActionEnds = DateTime.Now.AddMilliseconds(config.RecoveryCooldownMs);
            context.ValidationActive = true;
            controlStatuses[plid] = $"Recovery cool: {reason}";
            if (resetToFirstGear)
                gearboxController.ResetToFirstGear(plid, reason);
            if (markEngineRunning)
                UpdateEngineState(plid, true);
            logger.Log($"PLID={plid} RECOVERY COOLING: {reason}");
        }

        /// <summary>
        ///     Determine if the prior recovery produced enough improvement to reset failure counters.
        /// </summary>
        private void EvaluateRecoveryCooldown(byte plid, double currentDistance, double speedKmh, int headingError)
        {
            var context = GetRecoveryContext(plid);
            if (!context.ValidationActive)
                return;

            var distanceImproved = context.BaselineDistance - currentDistance >= config.RecoverySuccessDistanceMeters;
            var speedRecovered = speedKmh >= config.RecoverySuccessSpeedKmh;
            var headingImproved = Math.Abs(context.BaselineHeadingError) - Math.Abs(headingError);

            if (distanceImproved || speedRecovered || headingImproved > 2000)
            {
                MarkRecoverySuccess(plid,
                    $"distance Δ={(context.BaselineDistance - currentDistance):F1}m, speed={speedKmh:F1}");
                return;
            }

            var elapsed = DateTime.Now - context.StateStarted;
            if (elapsed.TotalMilliseconds >= config.RecoveryValidationWindowMs)
                MarkRecoveryFailure(plid, "No improvement after recovery window");
        }

        /// <summary>
        ///     Reset to normal driving after a successful recovery.
        /// </summary>
        private void MarkRecoverySuccess(byte plid, string reason)
        {
            var context = GetRecoveryContext(plid);
            context.State = RecoveryState.Driving;
            context.ValidationActive = false;
            context.AttemptCount = 0;
            context.FailureCount = 0;
            movementIssueCounts[plid] = 0;
            movementIssues[plid] = MovementIssue.None;
            controlStatuses[plid] = $"Recovery success: {reason}";
            logger.Log($"PLID={plid} RECOVERY SUCCESS: {reason}");
        }

        /// <summary>
        ///     Track a failed recovery attempt and escalate if needed.
        /// </summary>
        private void MarkRecoveryFailure(byte plid, string reason)
        {
            var context = GetRecoveryContext(plid);
            context.ValidationActive = false;
            context.State = RecoveryState.Driving;
            context.FailureCount++;
            context.AttemptCount++;
            controlStatuses[plid] = $"Recovery failed ({context.FailureCount})";

            logger.LogWarning(
                $"PLID={plid} RECOVERY FAILURE {context.FailureCount}/{config.RecoveryMaxFailureCount}: {reason}");

            if (context.FailureCount >= config.RecoveryMaxFailureCount)
            {
                logger.LogWarning($"PLID={plid} RECOVERY FAILURE: Escalating to pitlane command");
                recoveryFailedHandler?.Invoke(plid);
            }
        }

        /// <summary>
        ///     Update engine running state and trigger recovery on stall.
        /// </summary>
        public void UpdateEngineState(byte plid, bool isRunning)
        {
            if (!engineRunning.ContainsKey(plid))
                engineRunning[plid] = isRunning;

            if (engineRunning[plid] == isRunning)
                return;

            if (IsWarmup(plid) && !isRunning)
            {
                engineRunning[plid] = false;
                return;
            }

            logger.Log($"PLID={plid} ENGINE: {(isRunning ? "STARTED" : "STALLED")}");

            if (!isRunning)
            {
                engineStartStates[plid] = EngineStartState.StallDetected;
                engineStateTimers[plid] = DateTime.Now;
                consecutiveStalls[plid] = consecutiveStalls.TryGetValue(plid, out var stalls) ? stalls + 1 : 1;
                BeginEngineRecovery(plid, 0, 0, 0, 0, "Engine stalled");
            }
            else
            {
                consecutiveStalls[plid] = 0;
                engineStartStates[plid] = EngineStartState.Normal;
            }

            engineRunning[plid] = isRunning;
        }

        /// <summary>
        ///     Apply extra throttle during stall recovery when steering away from center.
        /// </summary>
        private int CalculateStallRecoveryThrottleBoost(int steering)
        {
            var steeringOffset = Math.Abs(steering - config.SteeringCenter);
            if (steeringOffset < 1000)
                return 0;

            return Math.Min(8000, steeringOffset / 4);
        }

        /// <summary>
        ///     Provide the latest control/debug string for the specified AI.
        /// </summary>
        public string GetControlInfo(byte plid)
        {
            return controlInputs.ContainsKey(plid) ? controlInputs[plid] : "Not initialized";
        }

        /// <summary>
        ///     Translate any tracked movement issues into a short label for debug display.
        /// </summary>
        private string GetMovementIssueLabel(byte plid)
        {
            if (!movementIssues.TryGetValue(plid, out var issue))
                return string.Empty;

            return issue switch
            {
                MovementIssue.SlowProgress => "Slow progress",
                MovementIssue.StuckLowSpeed => "Stuck (low speed)",
                _ => string.Empty
            };
        }

        /// <summary>
        /// Provide a compact description of the AI's current driving state for UI display.
        /// </summary>
        public string GetStateDescription(byte plid)
        {
            var recoveryContext = GetRecoveryContext(plid);
            if (recoveryContext.State == RecoveryState.EngineRestart)
                return "Engine Restart";
            if (recoveryContext.State == RecoveryState.ReverseShort || recoveryContext.State == RecoveryState.ReverseLong)
                return "Recovery";
            if (recoveryContext.State == RecoveryState.Cooldown)
                return "Recovery Check";

            if (engineStartStates.TryGetValue(plid, out var engineState) &&
                engineState != EngineStartState.Normal)
            {
                return engineState switch
                {
                    EngineStartState.StallDetected => "Engine: Stall",
                    EngineStartState.PressClutch => "Engine: Clutch",
                    EngineStartState.TurnIgnition => "Engine: Ignition",
                    EngineStartState.ReleaseClutch => "Engine: Release",
                    EngineStartState.Recovery => "Engine: Recovery",
                    _ => "Engine: Restart"
                };
            }

            if (controlStatuses.TryGetValue(plid, out var info))
            {
                if (info.StartsWith("COLLISION", StringComparison.OrdinalIgnoreCase))
                    return "Collision Stop";

                if (info.IndexOf("Recovery", StringComparison.OrdinalIgnoreCase) >= 0)
                    return "Recovery";
            }

            var movementLabel = GetMovementIssueLabel(plid);
            if (!string.IsNullOrEmpty(movementLabel))
                return movementLabel;

            var progressLabel = waypointFollower.GetProgressStateLabel(plid);
            if (!string.IsNullOrEmpty(progressLabel))
                return progressLabel;

            return "Driving";
        }

        /// <summary>
        ///     Calculate steering using pure pursuit when enabled, falling back to heading error control.
        /// </summary>
        private int CalculateSteering(
            byte plid,
            double carX,
            double carY,
            double currentHeading,
            double speedKmh,
            int headingError)
        {
            if (config.UsePurePursuitSteering)
            {
                var (targetX, targetY, lookaheadDistance) =
                    waypointFollower.CalculatePurePursuitTarget(plid, carX, carY, speedKmh);

                if (lookaheadDistance > 0.01)
                {
                    var desiredHeading =
                        CoordinateUtils.CalculateHeadingToTarget(targetX - carX, targetY - carY);
                    var pursuitError = CoordinateUtils.CalculateHeadingError(
                        CoordinateUtils.NormalizeHeading((int)currentHeading), desiredHeading);

                    var purePursuitSteering = CalculatePurePursuitSteeringValue(pursuitError, lookaheadDistance);
                    if (purePursuitSteering.HasValue)
                        return purePursuitSteering.Value;
                }
            }

            return CalculateHeadingBasedSteering(headingError);
        }

        /// <summary>
        ///     Convert pure pursuit curvature into a steering command for LFS.
        /// </summary>
        private int? CalculatePurePursuitSteeringValue(int headingError, double lookaheadDistance)
        {
            if (lookaheadDistance <= 0.01)
                return null;

            var deadzoneDeg = Math.Max(0.0, config.SteeringDeadzoneDegrees);
            var headingErrorDegrees = Math.Abs(headingError) * 360.0 / CoordinateUtils.FULL_CIRCLE;
            if (headingErrorDegrees < deadzoneDeg) return config.SteeringCenter;

            var alpha = headingError * (2 * Math.PI / CoordinateUtils.FULL_CIRCLE);
            var curvature = (2 * Math.Sin(alpha)) / Math.Max(0.1, lookaheadDistance);
            curvature *= Math.Max(0.01, config.PurePursuitSteeringGain);

            var wheelAngle = Math.Atan(curvature * Math.Max(0.5, config.PurePursuitWheelbaseMeters));
            var maxSteerRadians = Math.Max(0.017, config.PurePursuitMaxSteerDegrees * Math.PI / 180.0);
            var normalized = wheelAngle / maxSteerRadians;

            normalized *= Math.Max(0.1, config.SteeringResponseDamping);
            var clamped = Math.Max(-1.0, Math.Min(1.0, normalized));

            var steeringRange = config.SteeringCenter - 1;
            var steeringValue = config.SteeringCenter - (int)(clamped * steeringRange);
            steeringValue = Math.Max(config.MinSteering, Math.Min(config.MaxSteering, steeringValue));

            return steeringValue;
        }

        /// <summary>
        ///     Fallback steering controller based purely on heading error.
        /// </summary>
        private int CalculateHeadingBasedSteering(int headingError)
        {
            var steeringCenter = config.SteeringCenter;

            var damping = Math.Max(0.1, config.SteeringResponseDamping);
            var deadzoneDeg = Math.Max(0.0, config.SteeringDeadzoneDegrees);
            var deadzoneUnits = deadzoneDeg * 65536.0 / 360.0;

            var adjustedError = Math.Max(0.0, Math.Abs(headingError) - deadzoneUnits);
            if (adjustedError <= 0.0) return steeringCenter;

            var response = Math.Min(1.0, adjustedError / 16384.0 * damping);

            var steeringRange = steeringCenter - 100;

            var steering = steeringCenter - (int)(Math.Sign(headingError) * response * steeringRange);
            return Math.Max(config.MinSteering, Math.Min(config.MaxSteering, steering));
        }

        /// <summary>
        ///     Detects if there is a car in front that may cause a collision
        /// </summary>
        private bool DetectPotentialCollision(byte plid, CompCar car, CompCar[] allCars)
        {
            // Calculate position in meters
            var carX = car.X / 65536.0;
            var carY = car.Y / 65536.0;
            var carHeading = car.Heading;

            var headingRadians =
                (carHeading + CoordinateUtils.QUARTER_CIRCLE) * 2 * Math.PI / CoordinateUtils.FULL_CIRCLE;
            var forwardX = Math.Cos(headingRadians);
            var forwardY = Math.Sin(headingRadians);

            foreach (var otherCar in allCars)
            {
                // Skip if same car or invalid car
                if (otherCar.PLID == plid || otherCar.PLID == 0)
                    continue;

                // Calculate relative position
                var otherX = otherCar.X / 65536.0;
                var otherY = otherCar.Y / 65536.0;
                var relativeX = otherX - carX;
                var relativeY = otherY - carY;

                // Project into car's forward frame to find lateral offset
                var forwardDistance = relativeX * forwardX + relativeY * forwardY;
                if (forwardDistance < 0)
                    continue; // Behind the car

                var lateralOffset = Math.Abs(-forwardY * relativeX + forwardX * relativeY);
                if (lateralOffset > config.CollisionDetectionHalfWidthM)
                    continue; // Outside detection corridor

                // Distance to other car
                var distance = Math.Sqrt(relativeX * relativeX + relativeY * relativeY);

                // Skip if outside detection range
                if (distance > config.CollisionDetectionRangeM)
                    continue;

                // Calculate heading to target car using LFS coordinates
                var headingToTarget = CoordinateUtils.CalculateHeadingToTarget(relativeX, relativeY);

                // Calculate heading error (difference between current heading and heading to target)
                var headingError = CoordinateUtils.CalculateHeadingError(carHeading, headingToTarget);

                // Convert error to absolute angle in degrees (0-180)
                var errorAngleDegrees = Math.Abs(CoordinateUtils.HeadingToDegrees(Math.Abs(headingError)));
                if (errorAngleDegrees > 180)
                    errorAngleDegrees = 360 - errorAngleDegrees;

                // Check if it's been at least 2 seconds since last log for this car
                if (!lastCollisionLogTime.ContainsKey(plid) ||
                    (DateTime.Now - lastCollisionLogTime[plid]).TotalSeconds >= 2)
                {
                    logger.Log(
                        $"PLID={plid} COLLISION DANGER: Car {otherCar.PLID} at {distance:F1}m, angle={errorAngleDegrees:F1}°");
                    lastCollisionLogTime[plid] = DateTime.Now;
                }

                // Check if car is within detection angle in front and too close
                if (errorAngleDegrees <= config.CollisionDetectionAngle &&
                    distance < config.MinimumSafetyDistanceM) return true;
            }

            return false;
        }

        /// <summary>
        ///     Send initial AI car controls and settings
        /// </summary>
        private void SendAIControls(byte plid)
        {
            // Reset all inputs
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_RESET_INPUTS, Time = 0, Value = 0 } }) { PLID = plid },
                InSimClientExtensions.PacketPriority.High);

            // Press clutch fully first
            insim.Send(new IS_AIC(new List<AIInputVal>
                    { new AIInputVal { Input = AicInputType.CS_CLUTCH, Time = 0, Value = config.ClutchFullyPressed } })
                { PLID = plid }, InSimClientExtensions.PacketPriority.High);

            // Apply high throttle before starting (much higher than before)
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_THROTTLE, Time = 0, Value = 20000 } }) { PLID = plid },
                InSimClientExtensions.PacketPriority.High);

            // Set initial gear to 1st
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_GEAR, Time = 0, Value = 2 } }) { PLID = plid },
                InSimClientExtensions.PacketPriority.High);

            // Turn on ignition
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_IGNITION, Time = 0, Value = 3 } }) { PLID = plid },
                InSimClientExtensions.PacketPriority.High);

            // Disable automatic clutch
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_SET_HELP_FLAGS, Time = 0, Value = 0 } })
                { PLID = plid }, InSimClientExtensions.PacketPriority.High);

            // Request periodic info updates
            insim.Send(new IS_AIC(new List<AIInputVal>
            {
                new AIInputVal
                {
                    Input = AicInputType.CS_REPEAT_AI_INFO,
                    Time = (byte)Math.Max(1, Math.Min(255, config.AiiIntervalHundredthsMin)),
                    Value = 0
                }
            }) { PLID = plid });
        }

        /// <summary>
        ///     Send control inputs to the AI car
        /// </summary>
        private void SendControlInputs(byte plid, int steering, int throttle, int brake, byte gear, int clutchValue)
        {
            RecordControlInputs(plid, steering, throttle, brake, gear, clutchValue);

            var inputs = new List<AIInputVal>
            {
                new AIInputVal { Input = AicInputType.CS_MSX, Value = (ushort)steering },
                new AIInputVal { Input = AicInputType.CS_THROTTLE, Value = (ushort)throttle },
                new AIInputVal { Input = AicInputType.CS_BRAKE, Value = (ushort)brake },
                new AIInputVal { Input = AicInputType.CS_GEAR, Value = gear },
                new AIInputVal { Input = AicInputType.CS_CLUTCH, Value = (ushort)clutchValue }
            };

            insim.Send(new IS_AIC(inputs) { PLID = plid }, InSimClientExtensions.PacketPriority.High);
        }

        /// <summary>
        /// Record the last control inputs we sent for UI display.
        /// </summary>
        private void RecordControlInputs(byte plid, int steering, int throttle, int brake, byte gear, int clutchValue)
        {
            var steerOffset = steering - config.SteeringCenter;
            controlInputs[plid] =
                $"T:{throttle / 1000}k B:{brake / 1000}k G:{gear} C:{clutchValue / 1000}k S:{steerOffset}";
        }

        public void StopCar(byte plid)
        {
            var inputs = new List<AIInputVal>
            {
                new AIInputVal { Input = AicInputType.CS_THROTTLE, Value = 0 },
                new AIInputVal { Input = AicInputType.CS_BRAKE, Value = 65535 }
            };

            insim.Send(new IS_AIC(inputs) { PLID = plid }, InSimClientExtensions.PacketPriority.High);
        }

        /// <summary>
        /// Apply full brakes, engage handbrake, and switch off ignition to park the car.
        /// </summary>
        public void ParkCar(byte plid)
        {
            var inputs = new List<AIInputVal>
            {
                new AIInputVal { Input = AicInputType.CS_THROTTLE, Value = 0 },
                new AIInputVal { Input = AicInputType.CS_BRAKE, Value = 65535 },
                new AIInputVal { Input = AicInputType.CS_HANDBRAKE, Value = 65535 },
                new AIInputVal { Input = AicInputType.CS_IGNITION, Value = (ushort)AIInputVal_ToggleValues.SwitchOff }
            };

            insim.Send(new IS_AIC(inputs) { PLID = plid }, InSimClientExtensions.PacketPriority.High);
        }

        /// <summary>
        /// Release brakes and turn ignition on to resume driving.
        /// </summary>
        public void StartCar(byte plid)
        {
            var inputs = new List<AIInputVal>
            {
                new AIInputVal { Input = AicInputType.CS_HANDBRAKE, Value = 0 },
                new AIInputVal { Input = AicInputType.CS_BRAKE, Value = 0 },
                new AIInputVal { Input = AicInputType.CS_THROTTLE, Value = 0 },
                new AIInputVal { Input = AicInputType.CS_IGNITION, Value = (ushort)AIInputVal_ToggleValues.SwitchOn }
            };

            insim.Send(new IS_AIC(inputs) { PLID = plid }, InSimClientExtensions.PacketPriority.High);
        }
    }
}
