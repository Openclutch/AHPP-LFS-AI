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

        // Enumeration for wall recovery state machine
        public enum WallRecoveryState
        {
            Normal, // Normal operation
            Detected, // Stuck against wall detected
            Reversing, // Step 1: Reverse away from wall
            Turning, // Step 2: Turn away from wall
            Stopping, // Step 3: Come to a complete stop
            MovingForward, // Step 4: Attempt to move forward
            Complete // Recovery complete
        }

        // Wall recovery configuration
        private const int MAX_WALL_RECOVERY_ATTEMPTS = 2;
        private const double REVERSE_TIME_MS = 2000;
        private const double TURNING_TIME_MS = 3000;
        private const double STOPPING_TIME_MS = 500;
        private const double MOVING_FORWARD_TIME_MS = 500;
        private const double POSITION_CHANGE_THRESHOLD = 2; // Smaller threshold for detecting stuck
        private const double STUCK_CHECK_INTERVAL_MS = 1000; // Check progress every second

        private readonly AIConfig config;

        // Control state
        private readonly Dictionary<byte, string> controlInfo = new Dictionary<byte, string>();

        // Engine state tracking
        private readonly Dictionary<byte, bool> engineRunning = new Dictionary<byte, bool>();

        private readonly Dictionary<byte, EngineStartState>
            engineStartStates = new Dictionary<byte, EngineStartState>();

        private readonly Dictionary<byte, DateTime> engineStateTimers = new Dictionary<byte, DateTime>();
        private readonly GearboxController gearboxController;
        private readonly InSimClient insim;
        private readonly Dictionary<byte, DateTime> lastCollisionLogTime = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, double> lastProgressDistance = new Dictionary<byte, double>();
        private readonly Dictionary<byte, DateTime> lastStuckCheckTime = new Dictionary<byte, DateTime>();
        private Action<byte> recoveryFailedHandler;
        private readonly Logger logger;
        private readonly Dictionary<byte, int> stuckHeading = new Dictionary<byte, int>();
        private readonly Dictionary<byte, double> stuckPositionX = new Dictionary<byte, double>();
        private readonly Dictionary<byte, double> stuckPositionY = new Dictionary<byte, double>();
        private readonly Dictionary<byte, int> wallRecoveryAttempts = new Dictionary<byte, int>();

        // Wall recovery state tracking
        private readonly Dictionary<byte, WallRecoveryState> wallRecoveryStates =
            new Dictionary<byte, WallRecoveryState>();

        private readonly Dictionary<byte, int> wallRecoverySteeringDirection = new Dictionary<byte, int>();
        private readonly Dictionary<byte, DateTime> wallRecoveryTimers = new Dictionary<byte, DateTime>();
        private readonly WaypointFollower waypointFollower;

        public AIDriver(
            AIConfig config,
            Logger logger,
            WaypointFollower waypointFollower,
            GearboxController gearboxController,
            InSimClient insim)
        {
            this.config = config;
            this.logger = logger;
            this.waypointFollower = waypointFollower;
            this.gearboxController = gearboxController;
            this.insim = insim;
        }

        public void SetRecoveryFailedHandler(Action<byte> handler)
        {
            recoveryFailedHandler = handler;
        }

        /// <summary>
        ///     Initialize a new AI driver
        /// </summary>
        public void InitializeDriver(byte plid)
        {
            gearboxController.InitializeGearbox(plid);
            controlInfo[plid] = "Initializing";
            engineRunning[plid] = false; // Always start with engine "not running" to trigger start procedure
            engineStartStates[plid] = EngineStartState.PressClutch; // Start in clutch pressed state
            engineStateTimers[plid] = DateTime.Now;

            // Initialize wall recovery state
            wallRecoveryStates[plid] = WallRecoveryState.Normal;
            wallRecoveryTimers[plid] = DateTime.Now;
            wallRecoveryAttempts[plid] = 0;
            stuckPositionX[plid] = 0;
            stuckPositionY[plid] = 0;
            stuckHeading[plid] = 0;
            wallRecoverySteeringDirection[plid] = 1; // Default to turning right in recovery
            lastStuckCheckTime[plid] = DateTime.Now;
            lastProgressDistance[plid] = 0;

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
            WaypointManager waypointManager,
            AIConfig config,
            List<ObjectInfo> layoutObjects)
        {
            try
            {
                // Initialize path if needed
                if (!waypointFollower.InitializePath(plid, car, waypointManager, config, layoutObjects))
                    return;

                // Check if we should activate approach curve based on distance to track
                // TODO commented out for now, causing AI to never get onto the track
                // waypointFollower.CheckAndUpdateApproachCurve(plid, car);

                // Calculate position in meters
                var carX = car.X / 65536.0;
                var carY = car.Y / 65536.0;
                var carZ = car.Z / 65536.0;

                // Initialize path if needed
                if (!waypointFollower.InitializePath(plid, car, waypointManager, config, layoutObjects))
                    return;

                // Get current speed in km/h
                var speedKmh = 360.0 * car.Speed / 32768.0;
                double currentHeading = car.Heading;


                // Check if we're stuck against a wall
                if (wallRecoveryStates[plid] != WallRecoveryState.Normal && config.WallRecoveryEnabled)
                    // Handle wall recovery procedure
                    if (HandleWallRecovery(plid, carX, carY, speedKmh, (int)currentHeading))
                    {
                        // Update control info for debug display to show wall recovery
                        controlInfo[plid] = $"Wall Recovery: {wallRecoveryStates[plid]}";
                        return; // Skip normal controls while handling wall recovery
                    }

                // Handle engine stall and restart if necessary
                if (HandleEngineState(plid, speedKmh))
                {
                    // Update control info for debug display to show stall recovery
                    controlInfo[plid] = $"Engine Restart: {engineStartStates[plid]}";
                    return; // Skip normal controls while handling stall
                }

                // Calculate distance and heading to target waypoint
                var (distance, desiredHeading, headingError) = waypointFollower.CalculateTargetData(
                    plid, carX, carY, (int)currentHeading);

                // Check progress toward waypoint and also check if stuck
                var recoveryFailed = waypointFollower.CheckWaypointProgress(plid, distance);
                if (recoveryFailed)
                {
                    controlInfo[plid] = "Recovery reset";
                    recoveryFailedHandler?.Invoke(plid);
                    return;
                }

                // Check if we're making progress at regular intervals
                CheckIfStuck(plid, carX, carY, (int)currentHeading, speedKmh, distance);

                // Calculate steering, throttle and brake
                var steering = CalculateSteering(headingError);

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
                gearboxController.UpdateGearbox(plid, speedKmh);
                gearboxController.ApplyBrakingClutch(plid, brake);

                // Get current gearbox state
                var (gear, clutchValue, clutchState) = gearboxController.GetGearboxInfo(plid);

                // Check if we should apply throttle based on clutch state
                if (!gearboxController.ShouldApplyThrottle(plid, speedKmh)) throttle = 0;

                // Check for potential collisions
                var collisionDanger = DetectPotentialCollision(plid, car, allCars);
                if (collisionDanger)
                {
                    // Override controls - stop the car
                    throttle = 0;
                    brake = 65535; // Full brake
                    clutchValue = 65535;

                    // Update control info for debug display
                    controlInfo[plid] = "COLLISION AVOIDANCE: Stopping";
                }

                // Update control info for debug display
                controlInfo[plid] =
                    $"T:{throttle / 1000} B:{brake / 1000} G:{gear} C:{clutchValue / 1000} S:{clutchState} Eng:{(engineRunning[plid] ? "ON" : "OFF")}";

                // Send AI controls
                SendControlInputs(plid, steering, throttle, brake, gear, clutchValue);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Error processing AI control for PLID={plid}");
            }
        }

        /// <summary>
        ///     Check if the vehicle is stuck against a wall based on lack of movement
        /// </summary>
        private void CheckIfStuck(byte plid, double carX, double carY, int currentHeading, double speedKmh,
            double currentDistance)
        {
            // Only check if we're in normal operation (not already in recovery)
            if (wallRecoveryStates[plid] != WallRecoveryState.Normal)
                return;

            // Only check at regular intervals
            if ((DateTime.Now - lastStuckCheckTime[plid]).TotalMilliseconds < STUCK_CHECK_INTERVAL_MS)
                return;

            lastStuckCheckTime[plid] = DateTime.Now;

            // If this is the first check, initialize
            if (stuckPositionX[plid] == 0 && stuckPositionY[plid] == 0)
            {
                stuckPositionX[plid] = carX;
                stuckPositionY[plid] = carY;
                stuckHeading[plid] = currentHeading;
                lastProgressDistance[plid] = currentDistance;
                return;
            }

            // Calculate how far we've moved since last check
            var dx = carX - stuckPositionX[plid];
            var dy = carY - stuckPositionY[plid];
            var distanceMoved = Math.Sqrt(dx * dx + dy * dy);

            // Calculate progress toward waypoint
            var distanceProgress = lastProgressDistance[plid] - currentDistance;

            // Detect if we're making insufficient progress
            var isStuck = false;

            // Option 1: We haven't moved enough physically
            if (distanceMoved < POSITION_CHANGE_THRESHOLD && speedKmh < 5.0)
            {
                isStuck = true;
                logger.Log($"PLID={plid} STUCK DETECTION: Moved only {distanceMoved:F2}m, speed={speedKmh:F1}km/h");
            }

            if (isStuck)
            {
                // Increment wall recovery attempts 
                wallRecoveryAttempts[plid]++;

                logger.Log(
                    $"PLID={plid} POSSIBLE WALL STUCK: Attempt {wallRecoveryAttempts[plid]} of {MAX_WALL_RECOVERY_ATTEMPTS}");

                // Check if we need to start wall recovery
                if (wallRecoveryAttempts[plid] >= MAX_WALL_RECOVERY_ATTEMPTS)
                {
                    // Start wall recovery procedure
                    wallRecoveryStates[plid] = WallRecoveryState.Detected;
                    wallRecoveryTimers[plid] = DateTime.Now;

                    // Choose a recovery steering direction - opposite of current heading error
                    var (_, _, headingError) = waypointFollower.CalculateTargetData(plid, carX, carY, currentHeading);
                    wallRecoverySteeringDirection[plid] = Math.Sign(headingError) == 0 ? 1 : -Math.Sign(headingError);

                    logger.Log(
                        $"PLID={plid} WALL STUCK: Starting wall recovery procedure, turning direction: {wallRecoverySteeringDirection[plid]}");
                }
            }
            else
            {
                // Making good progress, reset attempts
                if (wallRecoveryAttempts[plid] > 0)
                {
                    logger.Log($"PLID={plid} PROGRESS OK: Resetting stuck detection, moved {distanceMoved:F2}m");
                    wallRecoveryAttempts[plid] = 0;
                }
            }

            // Update for next check
            stuckPositionX[plid] = carX;
            stuckPositionY[plid] = carY;
            stuckHeading[plid] = currentHeading;
            lastProgressDistance[plid] = currentDistance;
        }

        /// <summary>
        ///     Handle wall recovery procedure with reversing and turning
        /// </summary>
        private bool HandleWallRecovery(byte plid, double carX, double carY, double speedKmh, int currentHeading)
        {
            // Get elapsed time since last state change
            var elapsed = DateTime.Now.Subtract(wallRecoveryTimers[plid]).TotalMilliseconds;

            // Get the turning direction
            var turnDirection = wallRecoverySteeringDirection[plid];

            // State machine for wall recovery
            switch (wallRecoveryStates[plid])
            {
                case WallRecoveryState.Detected:
                    // Start the recovery procedure by reversing
                    wallRecoveryStates[plid] = WallRecoveryState.Reversing;
                    wallRecoveryTimers[plid] = DateTime.Now;
                    logger.Log($"PLID={plid} WALL RECOVERY: Reversing");
                    return true;

                case WallRecoveryState.Reversing:
                    // Reverse away from wall - steer opposite to get away from wall
                    // Center steering plus slight opposite turn
                    var reverseSteeringValue = config.SteeringCenter + turnDirection * 15000;

                    // First fully press clutch, then apply reverse gear (0 in LFS), with moderate throttle
                    SendControlInputs(plid, reverseSteeringValue, 50000, 0, 0,
                        config.ClutchFullyPressed); // Press clutch + reverse gear

                    if (elapsed >= 100)
                        SendControlInputs(plid, reverseSteeringValue, 50000, 0, 0,
                            config.ClutchReleased); // Press clutch + reverse gear

                    // Check if we've reversed long enough
                    if (elapsed >= REVERSE_TIME_MS)
                    {
                        wallRecoveryStates[plid] = WallRecoveryState.Turning;
                        wallRecoveryTimers[plid] = DateTime.Now;
                        logger.Log($"PLID={plid} WALL RECOVERY: Turning");
                    }

                    return true;

                case WallRecoveryState.Turning:
                    // Apply hard turn while still in reverse to get the car in a new direction
                    // Turn aggressively in the chosen direction
                    var turnSteeringValue = config.SteeringCenter + turnDirection * 30000;

                    // Keep in reverse with throttle
                    SendControlInputs(plid, turnSteeringValue, 40000, 0, 0,
                        config.ClutchReleased); // 0 = reverse gear in LFS

                    // Check if we've turned enough
                    if (elapsed >= TURNING_TIME_MS)
                    {
                        wallRecoveryStates[plid] = WallRecoveryState.Stopping;
                        wallRecoveryTimers[plid] = DateTime.Now;
                        logger.Log($"PLID={plid} WALL RECOVERY: Stopping");
                    }

                    return true;

                case WallRecoveryState.Stopping:
                    // Come to a complete stop by applying brakes
                    SendControlInputs(plid, config.SteeringCenter, 0, 65000, 2,
                        config.ClutchFullyPressed); // 2 = 1st gear

                    // Wait for car to stop
                    if (elapsed >= STOPPING_TIME_MS)
                    {
                        wallRecoveryStates[plid] = WallRecoveryState.MovingForward;
                        wallRecoveryTimers[plid] = DateTime.Now;
                        logger.Log($"PLID={plid} WALL RECOVERY: Moving forward");
                    }

                    return true;

                case WallRecoveryState.MovingForward:
                    // Move forward with slight steering in recovery direction
                    var forwardSteeringValue = config.SteeringCenter + turnDirection * 5000;

                    // Apply forward with full throttle in 1st gear
                    SendControlInputs(plid, forwardSteeringValue, 65000, 0, 2, 0); // 2 = 1st gear

                    // Check if we've moved forward enough
                    if (elapsed >= MOVING_FORWARD_TIME_MS)
                    {
                        // Reset recovery state
                        wallRecoveryStates[plid] = WallRecoveryState.Complete;
                        wallRecoveryTimers[plid] = DateTime.Now;
                        logger.Log($"PLID={plid} WALL RECOVERY: Complete");
                    }

                    return true;

                case WallRecoveryState.Complete:
                    // Final cleanup and return to normal operation
                    wallRecoveryStates[plid] = WallRecoveryState.Normal;
                    wallRecoveryAttempts[plid] = 0;
                    stuckPositionX[plid] = carX;
                    stuckPositionY[plid] = carY;
                    stuckHeading[plid] = currentHeading;
                    logger.Log($"PLID={plid} WALL RECOVERY: Returning to normal operation");
                    return false;

                default:
                    return false;
            }
        }

        /// <summary>
        ///     Update engine state from AI info packet
        /// </summary>
        public void UpdateEngineState(byte plid, bool isRunning)
        {
            // Only log if state changed
            if (engineRunning.ContainsKey(plid) && engineRunning[plid] != isRunning)
            {
                logger.Log($"PLID={plid} ENGINE: {(isRunning ? "STARTED" : "STALLED")}");

                // If engine just stalled, initiate recovery sequence
                if (!isRunning)
                {
                    engineStartStates[plid] = EngineStartState.StallDetected;
                    engineStateTimers[plid] = DateTime.Now;
                    logger.Log($"PLID={plid} STALL: Starting engine recovery procedure");
                }
            }

            engineRunning[plid] = isRunning;
        }

        /// <summary>
        ///     Handle engine stall and restart process
        /// </summary>
        private bool HandleEngineState(byte plid, double speedKmh)
        {
            // If engine is running, no need for recovery
            if (engineRunning[plid] && engineStartStates[plid] == EngineStartState.Normal)
                return false;

            // Get elapsed time since last state change
            var elapsed = DateTime.Now.Subtract(engineStateTimers[plid]).TotalMilliseconds;

            // State machine for engine restart procedure
            switch (engineStartStates[plid])
            {
                case EngineStartState.StallDetected:
                    // Initialize restart procedure
                    engineStartStates[plid] = EngineStartState.PressClutch;
                    engineStateTimers[plid] = DateTime.Now;
                    logger.Log($"PLID={plid} STALL: Pressing clutch");
                    return true;

                case EngineStartState.PressClutch:
                    // Press clutch fully and apply some throttle
                    SendControlInputs(plid, config.SteeringCenter, 3000, 0, 2, config.ClutchFullyPressed);

                    // Wait for clutch press delay
                    if (elapsed >= 500)
                    {
                        engineStartStates[plid] = EngineStartState.TurnIgnition;
                        engineStateTimers[plid] = DateTime.Now;
                        logger.Log($"PLID={plid} STALL: Turning ignition");
                    }

                    return true;

                case EngineStartState.TurnIgnition:
                    // Keep clutch pressed and apply significant throttle before ignition
                    SendControlInputs(plid, config.SteeringCenter, 15000, 0, 2, config.ClutchFullyPressed);

                    // First turn ignition off, then back on to crank the engine
                    if (elapsed < 300)
                    {
                        // Turn ignition off
                        insim.Send(new IS_AIC(new List<AIInputVal>
                                { new AIInputVal { Input = AicInputType.CS_IGNITION, Time = 0, Value = 2 } })
                            { PLID = plid });
                    }
                    else if (elapsed < 100)
                    {
                        // Small pause between off and on, maintain throttle
                    }
                    else
                    {
                        // Turn ignition on (crank the engine) while maintaining throttle
                        insim.Send(new IS_AIC(new List<AIInputVal>
                                { new AIInputVal { Input = AicInputType.CS_IGNITION, Time = 0, Value = 3 } })
                            { PLID = plid });

                        if (elapsed >= 2000)
                        {
                            engineStartStates[plid] = EngineStartState.ReleaseClutch;
                            engineStateTimers[plid] = DateTime.Now;
                            logger.Log($"PLID={plid} STALL: Preparing for clutch release with throttle");
                        }
                    }

                    return true;

                case EngineStartState.ReleaseClutch:
                    // Use config parameters for clutch values and timing but much slower release
                    var totalReleaseTime =
                        config.ClutchReleaseSteps * config.ClutchReleaseIntervalMs * 5; // 5x normal time
                    var releaseProgress = Math.Min(1.0, elapsed / totalReleaseTime);

                    // More gradual curve with longer initial period
                    var progressCurve = Math.Pow(releaseProgress, 3) * (3 - 2 * Math.Pow(releaseProgress, 1.5));

                    // Calculate clutch value from fully pressed to released based on config values
                    var clutchValue = (int)(config.ClutchFullyPressed * (1 - progressCurve));

                    // Apply much more throttle before and during clutch release
                    // Start with high throttle and maintain it throughout
                    var throttle = 20000; // Much higher base throttle

                    // Use lower gear for initial starts
                    byte gear = 2; // 1st gear

                    // Send controls with extremely gradual clutch release
                    SendControlInputs(plid, config.SteeringCenter, throttle, 0, gear, clutchValue);

                    // Log clutch release progress periodically
                    if ((int)(elapsed / 1000) != (int)((elapsed - 20) / 1000))
                        logger.Log(
                            $"PLID={plid} CLUTCH: Release={progressCurve:F2}, Value={clutchValue}, Throttle={throttle}");

                    // Transition to recovery state when clutch is fully released
                    if (progressCurve >= 1.0)
                    {
                        engineStartStates[plid] = EngineStartState.Recovery;
                        engineStateTimers[plid] = DateTime.Now;
                        logger.Log($"PLID={plid} STALL: Clutch release complete");
                    }

                    return true;

                case EngineStartState.Recovery:
                    // Much longer recovery period with maintained throttle
                    if (elapsed >= 100)
                    {
                        engineStartStates[plid] = EngineStartState.Normal;
                        logger.Log($"PLID={plid} STALL: Returning to normal operation");
                        return false;
                    }

                    // Hold high throttle to prevent immediate stall
                    SendControlInputs(plid, config.SteeringCenter, 15000, 0, 2, 0);
                    return true;
                default:
                    // Should not get here, but reset if it does
                    engineStartStates[plid] = EngineStartState.Normal;
                    return false;
            }
        }

        /// <summary>
        ///     Send ignition command to start the engine
        /// </summary>
        private void SendIgnitionCommand(byte plid)
        {
            // Turn on ignition
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_IGNITION, Time = 0, Value = 3 } }) { PLID = plid });
        }

        /// <summary>
        ///     Get current control info for debugging
        /// </summary>
        public string GetControlInfo(byte plid)
        {
            return controlInfo.ContainsKey(plid) ? controlInfo[plid] : "Not initialized";
        }

        /// <summary>
        ///     Calculate steering value based on heading error
        /// </summary>
        private int CalculateSteering(int headingError)
        {
            const int STEERING_CENTER = 32768;

            // Basic proportional steering - invert the sign to correct direction
            return STEERING_CENTER - (int)(Math.Sign(headingError) *
                                           Math.Min(1.0, Math.Abs(headingError) / 16384.0) *
                                           (STEERING_CENTER - 100));
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
                { new AIInputVal { Input = AicInputType.CS_RESET_INPUTS, Time = 0, Value = 0 } }) { PLID = plid });

            // Press clutch fully first
            insim.Send(new IS_AIC(new List<AIInputVal>
                    { new AIInputVal { Input = AicInputType.CS_CLUTCH, Time = 0, Value = config.ClutchFullyPressed } })
                { PLID = plid });

            // Apply high throttle before starting (much higher than before)
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_THROTTLE, Time = 0, Value = 20000 } }) { PLID = plid });

            // Set initial gear to 1st
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_GEAR, Time = 0, Value = 2 } }) { PLID = plid });

            // Turn on ignition
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_IGNITION, Time = 0, Value = 3 } }) { PLID = plid });

            // Disable automatic clutch
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_SET_HELP_FLAGS, Time = 0, Value = 0 } }) { PLID = plid });

            // Request periodic info updates
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_REPEAT_AI_INFO, Time = 10, Value = 0 } }) { PLID = plid });
        }

        /// <summary>
        ///     Send control inputs to the AI car
        /// </summary>
        private void SendControlInputs(byte plid, int steering, int throttle, int brake, byte gear, int clutchValue)
        {
            var inputs = new List<AIInputVal>
            {
                new AIInputVal { Input = AicInputType.CS_MSX, Value = (ushort)steering },
                new AIInputVal { Input = AicInputType.CS_THROTTLE, Value = (ushort)throttle },
                new AIInputVal { Input = AicInputType.CS_BRAKE, Value = (ushort)brake },
                new AIInputVal { Input = AicInputType.CS_GEAR, Value = gear },
                new AIInputVal { Input = AicInputType.CS_CLUTCH, Value = (ushort)clutchValue }
            };

            insim.Send(new IS_AIC(inputs) { PLID = plid });
        }

        public void StopCar(byte plid)
        {
            var inputs = new List<AIInputVal>
            {
                new AIInputVal { Input = AicInputType.CS_THROTTLE, Value = 0 },
                new AIInputVal { Input = AicInputType.CS_BRAKE, Value = 65535 }
            };

            insim.Send(new IS_AIC(inputs) { PLID = plid });
        }
    }
}
