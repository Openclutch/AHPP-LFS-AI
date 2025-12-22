using System;
using System.Collections.Generic;
using System.Linq;
using AHPP_AI.Util;
using InSimDotNet.Packets;
using InSimClient = InSimDotNet.InSimClient;

namespace AHPP_AI.Debug
{
    /// <summary>
    ///     Provides a visual debug interface using InSim buttons
    /// </summary>
    public class DebugUI
    {
        // Connection constants
        private const byte UCID_LOCAL_PLAYER = 0; // Local player's UCID (connection ID)

        // Button constants
        // InSim docs recommend avoiding INST_ALWAYS_ON unless absolutely
        // necessary as it forces buttons to appear over the LFS interface.
        private const byte INST_NORMAL = 0;
        private const int DEBUG_UPDATE_INTERVAL_MS = 100;

        // Button positioning
        private const byte LEFT_COLUMN = 5;
        private const byte RIGHT_COLUMN = 50;
        private const byte TOP_ROW = 5;
        private const byte ROW_HEIGHT = 4;
        private const byte BUTTON_WIDTH = 20;
        private const double WAYPOINT_THRESHOLD = 1.5; // Same threshold as the AI uses
        private const int WAYPOINT_TIMEOUT_SECONDS = 600; // Same timeout as the AI uses

        // Use a high range to avoid conflicts with any other InSim programs
        // or LFS interface elements. Valid IDs are 0-239 as documented in the
        // official InSim specification.
        private readonly Dictionary<string, byte> aiButtonIds = new Dictionary<string, byte>
        {
            { "Position", 224 },
            { "Speed", 223 },
            { "Heading", 222 },
            { "Direction", 221 },
            { "Angle", 220 },
            { "Steering", 219 },
            { "Waypoint", 218 },
            { "Distance", 217 },
            { "HeadingError", 216 },
            { "TargetSpeed", 215 },
            { "ControlInfo", 214 },
            { "RouteInfo", 213 },
            { "StartAll", 206 }
        };

        // Track button states and update timing
        private readonly Dictionary<byte, byte> debugButtonsActive = new Dictionary<byte, byte>();

        private readonly InSimClient insim;
        private readonly Logger logger;

        // Button IDs by category and type
        private readonly Dictionary<string, byte> playerButtonIds = new Dictionary<string, byte>
        {
            { "RecRoute1", 239 },
            { "RecRoute2", 238 },
            { "RecRoute3", 237 },
            { "Position", 236 },
            { "Speed", 235 },
            { "Heading", 234 },
            { "Direction", 233 },
            { "Angle", 232 },
            { "Steering", 231 },
            { "Waypoint", 230 },
            { "Distance", 229 },
            { "HeadingError", 228 },
            { "TargetSpeed", 227 },
            { "ControlInfo", 226 },
            { "RouteInfo", 225 }
        };

        private readonly Dictionary<string, string> recordButtonMap = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase)
        {
            { "route1", "RecRoute1" },
            { "main_loop", "RecRoute1" },
            { "route2", "RecRoute2" },
            { "pit_entry", "RecRoute2" },
            { "route3", "RecRoute3" },
            { "detour1", "RecRoute3" }
        };

        private byte aiPLID;

        private bool debugUIInitialized;
        private DateTime lastDebugUpdate = DateTime.MinValue;
        private DateTime playerLastWaypointTime = DateTime.MinValue;

        // Player and AI tracking
        private byte playerPLID;

        // Track the player's current waypoint target (simulating what AI does)
        private int playerTargetWaypointIndex;

        /// <summary>
        ///     Initializes a new instance of the DebugUI
        /// </summary>
        /// <param name="insim">InSim connection</param>
        /// <param name="logger">Logger for debug information</param>
        public DebugUI(InSimClient insim, Logger logger)
        {
            this.insim = insim;
            this.logger = logger;
        }

        public void ShowRecordingButtons(bool show)
        {
            // Player-facing recording buttons are deprecated; ensure any legacy buttons are removed.
            DeleteButton(playerButtonIds["RecRoute1"]);
            DeleteButton(playerButtonIds["RecRoute2"]);
            DeleteButton(playerButtonIds["RecRoute3"]);
            debugButtonsActive[playerButtonIds["RecRoute1"]] = 0;
            debugButtonsActive[playerButtonIds["RecRoute2"]] = 0;
            debugButtonsActive[playerButtonIds["RecRoute3"]] = 0;
        }

        public void UpdateRecordingButton(string routeName, int count, bool recording)
        {
            // Player telemetry UI has been removed; no updates required.
            return;
        }

        public void ShowAIButtons(bool show)
        {
            var baseRow = (byte)(TOP_ROW + ROW_HEIGHT * aiButtonIds.Count);
            byte spawnId = 212;
            byte removeId = 211;
            byte removeAllId = 210;
            byte stopAllId = 209;
            byte startAllId = 206;
            byte specAllId = 208;
            byte speedId = 207;

            if (show)
            {
                CreateDebugButton(spawnId, "SPAWN AI", RIGHT_COLUMN, baseRow, BUTTON_WIDTH, ROW_HEIGHT);
                CreateDebugButton(removeId, "REMOVE AI", RIGHT_COLUMN, (byte)(baseRow + ROW_HEIGHT), BUTTON_WIDTH, ROW_HEIGHT);
                CreateDebugButton(removeAllId, "REMOVE ALL", RIGHT_COLUMN, (byte)(baseRow + ROW_HEIGHT * 2), BUTTON_WIDTH, ROW_HEIGHT);
                CreateDebugButton(stopAllId, "STOP ALL", RIGHT_COLUMN, (byte)(baseRow + ROW_HEIGHT * 3), BUTTON_WIDTH, ROW_HEIGHT);
                CreateDebugButton(startAllId, "START ALL", RIGHT_COLUMN, (byte)(baseRow + ROW_HEIGHT * 4), BUTTON_WIDTH, ROW_HEIGHT);
                CreateDebugButton(specAllId, "SPEC ALL", RIGHT_COLUMN, (byte)(baseRow + ROW_HEIGHT * 5), BUTTON_WIDTH, ROW_HEIGHT);
                CreateDebugButton(speedId, "SET SPEED", RIGHT_COLUMN, (byte)(baseRow + ROW_HEIGHT * 6), BUTTON_WIDTH, ROW_HEIGHT);
                debugButtonsActive[spawnId] = 1;
                debugButtonsActive[removeId] = 1;
                debugButtonsActive[removeAllId] = 1;
                debugButtonsActive[stopAllId] = 1;
                debugButtonsActive[startAllId] = 1;
                debugButtonsActive[specAllId] = 1;
                debugButtonsActive[speedId] = 1;
            }
            else
            {
                DeleteButton(spawnId);
                DeleteButton(removeId);
                DeleteButton(removeAllId);
                DeleteButton(stopAllId);
                DeleteButton(startAllId);
                DeleteButton(specAllId);
                DeleteButton(speedId);
                debugButtonsActive[spawnId] = 0;
                debugButtonsActive[removeId] = 0;
                debugButtonsActive[removeAllId] = 0;
                debugButtonsActive[stopAllId] = 0;
                debugButtonsActive[startAllId] = 0;
                debugButtonsActive[specAllId] = 0;
                debugButtonsActive[speedId] = 0;
            }
        }

        /// <summary>
        ///     Initialize the debug UI by creating all buttons
        /// </summary>
        public void Initialize()
        {
            if (debugUIInitialized) return;

            try
            {
                logger.Log("Initializing debug UI...");

                // Create AI debug buttons on the right side
                var row = TOP_ROW;
                foreach (var entry in aiButtonIds)
                {
                    CreateDebugButton(entry.Value, $"AI_{entry.Key.Substring(0, Math.Min(4, entry.Key.Length))}: --",
                        RIGHT_COLUMN, row, BUTTON_WIDTH, ROW_HEIGHT);
                    debugButtonsActive[entry.Value] = 1;
                    row += ROW_HEIGHT;
                }

                insim.Send(new IS_MST { Msg = "Debug UI initialized - showing info for AI" });
                debugUIInitialized = true;
                logger.Log("Debug UI initialization complete");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Error initializing debug UI");
            }
        }

        /// <summary>
        ///     Set the player ID to track
        /// </summary>
        /// <param name="plid">Player ID</param>
        public void SetPlayerPLID(byte plid)
        {
            if (plid != playerPLID)
            {
                playerPLID = plid;
                insim.Send(new IS_MST { Msg = $"Debug tracking Player PLID: {plid}" });
                logger.Log($"Set debug tracking for player PLID: {plid}");
            }
        }

        /// <summary>
        ///     Set the AI car ID to track
        /// </summary>
        /// <param name="plid">AI Player ID</param>
        public void SetAIPLID(byte plid)
        {
            if (plid != aiPLID)
            {
                aiPLID = plid;
                insim.Send(new IS_MST { Msg = $"Debug tracking AI PLID: {plid}" });
                logger.Log($"Set debug tracking for AI PLID: {plid}");
            }
        }

        /// <summary>
        ///     Update debug information displayed in UI
        /// </summary>
        /// <param name="allCars">All cars telemetry data</param>
        /// <param name="aiTargetWaypointIndices">Dictionary of AI target waypoint indices</param>
        /// <param name="aiPaths">Dictionary of AI paths</param>
        /// <param name="aiTargetSpeeds">Dictionary of AI target speeds</param>
        /// <param name="aiControlInfo">Optional dictionary with AI control information</param>
        public void UpdateDebugInfo(
            CompCar[] allCars,
            Dictionary<byte, int> aiTargetWaypointIndices = null,
            Dictionary<byte, List<Util.Waypoint>> aiPaths = null,
            Dictionary<byte, double> aiTargetSpeeds = null,
            Dictionary<byte, string> aiControlInfo = null)
        {
            // Check if we need to update (throttle updates to reduce bandwidth)
            if ((DateTime.Now - lastDebugUpdate).TotalMilliseconds < DEBUG_UPDATE_INTERVAL_MS)
                return;

            lastDebugUpdate = DateTime.Now;

            if (!debugUIInitialized) return;

            try
            {
                // Find first valid AI car if not set
                if (aiPLID == 0 && aiTargetWaypointIndices != null)
                    foreach (var kvp in aiTargetWaypointIndices)
                        if (kvp.Key > 0 && Array.Exists(allCars, c => c.PLID == kvp.Key))
                        {
                            aiPLID = kvp.Key;
                            insim.Send(new IS_MST { Msg = $"Auto-detected AI PLID: {aiPLID}" });
                            break;
                        }

                // Update AI data if available
                if (aiPLID > 0 && aiTargetWaypointIndices != null && aiPaths != null && aiTargetSpeeds != null)
                    UpdateAIDebugInfo(allCars, aiTargetWaypointIndices, aiPaths, aiTargetSpeeds, aiControlInfo);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Error updating debug UI");
            }
        }

        /// <summary>
        ///     Update debug information for the player car
        /// </summary>
        private void UpdatePlayerDebugInfo(
            CompCar[] allCars,
            Dictionary<byte, List<Util.Waypoint>> allPaths,
            Dictionary<byte, double> targetSpeeds = null)
        {
            // Get player car data from MCI
            var car = Array.Find(allCars, c => c.PLID == playerPLID);
            if (car.PLID == 0) // No MCI data yet for player PLID
                return;

            // Calculate current values
            var speedKmh = 360.0 * car.Speed / 32768.0;

            // Calculate positions in meters
            var carX = car.X / 65536.0;
            var carY = car.Y / 65536.0;
            var carZ = car.Z / 65536.0;

            // Calculate heading and direction in degrees (0-359)
            var heading = car.Heading * 360.0 / 65536.0 % 360.0;
            var direction = car.Direction * 360.0 / 65536.0 % 360.0;

            // Calculate angle between heading and direction
            var angle = (heading - direction) % 360.0;
            if (angle > 180.0) angle -= 360.0;
            if (angle < -180.0) angle += 360.0;

            // Update the basic telemetry buttons
            UpdateDebugButton(playerButtonIds["Position"], $"P_POS: {carX:F1},{carY:F1},{carZ:F1}m");
            UpdateDebugButton(playerButtonIds["Speed"], $"P_SPD: {speedKmh:F1} km/h");
            UpdateDebugButton(playerButtonIds["Heading"], $"P_HDG: {heading:F1}°");
            UpdateDebugButton(playerButtonIds["Direction"], $"P_DIR: {direction:F1}°");
            UpdateDebugButton(playerButtonIds["Angle"], $"P_ANG: {angle:F1}°");

            // Steering angle - estimated from the difference between heading and direction
            if (car.Speed > 0)
                UpdateDebugButton(playerButtonIds["Steering"], $"P_STR: {angle:F1}°");
            else
                UpdateDebugButton(playerButtonIds["Steering"], "P_STR: 0.0°");

            // Get first available AI path to use for player tracking
            if (allPaths != null && allPaths.Count > 0)
            {
                var firstAiPlid = allPaths.Keys.FirstOrDefault();
                if (firstAiPlid > 0 && allPaths[firstAiPlid] != null && allPaths[firstAiPlid].Count > 0)
                {
                    var path = allPaths[firstAiPlid];

                    // Initialize player target waypoint if needed
                    if (playerTargetWaypointIndex >= path.Count || playerLastWaypointTime == DateTime.MinValue)
                    {
                        var (closestWpIndex, distance, _) = FindClosestWaypoint(car, path);
                        playerTargetWaypointIndex = closestWpIndex;
                        playerLastWaypointTime = DateTime.Now;
                    }

                    // Get current target waypoint
                    var currentWaypoint = path[playerTargetWaypointIndex];

                    // Calculate distance to current target waypoint
                    var targetX = currentWaypoint.Position.X / 65536.0 - carX;
                    var targetY = currentWaypoint.Position.Y / 65536.0 - carY;
                    var distanceCurrent = Math.Sqrt(targetX * targetX + targetY * targetY);

                    // Next waypoint index
                    var nextIndex = (playerTargetWaypointIndex + 1) % path.Count;

                    // Handle waypoint advancement
                    var waypointTimeout = TimeSpan.FromSeconds(WAYPOINT_TIMEOUT_SECONDS);
                    if (DateTime.Now - playerLastWaypointTime > waypointTimeout)
                    {
                        // Timeout - move to next waypoint
                        var oldIndex = playerTargetWaypointIndex;
                        playerTargetWaypointIndex = nextIndex;
                        playerLastWaypointTime = DateTime.Now;
                    }
                    else if (distanceCurrent < WAYPOINT_THRESHOLD)
                    {
                        // Reached waypoint - move to next
                        var oldIndex = playerTargetWaypointIndex;
                        playerTargetWaypointIndex = nextIndex;
                        playerLastWaypointTime = DateTime.Now;
                    }

                    // Calculate desired heading to target using utility class
                    var desiredHeading = CoordinateUtils.CalculateHeadingToTarget(targetX, targetY);
                    var headingError = CoordinateUtils.CalculateHeadingError(car.Heading, desiredHeading);

                    // Convert to degrees for display
                    var headingErrorDeg = headingError * 360.0 / 65536.0;

                    // Display waypoint info - now shows TARGET waypoint, not closest
                    UpdateDebugButton(playerButtonIds["Waypoint"],
                        $"P_WP: {playerTargetWaypointIndex}/{path.Count - 1}");

                    // Display distance to target waypoint
                    UpdateDebugButton(playerButtonIds["Distance"], $"P_DST: {distanceCurrent:F1}m");

                    // Display heading error info
                    UpdateDebugButton(playerButtonIds["HeadingError"], $"P_ERR: {headingErrorDeg:F1}°");

                    // Display target speed
                    var targetSpeed = path[playerTargetWaypointIndex].SpeedLimit;
                    UpdateDebugButton(playerButtonIds["TargetSpeed"], $"P_TGT: {targetSpeed:F1} km/h");

                    // Additional route info
                    var wpX = currentWaypoint.Position.X / 65536.0;
                    var wpY = currentWaypoint.Position.Y / 65536.0;
                    UpdateDebugButton(playerButtonIds["RouteInfo"], $"P_WP_POS: {wpX:F1},{wpY:F1}");
                }
            }
        }

        /// <summary>
        ///     Find the closest waypoint to a car
        /// </summary>
        private (int index, double distance, int desiredHeading) FindClosestWaypoint(CompCar car,
            List<Util.Waypoint> path)
        {
            var carX = car.X / 65536.0;
            var carY = car.Y / 65536.0;

            var closestIndex = 0;
            var minDistance = double.MaxValue;
            var desiredHeading = 0;

            for (var i = 0; i < path.Count; i++)
            {
                var wpX = path[i].Position.X / 65536.0;
                var wpY = path[i].Position.Y / 65536.0;

                var dx = wpX - carX;
                var dy = wpY - carY;
                var distance = Math.Sqrt(dx * dx + dy * dy);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestIndex = i;

                    // Calculate heading to waypoint using utility class
                    desiredHeading = CoordinateUtils.CalculateHeadingToTarget(dx, dy);
                }
            }

            return (closestIndex, minDistance, desiredHeading);
        }

        /// <summary>
        ///     Update debug information for the AI car
        /// </summary>
        private void UpdateAIDebugInfo(
            CompCar[] allCars,
            Dictionary<byte, int> aiTargetWaypointIndices,
            Dictionary<byte, List<Util.Waypoint>> aiPaths,
            Dictionary<byte, double> aiTargetSpeeds,
            Dictionary<byte, string> aiControlInfo = null)
        {
            // Get AI car data
            var car = Array.Find(allCars, c => c.PLID == aiPLID);
            if (car.PLID == 0) // No MCI data yet for AI PLID
                return;

            // Check if we have path data for this AI
            if (!aiTargetWaypointIndices.TryGetValue(aiPLID, out var targetIndex) ||
                !aiPaths.TryGetValue(aiPLID, out var path) ||
                path == null || targetIndex >= path.Count)
                return;

            // Calculate current values
            var speedKmh = 360.0 * car.Speed / 32768.0;
            var carX = car.X / 65536.0;
            var carY = car.Y / 65536.0;
            var carZ = car.Z / 65536.0;

            // Get current waypoint info
            var currentWaypoint = path[targetIndex];
            var wpX = currentWaypoint.Position.X / 65536.0;
            var wpY = currentWaypoint.Position.Y / 65536.0;

            // Calculate distance to waypoint
            var dx = wpX - carX;
            var dy = wpY - carY;
            var distance = Math.Sqrt(dx * dx + dy * dy);

            // Calculate heading and direction in degrees (0-359)
            var heading = car.Heading * 360.0 / 65536.0 % 360.0;
            var direction = car.Direction * 360.0 / 65536.0 % 360.0;

            // Calculate angle between heading and direction
            var angle = (heading - direction) % 360.0;
            if (angle > 180.0) angle -= 360.0;
            if (angle < -180.0) angle += 360.0;

            // Get target speed for this waypoint
            var targetSpeed = aiTargetSpeeds.TryGetValue(aiPLID, out var speed) ? speed : 0.0;

            // Calculate desired heading to waypoint using utility class
            var desiredHeading = CoordinateUtils.CalculateHeadingToTarget(dx, dy);
            var headingError = CoordinateUtils.CalculateHeadingError(car.Heading, desiredHeading);

            // Convert to degrees for display
            var headingErrorDeg = headingError * 360.0 / 65536.0;

            // Update the debug buttons with current values
            UpdateDebugButton(aiButtonIds["Position"], $"AI_POS: {carX:F1},{carY:F1},{carZ:F1}m");
            UpdateDebugButton(aiButtonIds["Speed"], $"AI_SPD: {speedKmh:F1} km/h");
            UpdateDebugButton(aiButtonIds["Heading"], $"AI_HDG: {heading:F1}°");
            UpdateDebugButton(aiButtonIds["Direction"], $"AI_DIR: {direction:F1}°");
            UpdateDebugButton(aiButtonIds["Angle"], $"AI_ANG: {angle:F1}°");

            // Steering angle - estimated from the difference between heading and direction
            if (car.Speed > 0)
                UpdateDebugButton(aiButtonIds["Steering"], $"AI_STR: {angle:F1}°");
            else
                UpdateDebugButton(aiButtonIds["Steering"], "AI_STR: 0.0°");

            UpdateDebugButton(aiButtonIds["Waypoint"], $"AI_WP: {targetIndex}/{path.Count - 1}");
            UpdateDebugButton(aiButtonIds["Distance"], $"AI_DST: {distance:F1}m");
            UpdateDebugButton(aiButtonIds["TargetSpeed"], $"AI_TGT: {targetSpeed:F1} km/h");
            UpdateDebugButton(aiButtonIds["HeadingError"], $"AI_ERR: {headingErrorDeg:F1}°");

            // Additional info
            if (aiControlInfo != null && aiControlInfo.TryGetValue(aiPLID, out var controlInfo))
                UpdateDebugButton(aiButtonIds["ControlInfo"], $"AI_CTL: {controlInfo}");

            UpdateDebugButton(aiButtonIds["RouteInfo"], $"AI_WP_POS: {wpX:F1},{wpY:F1}");
        }

        /// <summary>
        ///     Create a debug button
        /// </summary>
        private void CreateDebugButton(byte id, string text, byte left, byte top, byte width, byte height)
        {
            try
            {
                var btn = new IS_BTN
                {
                    ReqI = 1,
                    UCID = 0,
                    ClickID = id,
                    Inst = INST_NORMAL,
                    BStyle = ButtonStyles.ISB_DARK | ButtonStyles.ISB_LEFT | ButtonStyles.ISB_CLICK,
                    TypeIn = 0,
                    L = left,
                    T = top,
                    W = width,
                    H = height,
                    Text = text
                };

                insim.Send(btn);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Error creating debug button ID {id}");
            }
        }

        /// <summary>
        ///     Update the text of a debug button
        /// </summary>
        private void UpdateDebugButton(byte id, string text)
        {
            if (!debugButtonsActive.TryGetValue(id, out var active) || active == 0)
                return;

            try
            {
                var btn = new IS_BTN
                {
                    ReqI = 1,
                    UCID = 0,
                    ClickID = id,
                    Text = text
                };

                insim.Send(btn);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Error updating debug button ID {id}");

                // Mark button as inactive to avoid repeated errors
                debugButtonsActive[id] = 0;
            }
        }

        /// <summary>
        /// Build a recording button label with simple color cues for state.
        /// </summary>
        private string BuildRecordLabel(string key, int count, bool recording)
        {
            // Classic color codes: ^1 red when active, ^2 green when idle
            var routeIdx = key[key.Length - 1];
            var color = recording ? "^1" : "^2";
            var suffix = recording ? $": {count}" : string.Empty;
            return $"{color}REC {routeIdx}{suffix}";
        }

        private void DeleteButton(byte id)
        {
            try
            {
                var del = new IS_BFN
                {
                    ReqI = 1,
                    SubT = ButtonFunction.BFN_DEL_BTN,
                    UCID = 0,
                    ClickID = id,
                    ClickMax = id,
                    Inst = 0
                };

                insim.Send(del);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Error deleting button ID {id}");
            }
        }
    }
}
