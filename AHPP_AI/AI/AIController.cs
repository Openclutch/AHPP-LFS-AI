using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using AHPP_AI.Debug;
using AHPP_AI.Waypoint;
using InSimDotNet.Out;
using InSimDotNet;
using InSimDotNet.Packets;
using AHPP_AI.UI;
using AHPP_AI.Util;
using InSimClient = InSimDotNet.InSimClient;
using Vec = AHPP_AI.Util.Vec;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Main controller coordinating AI car behavior and management
    /// </summary>
    public class AIController
    {
        private readonly List<byte> aiPLIDs = new List<byte>();
        private readonly Dictionary<byte, Vec> aiSpawnPositions = new Dictionary<byte, Vec>();
        private readonly AIConfig config;

        // Debug UI
        private readonly DebugUI debugUI;
        private readonly AIDriver driver;

        // Engine state tracking
        private readonly Dictionary<byte, bool> engineStateMap = new Dictionary<byte, bool>();
        private readonly GearboxController gearboxController;
        private readonly InSimClient insim;
        private readonly LFSLayout lfsLayout;
        private readonly Logger logger;
        private readonly WaypointFollower waypointFollower;
        private readonly WaypointManager waypointManager;
        private readonly PathManager pathManager;
        private readonly RouteRecorder routeRecorder;
        private readonly RouteLibrary routeLibrary;
        private readonly AILightController lightController;
        private readonly Dictionary<string, RouteMetadata> routePresets = new Dictionary<string, RouteMetadata>(StringComparer.OrdinalIgnoreCase);
        private readonly MainUI mainUI;
        private readonly Dictionary<byte, string> currentRoute = new Dictionary<byte, string>();
        private readonly Dictionary<byte, BranchRouteInfo> activeBranchSelections = new Dictionary<byte, BranchRouteInfo>();
        private readonly Random branchRandom = new Random();
        private readonly OutGauge outGauge;
        private string recordingRouteName = "main_loop";

        private float playerThrottle;
        private float playerBrake;
        private float playerSteering;

        private byte playerPLID;

        // Car tracking
        private CompCar[] allCars = Array.Empty<CompCar>();
        private bool debugUIInitialized;

        /// <summary>
        ///     Initialize the AI controller with dependencies
        /// </summary>
        public AIController(
            InSimClient insim,
            Logger logger,
            WaypointManager waypointManager,
            LFSLayout lfsLayout,
            RouteLibrary routeLibrary,
            bool debugEnabled = false)
        {
            this.insim = insim;
            this.logger = logger;
            this.waypointManager = waypointManager;
            this.lfsLayout = lfsLayout;
            this.routeLibrary = routeLibrary;

            // Create configuration
            config = new AIConfig { DebugEnabled = debugEnabled };

            pathManager = new PathManager(waypointManager, logger, routeLibrary);
            pathManager.LoadRoutes(config);

            // Initialize debug UI if enabled
            if (debugEnabled) debugUI = new DebugUI(insim, logger);

            // Create component hierarchy
            var steeringCalculator = new SteeringCalculator(logger);
            waypointFollower = new WaypointFollower(config, logger, steeringCalculator);
            gearboxController = new GearboxController(config, logger);
            lightController = new AILightController(insim, logger);
            driver = new AIDriver(config, logger, waypointFollower, gearboxController, insim, lightController);
            driver.SetRecoveryFailedHandler(ResetAI);
            mainUI = new MainUI(insim, logger);
            routeRecorder = new RouteRecorder(logger, lfsLayout, debugUI, routeLibrary, mainUI);
            outGauge = new OutGauge();
            outGauge.PacketReceived += OnOutGauge;
            InitializeRoutePresets();
        }

        /// <summary>
        /// Seed the default recording presets for the main loop, pit entry and detours.
        /// </summary>
        private void InitializeRoutePresets()
        {
            routePresets["main_loop"] = new RouteMetadata
            {
                Name = "main_loop",
                Type = RouteType.MainLoop,
                Description = "Primary loop that AIs can circulate on continuously.",
                IsLoop = true,
                DefaultSpeedLimit = 60
            };

            routePresets["pit_entry"] = new RouteMetadata
            {
                Name = "pit_entry",
                Type = RouteType.PitEntry,
                Description = "Route from pits or spawn to the main loop entry point.",
                AttachMainIndex = 0,
                DefaultSpeedLimit = 50
            };

            routePresets["detour1"] = new RouteMetadata
            {
                Name = "detour1",
                Type = RouteType.Detour,
                Description = "Optional detour route (slot 1).",
                DefaultSpeedLimit = 60
            };

            routePresets["detour2"] = new RouteMetadata
            {
                Name = "detour2",
                Type = RouteType.Detour,
                Description = "Optional detour route (slot 2).",
                DefaultSpeedLimit = 60
            };

            routePresets["detour3"] = new RouteMetadata
            {
                Name = "detour3",
                Type = RouteType.Detour,
                Description = "Optional detour route (slot 3).",
                DefaultSpeedLimit = 60
            };

            // Legacy mappings to keep the debug buttons usable
            routePresets["route1"] = CloneMetadata(routePresets["main_loop"]);
            routePresets["route2"] = CloneMetadata(routePresets["pit_entry"]);
            routePresets["route3"] = CloneMetadata(routePresets["detour1"]);
        }

        /// <summary>
        /// Make a copy of route metadata to avoid mutating presets.
        /// </summary>
        private static RouteMetadata CloneMetadata(RouteMetadata metadata)
        {
            if (metadata == null) return new RouteMetadata();
            return new RouteMetadata
            {
                Name = metadata.Name,
                Type = metadata.Type,
                Description = metadata.Description,
                Track = metadata.Track,
                Layout = metadata.Layout,
                IsLoop = metadata.IsLoop,
                AttachMainIndex = metadata.AttachMainIndex,
                RejoinMainIndex = metadata.RejoinMainIndex,
                DefaultSpeedLimit = metadata.DefaultSpeedLimit
            };
        }

        /// <summary>
        /// Build recording metadata using presets and any overrides provided by the caller.
        /// </summary>
        private RouteMetadata BuildRecordingMetadata(string name, RouteType type, int? attachIndex, int? rejoinIndex)
        {
            var key = string.IsNullOrWhiteSpace(name) ? "route" : name;
            RouteMetadata metadata;

            if (routePresets.TryGetValue(key, out var preset) && preset != null)
            {
                metadata = CloneMetadata(preset);
            }
            else
            {
                metadata = new RouteMetadata
                {
                    Name = key,
                    Type = type == RouteType.Unknown ? routeLibrary.GuessRouteType(key) : type,
                    IsLoop = type == RouteType.MainLoop,
                    DefaultSpeedLimit = 60
                };
            }

            metadata.Name = routeLibrary.NormalizeRouteName(metadata.Name);
            if (type != RouteType.Unknown) metadata.Type = type;
            if (metadata.Type == RouteType.MainLoop) metadata.IsLoop = true;
            if (attachIndex.HasValue) metadata.AttachMainIndex = attachIndex;
            if (rejoinIndex.HasValue) metadata.RejoinMainIndex = rejoinIndex;
            if (metadata.Type == RouteType.Unknown) metadata.Type = routeLibrary.GuessRouteType(metadata.Name);
            if (!metadata.DefaultSpeedLimit.HasValue) metadata.DefaultSpeedLimit = 60;
            if (string.IsNullOrWhiteSpace(metadata.Track)) metadata.Track = routeLibrary.TrackCode;
            if (string.IsNullOrWhiteSpace(metadata.Layout)) metadata.Layout = routeLibrary.LayoutName;

            return metadata;
        }

        public void ConnectOutGauge(string host, int port)
        {
            try
            {
                outGauge.Connect(host, port);
                logger.Log($"OutGauge connected to {host}:{port}");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Failed to connect OutGauge on {host}:{port}, retrying on any interface");
                try
                {
                    // Bind on all interfaces to avoid VM/host-only IP binding issues
                    outGauge.Connect("0.0.0.0", port);
                    logger.Log($"OutGauge connected on 0.0.0.0:{port}");
                }
                catch (Exception retryEx)
                {
                    logger.LogException(retryEx, "OutGauge retry on 0.0.0.0 failed");
                }
            }
        }

        /// <summary>
        ///     Initialize the debug UI interface
        /// </summary>
        public void InitializeDebugUI()
        {
            if (config.DebugEnabled && debugUI != null && !debugUIInitialized)
            {
                debugUI.Initialize();
                debugUIInitialized = true;
                logger.Log("Debug UI initialized");
                mainUI.Show();
            }
        }

        /// <summary>
        /// Start recording a route with metadata for later editing.
        /// </summary>
        public void StartRecording(string name, RouteType type = RouteType.Unknown, int? attachIndex = null,
            int? rejoinIndex = null)
        {
            var metadata = BuildRecordingMetadata(name, type, attachIndex, rejoinIndex);
            recordingRouteName = metadata.Name;
            RefreshRouteOptions(recordingRouteName);
            routeRecorder.Start(metadata);
        }

        public void UpdateUIForView(byte viewPlid)
        {
            if (!config.DebugEnabled || debugUI == null) return;

            if (aiPLIDs.Contains(viewPlid))
            {
                debugUI.SetAIPLID(viewPlid);
                debugUI.ShowAIButtons(true);
            }
            else
            {
                debugUI.ShowAIButtons(false);
            }
        }

        /// <summary>
        /// Update the record UI selection to highlight the chosen route.
        /// </summary>
        public void SetRecordingRouteSelection(string routeName)
        {
            recordingRouteName = routeLibrary.NormalizeRouteName(
                string.IsNullOrWhiteSpace(routeName) ? "main_loop" : routeName);
            mainUI?.UpdateRecordingRouteSelection(recordingRouteName);
            RefreshRouteOptions(recordingRouteName);
        }

        /// <summary>
        /// Refresh available route selection buttons based on the current track/layout context.
        /// </summary>
        public void RefreshRouteOptions(string preferredSelection = null)
        {
            var selection = string.IsNullOrWhiteSpace(preferredSelection)
                ? recordingRouteName
                : routeLibrary.NormalizeRouteName(preferredSelection);

            var options = new List<string>();
            AddOption(options, config.MainRouteName);
            AddOption(options, config.SpawnRouteName);

            try
            {
                var routes = routeLibrary.ListRoutes();
                foreach (var route in routes)
                {
                    AddOption(options, route?.Metadata?.Name);
                }
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to refresh route options from disk");
            }

            AddOption(options, selection);
            mainUI?.SetRouteOptions(options, selection);
        }

        /// <summary>
        /// Map a UI button click back to the recording route it represents.
        /// </summary>
        public bool TryGetRouteNameForButton(byte clickId, out string routeName)
        {
            if (mainUI == null)
            {
                routeName = string.Empty;
                return false;
            }

            return mainUI.TryGetRouteNameForButton(clickId, out routeName);
        }

        /// <summary>
        /// Add a route name to the provided list if it is valid and not already present.
        /// </summary>
        private static void AddOption(List<string> options, string name)
        {
            if (options == null) return;
            if (string.IsNullOrWhiteSpace(name)) return;
            if (options.Exists(o => o.Equals(name, StringComparison.OrdinalIgnoreCase))) return;
            options.Add(name);
        }

        /// <summary>
        /// Apply the current track and layout context so routes are loaded from the correct folder.
        /// </summary>
        public void SetTrackLayoutContext(string trackCode, string layoutName)
        {
            var track = string.IsNullOrWhiteSpace(trackCode) ? "UnknownTrack" : trackCode.Trim();
            var layout = string.IsNullOrWhiteSpace(layoutName) ? "DefaultLayout" : layoutName.Trim();

            routeLibrary.SetTrackLayout(track, layout);
            waypointManager.ClearRoutes();
            ReloadRoutes(true);
            RefreshRouteOptions(recordingRouteName);
            logger.Log($"Route context set to Track={track}, Layout={layout}");
        }

        /// <summary>
        /// Update spawn delay for AI car creation.
        /// </summary>
        public void SetSpawnDelayMs(int delayMs)
        {
            config.WaitTimeToSpawn = Math.Max(0, delayMs);
        }

        /// <summary>
        /// Configure how many waypoints ahead the AI should look when selecting targets.
        /// </summary>
        public void SetLookaheadWaypoints(int lookahead)
        {
            config.LookaheadWaypoints = Math.Max(1, lookahead);
        }

        /// <summary>
        /// Configure multiplier for waypoint proximity threshold.
        /// </summary>
        public void SetWaypointProximityMultiplier(double multiplier)
        {
            config.WaypointProximityMultiplier = Math.Max(0.1, multiplier);
        }

        /// <summary>
        /// Manually set indicator state for an AI with optional auto-cancel.
        /// </summary>
        public void SetIndicators(byte plid, AILightController.IndicatorState state, TimeSpan? duration = null)
        {
            lightController.SetIndicators(plid, state, duration);
        }

        /// <summary>
        /// Toggle hazard lights for an AI driver.
        /// </summary>
        public void SetHazards(byte plid, bool enabled, TimeSpan? duration = null)
        {
            lightController.SetHazards(plid, enabled, duration);
        }

        /// <summary>
        /// Change the headlight mode (off/side/low/high) for an AI.
        /// </summary>
        public void SetHeadlights(byte plid, AILightController.HeadlightMode mode)
        {
            lightController.SetHeadlights(plid, mode);
        }

        /// <summary>
        /// Flash the high beams on a specific AI for a short burst.
        /// </summary>
        public void FlashHighBeams(byte plid, byte durationHundredths = 15)
        {
            lightController.FlashHighBeams(plid, durationHundredths);
        }

        /// <summary>
        /// Sound the horn on a specific AI car.
        /// </summary>
        public void Honk(byte plid, byte hornTone = 1, byte durationHundredths = 20)
        {
            lightController.Honk(plid, hornTone, durationHundredths);
        }

        /// <summary>
        /// Set the minimum distance in meters between recorded points.
        /// </summary>
        public void SetRecordingInterval(double meters)
        {
            routeRecorder.SetInterval(meters);
        }

        /// <summary>
        /// Apply route names from configuration and reload path data.
        /// </summary>
        public void ApplyRouteConfig(string spawnRoute, string mainRoute, IEnumerable<string> branches)
        {
            if (!string.IsNullOrWhiteSpace(spawnRoute)) config.SpawnRouteName = spawnRoute;
            if (!string.IsNullOrWhiteSpace(mainRoute)) config.MainRouteName = mainRoute;

            if (branches != null)
                config.BranchRouteNames = branches
                    .Where(b => !string.IsNullOrWhiteSpace(b))
                    .ToList();

            pathManager.LoadRoutes(config);
        }

        public void StopRecording()
        {
            routeRecorder.Stop();
        }

        /// <summary>
        /// Reset layout visuals and remove all AI cars.
        /// </summary>
        public void ResetLayoutAndAI()
        {
            lfsLayout.ClearAllVisualizations();
            lfsLayout.WaypointsVisualized = false;
            RemoveAllAICars();
            insim.Send(new IS_MST { Msg = "Layout cleared and all AI removed." });
        }

        /// <summary>
        /// Reset a single AI by spectating it and spawning a new one.
        /// </summary>
        private void ResetAI(byte plid)
        {
            if (!aiPLIDs.Contains(plid))
                return;

            insim.Send(new IS_MST { Msg = $"/spec {plid}" });

            aiPLIDs.Remove(plid);
            aiSpawnPositions.Remove(plid);
            engineStateMap.Remove(plid);
            currentRoute.Remove(plid);
            activeBranchSelections.Remove(plid);

            insim.Send(new IS_MST { Msg = "/ai" });
            logger.Log($"Reset AI {plid} after max recovery attempts; spawned replacement");
            mainUI.UpdateAIList(GetAiTuples());
        }

        /// <summary>
        /// Toggle visualization of recorded routes in the layout editor for the current viewer.
        /// </summary>
        public void ToggleRouteVisualization(byte viewPlid)
        {
            if (viewPlid == 0)
            {
                logger.LogWarning("Cannot visualize routes: no player in view");
                insim.Send(new IS_MST { Msg = "Select a car/view to place layout objects." });
                return;
            }

            if (lfsLayout.WaypointsVisualized)
            {
                lfsLayout.ClearAllVisualizations();
                lfsLayout.WaypointsVisualized = false;
                insim.Send(new IS_MST { Msg = "Layout visualization hidden." });
                return;
            }

            // Always refresh the recorded routes before visualizing so recent recordings appear.
            var routeNames = new List<string>();
            if (!string.IsNullOrWhiteSpace(config.SpawnRouteName)) routeNames.Add(config.SpawnRouteName);
            if (!string.IsNullOrWhiteSpace(config.MainRouteName)) routeNames.Add(config.MainRouteName);
            if (config.BranchRouteNames != null) routeNames.AddRange(config.BranchRouteNames);

            var firstRoute = true;
            foreach (var name in routeNames)
            {
                waypointManager.LoadTrafficRoute(name);
                var recorded = waypointManager.GetRecordedRoute(name);
                if (recorded != null)
                {
                    lfsLayout.VisualizeRecordedRoute(viewPlid, recorded, firstRoute);
                    firstRoute = false;
                }
                else logger.LogWarning($"No recorded route found for visualization: {name}");
            }

            lfsLayout.WaypointsVisualized = true;
            insim.Send(new IS_MST { Msg = $"Visualized {routeNames.Count} recorded routes." });
        }

        public bool IsRecording => routeRecorder.IsRecording;

        /// <summary>
        ///     Get the first AI car's player ID
        /// </summary>
        public byte GetFirstAICarID()
        {
            return aiPLIDs.Count > 0 ? aiPLIDs[0] : (byte)0;
        }

        /// <summary>
        /// Find the index of the waypoint closest to the given car position.
        /// </summary>
        private int FindClosestIndex(List<Util.Waypoint> path, CompCar car)
        {
            if (path == null || path.Count == 0 || car.PLID == 0) return 0;

            var carX = car.X / 65536.0;
            var carY = car.Y / 65536.0;

            var closestIndex = 0;
            var minDistance = double.MaxValue;

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
                }
            }

            return closestIndex;
        }

        /// <summary>
        /// Determine indicator direction for a lane/route change by comparing path vectors.
        /// </summary>
        private AILightController.IndicatorState DetermineIndicatorDirection(
            List<Util.Waypoint> fromPath,
            int fromIndex,
            List<Util.Waypoint> targetPath,
            int targetIndex)
        {
            var fromVector = GetDirectionVector(fromPath, fromIndex);
            var targetVector = GetDirectionVector(targetPath, targetIndex);
            var fromPoint = GetWaypointPosition(fromPath, fromIndex);
            var targetPoint = GetWaypointPosition(targetPath, targetIndex);
            var offsetVector = NormalizeVector(targetPoint.x - fromPoint.x, targetPoint.y - fromPoint.y);

            // Prefer lateral offset to decide side; fall back to heading difference.
            var sideScore = Cross(fromVector, offsetVector);
            if (Math.Abs(sideScore) < 0.01) sideScore = Cross(fromVector, targetVector);

            if (Math.Abs(sideScore) < 0.01) return AILightController.IndicatorState.Hazard;

            return sideScore > 0 ? AILightController.IndicatorState.Left : AILightController.IndicatorState.Right;
        }

        /// <summary>
        /// Send a timed indicator signal for a lane or branch change with logging.
        /// </summary>
        private void SignalLaneChange(
            byte plid,
            List<Util.Waypoint> fromPath,
            int fromIndex,
            List<Util.Waypoint> targetPath,
            int targetIndex,
            string description)
        {
            var indicator = DetermineIndicatorDirection(fromPath, fromIndex, targetPath, targetIndex);
            lightController.SetIndicators(plid, indicator, TimeSpan.FromSeconds(5));
            logger.Log($"PLID={plid} ROUTE CHANGE: {description} using indicator {indicator}");
        }

        private static (double x, double y) GetDirectionVector(List<Util.Waypoint> path, int index)
        {
            if (path == null || path.Count < 2) return (0, 0);

            var startIndex = Math.Max(0, Math.Min(path.Count - 1, index));
            var endIndex = (startIndex + 1) % path.Count;
            var start = path[startIndex].Position;
            var end = path[endIndex].Position;
            return NormalizeVector(
                (end.X - start.X) / 65536.0,
                (end.Y - start.Y) / 65536.0);
        }

        private static (double x, double y) GetWaypointPosition(List<Util.Waypoint> path, int index)
        {
            if (path == null || path.Count == 0) return (0, 0);
            var clamped = Math.Max(0, Math.Min(path.Count - 1, index));
            var pos = path[clamped].Position;
            return (pos.X / 65536.0, pos.Y / 65536.0);
        }

        private static (double x, double y) NormalizeVector(double dx, double dy)
        {
            var magnitude = Math.Sqrt(dx * dx + dy * dy);
            if (magnitude < 0.0001) return (0, 0);
            return (dx / magnitude, dy / magnitude);
        }

        private static double Cross((double x, double y) a, (double x, double y) b)
        {
            return a.x * b.y - a.y * b.x;
        }

        public void SetNumberOfAIs(int count)
        {
            config.NumberOfAIs = Math.Max(0, count);
        }

        public void ShowAddAIDialog()
        {
            // Input field is now always visible; no-op to keep call sites safe.
        }

        public void HideAddAIDialog()
        {
            // No separate dialog to hide; kept for compatibility.
        }

        /// <summary>
        ///     Spawn AI cars in the game
        /// </summary>
        public void SpawnAICars()
        {
            for (var i = 0; i < config.NumberOfAIs; i++)
            {
                insim.Send(new IS_MST { Msg = "/ai" });
                logger.Log($"Spawned AI {i + 1} of {config.NumberOfAIs}");
                if (i < config.NumberOfAIs - 1)
                    Thread.Sleep(config.WaitTimeToSpawn);
            }
        }

        public void RemoveLastAICar()
        {
            if (aiPLIDs.Count == 0) return;
            var plid = aiPLIDs[aiPLIDs.Count - 1];
            insim.Send(new IS_MST { Msg = $"/spec {plid}" });
            aiPLIDs.Remove(plid);
            aiSpawnPositions.Remove(plid);
            engineStateMap.Remove(plid);
            currentRoute.Remove(plid);
            activeBranchSelections.Remove(plid);
            mainUI.UpdateAIList(GetAiTuples());
        }

        public void RemoveAICar(byte plid)
        {
            if (!aiPLIDs.Contains(plid) || plid == 0) return;
            insim.Send(new IS_MST { Msg = $"/spec {plid}" });
            aiPLIDs.Remove(plid);
            aiSpawnPositions.Remove(plid);
            engineStateMap.Remove(plid);
            currentRoute.Remove(plid);
            activeBranchSelections.Remove(plid);
            mainUI.UpdateAIList(GetAiTuples());
        }

        /// <summary>
        /// Try to remove an AI based on a remove-button ClickID from the UI.
        /// </summary>
        public bool TryRemoveAiFromButton(byte clickId)
        {
            if (mainUI.TryGetAiForRemoveButton(clickId, out var plid))
            {
                RemoveAICar(plid);
                return true;
            }

            return false;
        }

        public void RemoveAllAICars()
        {
            foreach (var plid in aiPLIDs.Where(p => p > 0))
            {
                insim.Send(new IS_MST { Msg = $"/spec {plid}" });
            }
            aiPLIDs.Clear();
            aiSpawnPositions.Clear();
            engineStateMap.Clear();
            currentRoute.Clear();
            activeBranchSelections.Clear();
            mainUI.UpdateAIList(GetAiTuples());
        }

        public void StopAllAIs()
        {
            foreach (var plid in aiPLIDs.Where(p => p > 0))
            {
                waypointFollower.SetManualTargetSpeed(plid, 0);
                driver.ParkCar(plid);
            }
        }

        /// <summary>
        /// Turn on ignition, release brakes, and resume path following for all AIs.
        /// </summary>
        public void StartAllAIs()
        {
            foreach (var plid in aiPLIDs.Where(p => p > 0))
            {
                waypointFollower.ClearManualTargetSpeed(plid);
                driver.StartCar(plid);
            }
        }

        public void SpectateAllAIs()
        {
            foreach (var plid in aiPLIDs)
            {
                insim.Send(new IS_MST { Msg = $"/spec {plid}" });
            }
        }

        /// <summary>
        /// Send all AI cars to the pits.
        /// </summary>
        public void PitAllAIs()
        {
            insim.Send(new IS_MST { Msg = "/pit_all" });
        }

        public void SetTargetSpeedForAll(double speed)
        {
            foreach (var plid in aiPLIDs)
            {
                waypointFollower.SetManualTargetSpeed(plid, speed);
            }
        }

        /// <summary>
        /// Request a lane change onto a specific recorded branch route and signal accordingly.
        /// </summary>
        public bool TrySwitchToBranch(byte plid, string branchName)
        {
            if (!aiPLIDs.Contains(plid)) return false;
            if (!pathManager.TryGetBranchByName(branchName, out var branchInfo))
            {
                logger.LogWarning($"No branch named {branchName} available for lane change.");
                return false;
            }

            var car = Array.Find(allCars, c => c.PLID == plid);
            if (car.PLID == 0)
            {
                logger.LogWarning($"Cannot switch AI {plid} to {branchName}: no MCI data.");
                return false;
            }

            SignalLaneChange(
                plid,
                pathManager.MainRoute,
                branchInfo.StartIndex,
                branchInfo.Path,
                0,
                $"Manual lane change to {branchName}");

            var bestIndex = FindClosestIndex(branchInfo.Path, car);
            waypointFollower.SetPath(plid, branchInfo.Path, bestIndex);
            currentRoute[plid] = "branch";
            activeBranchSelections[plid] = branchInfo;
            return true;
        }

        /// <summary>
        ///     Teleport AI to insim circle 10
        /// </summary>
        public async void TeleportAICars(byte aiPlid)
        {
            var circle = lfsLayout.layoutObjects.FirstOrDefault(o => o.Index == 253 && o.Heading == 10);
            if (circle == null)
            {
                logger.LogError("InSim Circle ID 10 not found.");
                return;
            }

            while (IsCircleOccupied(circle, allCars))
            {
                logger.Log($"Circle ID 10 is occupied, waiting to teleport AI {aiPlid}...");
                await Task.Delay(500); // wait and recheck
            }

            // Proceed to teleport
            var x = circle.X;
            var y = circle.Y;
            var z = circle.Zbyte;
            var heading = circle.Heading;

            insim.Send(new IS_JRR
            {
                ReqI = 0,
                JRRAction = JrrAction.JRR_RESET_NO_REPAIR,
                PLID = aiPlid,
                UCID = 0,
                StartPos = new ObjectInfo
                {
                    X = x,
                    Y = y,
                    Zbyte = z,
                    Heading = heading,
                    Flags = 0x80,
                    Index = 0
                }
            });

            logger.Log($"Teleported AI {aiPlid} to InSim Circle ID 10 at ({x / 16.0f}, {y / 16.0f})m");
        }

        /// <summary>
        ///     Check if the teleport location is full
        /// </summary>
        private bool IsCircleOccupied(ObjectInfo circle, CompCar[] allCars)
        {
            var circlePos = new Vec(circle.X, circle.Y);
            foreach (var car in allCars)
            {
                var carPos = new Vec(car.X, car.Y);
                if (Vec.Distance(circlePos, carPos) < 2.0) // 2 meters radius
                    return true;
            }

            return false;
        }

        /// <summary>
        ///     Handle new player events
        /// </summary>
        public void OnNewPlayer(IS_NPL npl)
        {
            if ((npl.PType & PlayerTypes.PLT_AI) == PlayerTypes.PLT_AI &&
                !npl.PName.Contains("Track")) // and not tracks
            {
                var plid = npl.PLID;

                if (!aiPLIDs.Contains(plid))
                {
                    // Add to our list of AIs
                    aiPLIDs.Add(plid);
                    aiSpawnPositions[plid] = new Vec();
                    engineStateMap[plid] = true; // Assume engine starts running

                    // Initialize the driver
                    driver.InitializeDriver(plid);

                    // Request AI info
                    RequestAIInfo(plid);

                    logger.Log($"New AI player spawned: PLID={plid}, Type={npl.PType}, Name={npl.PName}");

                    // don't teleport ai anymore
                    //TeleportAICars(plid);
                    //Thread.Sleep(1000);

                    // Assign spawn path (start at closest point once MCI data is available)
                    waypointFollower.SetPath(plid, pathManager.SpawnRoute);
                    currentRoute[plid] = "spawn";

                    // Set debug UI to track this AI if enabled
                    if (config.DebugEnabled && debugUI != null && debugUIInitialized)
                        debugUI.SetAIPLID(plid);

                    mainUI.UpdateAIList(GetAiTuples());
                }
            }
            else if (config.DebugEnabled && debugUI != null && debugUIInitialized)
            {
                // Track human player in debug UI
                debugUI.SetPlayerPLID(npl.PLID);
                playerPLID = npl.PLID;
                logger.Log($"Human player detected: PLID={npl.PLID}, Name={npl.PName}");
            }
        }

        /// <summary>
        ///     Request AI information from the game
        /// </summary>
        private void RequestAIInfo(byte plid)
        {
            var inputs = new List<AIInputVal>
            {
                new AIInputVal { Input = AicInputType.CS_REPEAT_AI_INFO, Time = 10, Value = 0 }
            };

            insim.Send(new IS_AIC(inputs) { PLID = plid });
        }

        /// <summary>
        ///     Handle MCI packet (car telemetry)
        /// </summary>
        public void OnMCI(IS_MCI mci)
        {
            allCars = mci.Info.ToArray();

            // Record player route if active
            if (playerPLID != 0)
            {
                var car = Array.Find(allCars, c => c.PLID == playerPLID);
                if (car.PLID != 0)
                {
                    var pos = new Vec(car.X, car.Y, car.Z);
                    var speed = 360.0 * car.Speed / 32768.0;
                    var angleDiff = (car.Heading - car.Direction) * 360f / 65536f;
                    routeRecorder.AddPoint(pos, speed, playerThrottle, playerBrake, angleDiff, car.Heading, playerPLID);
                }
            }

            // Update debug UI
            if (config.DebugEnabled && debugUI != null && debugUIInitialized)
            {
                // Collect waypoint and control info for the debug display
                var waypointIndices = new Dictionary<byte, int>();
                var targetSpeeds = new Dictionary<byte, double>();
                var controlInfo = new Dictionary<byte, string>();
                var aiStates = new Dictionary<byte, string>();

                foreach (var plid in aiPLIDs)
                {
                    var (targetIndex, _, targetSpeed, inRecovery) = waypointFollower.GetFollowerInfo(plid);
                    waypointIndices[plid] = targetIndex;
                    targetSpeeds[plid] = targetSpeed;
                    controlInfo[plid] = driver.GetControlInfo(plid);
                    var state = driver.GetStateDescription(plid);
                    aiStates[plid] = BuildAiStateLabel(plid, state, inRecovery);

                    // Place active waypoint marker
                    var activeWaypoint = waypointFollower.GetLookaheadWaypoint(plid);
                    if (activeWaypoint.Position != null)
                        lfsLayout.VisualizeActiveWaypoint(plid, activeWaypoint);
                }

                // Get paths for all AIs
                var paths = new Dictionary<byte, List<Util.Waypoint>>();
                foreach (var plid in aiPLIDs) paths[plid] = waypointFollower.GetPath(plid);

                debugUI.UpdateDebugInfo(allCars, waypointIndices, paths, targetSpeeds, controlInfo, aiStates);
            }
        }

        /// <summary>
        /// Build a short label describing the AI's current driving state and route.
        /// </summary>
        private string BuildAiStateLabel(byte plid, string driverState, bool inRecovery)
        {
            var state = string.IsNullOrWhiteSpace(driverState) ? "Driving" : driverState;

            if (inRecovery && state.Equals("Driving", StringComparison.OrdinalIgnoreCase))
                state = "Waypoint Rec";

            var routeLabel = GetRouteLabel(plid);
            if (!string.IsNullOrWhiteSpace(routeLabel))
            {
                if (state.Equals("Driving", StringComparison.OrdinalIgnoreCase))
                    return $"Driving ({routeLabel})";

                return $"{state} ({routeLabel})";
            }

            return state;
        }

        /// <summary>
        /// Summarize the active route for a specific AI as a compact label.
        /// </summary>
        private string GetRouteLabel(byte plid)
        {
            if (!currentRoute.TryGetValue(plid, out var routeName))
                return string.Empty;

            if (routeName.Equals("spawn", StringComparison.OrdinalIgnoreCase))
                return "Spawn";

            if (routeName.Equals("main", StringComparison.OrdinalIgnoreCase))
                return "Main";

            if (routeName.Equals("branch", StringComparison.OrdinalIgnoreCase))
            {
                if (activeBranchSelections.TryGetValue(plid, out var branch) &&
                    !string.IsNullOrWhiteSpace(branch?.Name))
                    return $"Branch {branch.Name}";
                return "Branch";
            }

            return routeName;
        }

        /// <summary>
        ///     Get the paths for all active AI cars
        /// </summary>
        /// <returns>Dictionary of paths by PLID</returns>
        public Dictionary<byte, List<Util.Waypoint>> GetAIPaths()
        {
            var result = new Dictionary<byte, List<Util.Waypoint>>();

            foreach (var plid in aiPLIDs)
            {
                var path = waypointFollower.GetPath(plid);
                if (path != null && path.Count > 0) result[plid] = path;
            }

            return result;
        }

        /// <summary>
        ///     Handle AII packet (AI info)
        /// </summary>
        public void OnAII(IS_AII aii)
        {
            var plid = aii.PLID;
            if (!aiPLIDs.Contains(plid)) return;

            // Check engine state - stalled if ignition is on but RPM is near zero
            var ignitionOn = (aii.Flags & AIFlags.AIFLAGS_IGNITION) != 0;
            var engineStalled = ignitionOn && aii.RPM < 0.1f;
            var engineRunning = ignitionOn && !engineStalled;

            // Update engine state in driver
            driver.UpdateEngineState(plid, engineRunning);

            // Find car data in MCI
            var car = Array.Find(allCars, c => c.PLID == plid);
            if (car.PLID == 0)
            {
                logger.Log($"PLID={plid} No MCI data available yet, skipping control update");
                return;
            }

            // Update AI controls based on current state
            driver.UpdateControls(plid, car, allCars, waypointManager, config, lfsLayout.layoutObjects);

            // Route management
            var (targetIndex, count, _, _) = waypointFollower.GetFollowerInfo(plid);
            if (currentRoute.TryGetValue(plid, out var routeName))
            {
                if (routeName == "spawn" && targetIndex >= pathManager.SpawnRoute.Count - 1)
                {
                    var bestIndex = FindClosestIndex(pathManager.MainRoute, car);
                    waypointFollower.SetPath(plid, pathManager.MainRoute, bestIndex);
                    currentRoute[plid] = "main";
                }
                else if (routeName == "main")
                {
                    if (pathManager.TryGetBranch(targetIndex, out var branchInfo) && branchRandom.NextDouble() < 0.5)
                    {
                        SignalLaneChange(
                            plid,
                            pathManager.MainRoute,
                            targetIndex,
                            branchInfo.Path,
                            0,
                            $"Taking branch {branchInfo.Name}");

                        var bestIndex = FindClosestIndex(branchInfo.Path, car);
                        waypointFollower.SetPath(plid, branchInfo.Path, bestIndex);
                        currentRoute[plid] = "branch";
                        activeBranchSelections[plid] = branchInfo;
                    }
                }
                else if (routeName == "branch")
                {
                    var path = waypointFollower.GetPath(plid);
                    if (path == null || path.Count == 0)
                    {
                        logger.LogWarning($"PLID={plid} has an empty branch path, switching back to main.");
                        waypointFollower.SetPath(plid, pathManager.MainRoute);
                        activeBranchSelections.Remove(plid);
                        currentRoute[plid] = "main";
                        return;
                    }

                    if (targetIndex >= path.Count - 1)
                    {
                        var bestIndex = FindClosestIndex(pathManager.MainRoute, car);

                        if (activeBranchSelections.TryGetValue(plid, out var branchInfo))
                        {
                            var hintedIndex = pathManager.MainRoute.Count == 0
                                ? 0
                                : Math.Max(
                                    0,
                                    Math.Min(pathManager.MainRoute.Count - 1, branchInfo.RejoinIndex));
                            bestIndex = hintedIndex;

                            var approachIndex = Math.Max(0, path.Count - 2);
                            SignalLaneChange(
                                plid,
                                path,
                                approachIndex,
                                pathManager.MainRoute,
                                bestIndex,
                                $"Rejoining main from {branchInfo.Name}");

                            activeBranchSelections.Remove(plid);
                        }

                        waypointFollower.SetPath(plid, pathManager.MainRoute, bestIndex);
                        currentRoute[plid] = "main";
                    }
                }
            }
        }

        private void OnOutGauge(object sender, OutGaugeEventArgs e)
        {
            if (e.PLID == playerPLID)
            {
                playerThrottle = e.Throttle;
                playerBrake = e.Brake;
            }
        }

        private IEnumerable<(byte id, string name)> GetAiTuples()
        {
            foreach (var plid in aiPLIDs)
            {
                yield return (plid, $"AI {plid}");
            }
        }

        /// <summary>
        /// Reload all route files from disk and reapply paths to active AIs.
        /// </summary>
        public void ReloadRoutes(bool silent = false)
        {
            waypointManager.ClearRoutes();
            pathManager.LoadRoutes(config);

            foreach (var plid in aiPLIDs)
            {
                if (!currentRoute.TryGetValue(plid, out var routeName))
                    continue;

                switch (routeName)
                {
                    case "spawn":
                        waypointFollower.SetPath(plid, pathManager.SpawnRoute);
                        break;
                    case "main":
                        waypointFollower.SetPath(plid, pathManager.MainRoute);
                        break;
                    case "branch":
                        waypointFollower.SetPath(plid, pathManager.MainRoute);
                        currentRoute[plid] = "main";
                        break;
                    default:
                        waypointFollower.SetPath(plid, pathManager.MainRoute);
                        currentRoute[plid] = "main";
                        break;
                }
            }

            if (!silent)
            {
                insim.Send(new IS_MST { Msg = "Routes reloaded from disk" });
            }

            RefreshRouteOptions(recordingRouteName);
            logger.Log("Routes reloaded and reapplied to active AIs");
        }

        /// <summary>
        /// Visualize a recorded route in the layout so it can be edited in LFS.
        /// </summary>
        public void VisualizeRouteForEditing(string routeName, byte plid)
        {
            var recorded = waypointManager.GetRecordedRoute(routeName);
            if (recorded == null)
            {
                logger.LogWarning($"Cannot visualize route {routeName}: no recorded data found");
                return;
            }

            lfsLayout.VisualizeRecordedRoute(plid, recorded);
        }
    }
}
