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
        private const double InnerBranchJoinChance = 0.25;
        private const double InnerBranchMaxDistanceMeters = 12.0;
        private const double InnerBranchMaxHeadingDegrees = 60.0;
        private const double InnerBranchMaxSpeedKmh = 55.0;
        private const double InnerBranchTargetSpeedKmh = 45.0;
        private static readonly TimeSpan InnerBranchCheckCooldown = TimeSpan.FromSeconds(12);
        private const double InnerLaneSwapChance = 0.35;
        private static readonly TimeSpan InnerLaneSwapCooldown = TimeSpan.FromSeconds(8);
        private readonly Dictionary<byte, DateTime> innerBranchLastCheck = new Dictionary<byte, DateTime>();
        private readonly HashSet<byte> innerBranchSpeedHold = new HashSet<byte>();
        private readonly Dictionary<byte, DateTime> innerLaneSwapLast = new Dictionary<byte, DateTime>();
        private readonly SemaphoreSlim spawnSemaphore = new SemaphoreSlim(1, 1);
        private readonly SemaphoreSlim visualizationSemaphore = new SemaphoreSlim(1, 1);
        private readonly OutGauge outGauge;
        private string recordingRouteName = "main_loop";
        private string visualizationRouteName = "main_loop";
        private int visualizationDetailStep = 2;
        private static readonly int[] VisualizationDetailSteps = { 1, 2, 4, 8, 16 };
        private int? selectedRouteNodeIndex;

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

            routePresets["main_alt"] = new RouteMetadata
            {
                Name = "main_alt",
                Type = RouteType.AlternateMain,
                Description = "Alternate lane for the main loop (e.g., highway inner lane).",
                IsLoop = true,
                DefaultSpeedLimit = 60
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
                mainUI.UpdateVisualizationDetail(visualizationDetailStep);
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

            var visualizationOptions = new List<string>();
            try
            {
                var routes = routeLibrary.ListRoutes();
                foreach (var route in routes)
                {
                    AddOption(visualizationOptions, route?.Metadata?.Name);
                }
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to refresh visualization routes from disk");
            }

            if (visualizationOptions.Count > 0 &&
                !visualizationOptions.Exists(r => r.Equals(visualizationRouteName, StringComparison.OrdinalIgnoreCase)))
                visualizationRouteName = visualizationOptions[0];

            mainUI?.SetVisualizationRouteOptions(visualizationOptions, visualizationRouteName);
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
        /// Map a UI button click back to the visualization route it represents.
        /// </summary>
        public bool TryGetVisualizationRouteNameForButton(byte clickId, out string routeName)
        {
            if (mainUI == null)
            {
                routeName = string.Empty;
                return false;
            }

            return mainUI.TryGetVisualizationRouteNameForButton(clickId, out routeName);
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
        /// Configure a heading error deadzone in degrees to ignore small steering corrections.
        /// </summary>
        public void SetSteeringDeadzoneDegrees(double deadzoneDegrees)
        {
            config.SteeringDeadzoneDegrees = Math.Max(0.0, deadzoneDegrees);
        }

        /// <summary>
        /// Adjust steering response damping to tune how aggressively heading errors are corrected.
        /// </summary>
        public void SetSteeringResponseDamping(double damping)
        {
            config.SteeringResponseDamping = Math.Max(0.1, damping);
        }

        /// <summary>
        /// Configure pure pursuit steering to smooth path following while keeping it tunable.
        /// </summary>
        public void ConfigurePurePursuit(
            bool enabled,
            double minLookahead,
            double maxLookahead,
            double speedFactor,
            double wheelbaseMeters,
            double steeringGain,
            double maxSteerDegrees)
        {
            config.UsePurePursuitSteering = enabled;
            config.PurePursuitLookaheadMinMeters = Math.Max(1.0, minLookahead);
            config.PurePursuitLookaheadMaxMeters =
                Math.Max(config.PurePursuitLookaheadMinMeters, maxLookahead);
            config.PurePursuitLookaheadSpeedFactor = Math.Max(0.0, speedFactor);
            config.PurePursuitWheelbaseMeters = Math.Max(0.5, wheelbaseMeters);
            config.PurePursuitSteeringGain = Math.Max(0.01, steeringGain);
            config.PurePursuitMaxSteerDegrees = Math.Max(1.0, maxSteerDegrees);
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
        /// <summary>
        /// Apply route names from configuration and reload path data.
        /// </summary>
        public void ApplyRouteConfig(string spawnRoute, string mainRoute, string alternateMainRoute,
            IEnumerable<string> branches)
        {
            if (!string.IsNullOrWhiteSpace(spawnRoute)) config.SpawnRouteName = spawnRoute;
            if (!string.IsNullOrWhiteSpace(mainRoute)) config.MainRouteName = mainRoute;
            if (!string.IsNullOrWhiteSpace(alternateMainRoute)) config.MainAlternateRouteName = alternateMainRoute;

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
        /// Reset layout visuals.
        /// </summary>
        public void ResetLayout()
        {
            lfsLayout.ClearAllVisualizations();
            lfsLayout.WaypointsVisualized = false;
            insim.Send(new IS_MST { Msg = "Layout cleared." });
        }

        /// <summary>
        /// Reset a single AI by spectating it and spawning a new one.
        /// </summary>
        private void ResetAI(byte plid)
        {
            if (!aiPLIDs.Contains(plid))
                return;

            insim.Send(new IS_MST { Msg = $"/pit {plid}" });
            insim.Send(new IS_MST { Msg = $"/spec {plid}" });

            aiPLIDs.Remove(plid);
            aiSpawnPositions.Remove(plid);
            engineStateMap.Remove(plid);
            currentRoute.Remove(plid);
            activeBranchSelections.Remove(plid);

            insim.Send(new IS_MST { Msg = "/ai" });
            logger.Log($"Reset AI {plid} after failed recovery; sent to pits/spectate and spawned replacement");
            mainUI.UpdateAIList(GetAiTuples());
        }

        /// <summary>
        /// Toggle visualization of recorded routes in the layout editor for the current viewer.
        /// </summary>
        /// <summary>
        /// Toggle the visualization of recorded routes for the current view, with a fallback to a default player.
        /// </summary>
        public void ToggleRouteVisualization(byte viewPlid)
        {
            var effectivePlid = viewPlid == 0 ? GetDefaultVisualizationPlid() : viewPlid;
            if (effectivePlid == 0)
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

            VisualizeSelectedRoute(effectivePlid);
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
        /// Pick a fallback PLID for visualization when no camera view is selected.
        /// </summary>
        private byte GetDefaultVisualizationPlid()
        {
            if (playerPLID != 0) return playerPLID;
            return GetFirstAICarID();
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
        /// Check if a branch represents an inner-lane route based on its name.
        /// </summary>
        private static bool IsInnerBranch(BranchRouteInfo branchInfo)
        {
            var name = branchInfo?.Name ?? string.Empty;
            if (branchInfo?.Metadata?.Type == RouteType.AlternateMain) return true;
            return name.IndexOf("inner", StringComparison.OrdinalIgnoreCase) >= 0 ||
                   name.IndexOf("alt", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        /// <summary>
        /// Pick a waypoint index that is near the car and roughly aligned with its heading.
        /// </summary>
        private (int index, double distance, double headingErrorDeg) FindAlignedWaypointIndex(
            List<Util.Waypoint> path,
            CompCar car,
            int preferredIndex,
            int searchRadius = 5)
        {
            if (path == null || path.Count == 0 || car.PLID == 0) return (0, double.MaxValue, 180);

            var carX = car.X / 65536.0;
            var carY = car.Y / 65536.0;
            var carHeading = CoordinateUtils.NormalizeHeading((int)car.Heading);

            var maxOffset = Math.Min(searchRadius, path.Count - 1);
            var startIndex = Math.Max(0, preferredIndex - maxOffset);
            var endIndex = Math.Min(path.Count - 1, preferredIndex + maxOffset);

            if (startIndex > endIndex)
            {
                startIndex = 0;
                endIndex = path.Count - 1;
            }

            var bestScore = double.MaxValue;
            var bestIndex = preferredIndex;
            var bestDistance = double.MaxValue;
            var bestHeadingError = 180.0;

            for (var i = startIndex; i <= endIndex; i++)
            {
                var wpX = path[i].Position.X / 65536.0;
                var wpY = path[i].Position.Y / 65536.0;
                var dx = wpX - carX;
                var dy = wpY - carY;
                var distance = Math.Sqrt(dx * dx + dy * dy);
                var desiredHeading = CoordinateUtils.CalculateHeadingToTarget(dx, dy);
                var headingError = Math.Abs(CoordinateUtils.CalculateHeadingError(carHeading, desiredHeading));
                var headingErrorDeg = Math.Abs(CoordinateUtils.HeadingToDegrees(headingError));
                if (headingErrorDeg > 180) headingErrorDeg = 360 - headingErrorDeg;

                var distanceScore = distance;
                var headingScore = headingErrorDeg * 0.1;
                var backwardPenalty = headingErrorDeg > 135 ? 50.0 : 0.0;
                var indexPenalty = Math.Abs(i - preferredIndex) * 0.5;
                var score = distanceScore + headingScore + backwardPenalty + indexPenalty;

                if (score < bestScore)
                {
                    bestScore = score;
                    bestIndex = i;
                    bestDistance = distance;
                    bestHeadingError = headingErrorDeg;
                }
            }

            return (bestIndex, bestDistance, bestHeadingError);
        }

        /// <summary>
        /// Opportunistically merge onto an inner-lane branch when nearby instead of forcing the switch at the branch start.
        /// </summary>
        private bool TryJoinInnerBranchLaneChange(byte plid, CompCar car, int mainRouteIndex)
        {
            if (!currentRoute.TryGetValue(plid, out var routeName) || routeName != "main")
                return false;

            if (innerBranchLastCheck.TryGetValue(plid, out var lastCheck) &&
                DateTime.Now - lastCheck < InnerBranchCheckCooldown)
                return false;

            var innerBranch = pathManager.GetAlternateMainRoute() ??
                              pathManager.GetBranches().FirstOrDefault(IsInnerBranch);
            innerBranchLastCheck[plid] = DateTime.Now;

            if (innerBranch == null || innerBranch.Path == null || innerBranch.Path.Count == 0)
                return false;

            var speedKmh = 360.0 * car.Speed / 32768.0;
            if (speedKmh > InnerBranchMaxSpeedKmh)
                return false;

            var preferredIndex = FindClosestIndex(innerBranch.Path, car);
            var (entryIndex, distance, headingError) = FindAlignedWaypointIndex(
                innerBranch.Path,
                car,
                preferredIndex,
                Math.Max(innerBranch.Path.Count - 1, 10));

            if (distance > InnerBranchMaxDistanceMeters || headingError > InnerBranchMaxHeadingDegrees)
                return false;

            if (branchRandom.NextDouble() > InnerBranchJoinChance)
                return false;

            SignalLaneChange(
                plid,
                pathManager.MainRoute,
                mainRouteIndex,
                innerBranch.Path,
                entryIndex,
                $"Taking branch {innerBranch.Name}");

            waypointFollower.SetPath(plid, innerBranch.Path, entryIndex);
            waypointFollower.SetManualTargetSpeed(plid, InnerBranchTargetSpeedKmh);
            innerBranchSpeedHold.Add(plid);
            currentRoute[plid] = "branch";
            activeBranchSelections[plid] = innerBranch;
            return true;
        }

        /// <summary>
        /// Attempt a lane change from the inner (right) branch back to the main (left) lane with cooldowns.
        /// </summary>
        private bool TrySwapInnerToMainLane(byte plid, CompCar car, int branchTargetIndex, List<Util.Waypoint> branchPath)
        {
            if (branchPath == null || branchPath.Count == 0) return false;

            if (!activeBranchSelections.TryGetValue(plid, out var branchInfo) || !IsInnerBranch(branchInfo))
                return false;

            if (branchTargetIndex >= branchPath.Count - 3)
                return false; // Skip swaps near the natural rejoin

            if (innerLaneSwapLast.TryGetValue(plid, out var lastSwap) &&
                DateTime.Now - lastSwap < InnerLaneSwapCooldown)
                return false;

            var speedKmh = 360.0 * car.Speed / 32768.0;
            if (speedKmh > InnerBranchMaxSpeedKmh)
                return false;

            var preferredIndex = FindClosestIndex(pathManager.MainRoute, car);
            var (mainIndex, distance, headingError) = FindAlignedWaypointIndex(
                pathManager.MainRoute,
                car,
                preferredIndex,
                8);

            if (distance > InnerBranchMaxDistanceMeters || headingError > InnerBranchMaxHeadingDegrees)
                return false;

            if (branchRandom.NextDouble() > InnerLaneSwapChance)
                return false;

            SignalLaneChange(
                plid,
                branchPath,
                branchTargetIndex,
                pathManager.MainRoute,
                mainIndex,
                "Inner lane change to main");

            waypointFollower.SetPath(plid, pathManager.MainRoute, mainIndex);
            if (innerBranchSpeedHold.Contains(plid))
            {
                waypointFollower.ClearManualTargetSpeed(plid);
                innerBranchSpeedHold.Remove(plid);
            }
            activeBranchSelections.Remove(plid);
            currentRoute[plid] = "main";
            innerLaneSwapLast[plid] = DateTime.Now;
            innerBranchLastCheck[plid] = DateTime.Now;
            return true;
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
            if (!spawnSemaphore.Wait(0))
            {
                logger.LogWarning("AI spawn already in progress; ignoring duplicate request.");
                return;
            }

            Task.Run(async () =>
            {
                try
                {
                    await SpawnAICarsAsync().ConfigureAwait(false);
                }
                catch (Exception ex)
                {
                    logger.LogException(ex, "Error while spawning AI cars asynchronously");
                }
                finally
                {
                    spawnSemaphore.Release();
                }
            });
        }

        /// <summary>
        /// Spawn AI cars without blocking the calling thread so InSim events keep flowing.
        /// </summary>
        private async Task SpawnAICarsAsync()
        {
            for (var i = 0; i < config.NumberOfAIs; i++)
            {
                insim.Send(new IS_MST { Msg = "/ai" });
                logger.Log($"Spawned AI {i + 1} of {config.NumberOfAIs}");
                if (i < config.NumberOfAIs - 1)
                    await Task.Delay(config.WaitTimeToSpawn).ConfigureAwait(false);
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

            var (bestIndex, _, _) = FindAlignedWaypointIndex(branchInfo.Path, car, 0, 8);
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
            driver.UpdateControls(plid, car, allCars, waypointManager, config, lfsLayout.layoutObjects, aii.RPM);

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
                    var switchedInner = TryJoinInnerBranchLaneChange(plid, car, targetIndex);

                    if (!switchedInner && pathManager.TryGetBranch(targetIndex, out var branchInfo))
                    {
                        if (IsInnerBranch(branchInfo))
                        {
                            // Skip start-based switching for inner branches; handled opportunistically elsewhere.
                        }
                        else if (branchRandom.NextDouble() < 0.5)
                        {
                            SignalLaneChange(
                                plid,
                                pathManager.MainRoute,
                                targetIndex,
                                branchInfo.Path,
                                0,
                                $"Taking branch {branchInfo.Name}");

                            var (bestIndex, _, _) = FindAlignedWaypointIndex(branchInfo.Path, car, 0, 8);
                            waypointFollower.SetPath(plid, branchInfo.Path, bestIndex);
                            currentRoute[plid] = "branch";
                            activeBranchSelections[plid] = branchInfo;
                        }
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
                        if (innerBranchSpeedHold.Contains(plid))
                        {
                            waypointFollower.ClearManualTargetSpeed(plid);
                            innerBranchSpeedHold.Remove(plid);
                        }
                        currentRoute[plid] = "main";
                        return;
                    }

                    if (activeBranchSelections.TryGetValue(plid, out var branchInfo) && IsInnerBranch(branchInfo))
                    {
                        if (TrySwapInnerToMainLane(plid, car, targetIndex, path))
                            return;
                    }

                    if (targetIndex >= path.Count - 1)
                    {
                        var bestIndex = FindClosestIndex(pathManager.MainRoute, car);

                        if (activeBranchSelections.TryGetValue(plid, out var branchInfoExit))
                        {
                            var hintedIndex = pathManager.MainRoute.Count == 0
                                ? 0
                                : Math.Max(
                                    0,
                                    Math.Min(pathManager.MainRoute.Count - 1, branchInfoExit.RejoinIndex));
                            bestIndex = hintedIndex;

                            var approachIndex = Math.Max(0, path.Count - 2);
                            SignalLaneChange(
                                plid,
                                path,
                                approachIndex,
                                pathManager.MainRoute,
                                bestIndex,
                                $"Rejoining main from {branchInfoExit.Name}");

                            activeBranchSelections.Remove(plid);
                        }

                        waypointFollower.SetPath(plid, pathManager.MainRoute, bestIndex);
                        if (innerBranchSpeedHold.Contains(plid))
                        {
                            waypointFollower.ClearManualTargetSpeed(plid);
                            innerBranchSpeedHold.Remove(plid);
                        }
                        innerBranchLastCheck[plid] = DateTime.Now;
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
            NotifyRouteValidationIssues();
            logger.Log("Routes reloaded and reapplied to active AIs");
        }

        /// <summary>
        /// Inform the connected user about detected route validation issues.
        /// </summary>
        public void NotifyRouteValidationIssues()
        {
            var issues = pathManager.ValidationIssues;
            if (issues == null || issues.Count == 0) return;

            SendIssueSummary("Route errors", RouteValidationSeverity.Error, issues);
            SendIssueSummary("Route warnings", RouteValidationSeverity.Warning, issues);
        }

        /// <summary>
        /// Send a compact summary of issues through InSim chat.
        /// </summary>
        private void SendIssueSummary(string label, RouteValidationSeverity severity,
            IReadOnlyCollection<RouteValidationIssue> issues)
        {
            var filtered = issues.Where(i => i.Severity == severity).ToList();
            if (filtered.Count == 0) return;

            var summary = BuildIssueSummary(filtered);
            insim.Send(new IS_MST { Msg = $"{label}: {summary}" });
        }

        /// <summary>
        /// Build a concise string for validation issues, trimming to chat-friendly length.
        /// </summary>
        private static string BuildIssueSummary(IReadOnlyCollection<RouteValidationIssue> issues)
        {
            if (issues == null || issues.Count == 0) return string.Empty;

            const int maxLength = 120;
            var parts = issues.Take(3).Select(i => i.Message).ToList();
            var summary = string.Join(" | ", parts);
            if (issues.Count > parts.Count) summary += $" (+{issues.Count - parts.Count} more)";
            if (summary.Length > maxLength) summary = summary.Substring(0, maxLength - 3) + "...";
            return summary;
        }

        /// <summary>
        /// Queue a visualization job so layout work runs off the InSim event thread.
        /// </summary>
        private void EnqueueVisualization(Action work)
        {
            Task.Run(async () =>
            {
                await visualizationSemaphore.WaitAsync().ConfigureAwait(false);
                try
                {
                    work();
                }
                catch (Exception ex)
                {
                    logger.LogException(ex, "Visualization task failed");
                }
                finally
                {
                    visualizationSemaphore.Release();
                }
            });
        }

        /// <summary>
        /// Visualize a recorded route in the layout so it can be edited in LFS.
        /// </summary>
        public void VisualizeRouteForEditing(string routeName, byte plid)
        {
            EnqueueVisualization(() => VisualizeRouteForEditingInternal(routeName, plid));
        }

        /// <summary>
        /// Render the requested route for editing after acquiring the visualization lock.
        /// </summary>
        private void VisualizeRouteForEditingInternal(string routeName, byte plid)
        {
            var recorded = waypointManager.GetRecordedRoute(routeName);
            if (recorded == null)
            {
                logger.LogWarning($"Cannot visualize route {routeName}: no recorded data found");
                return;
            }

            var detailStep = GetVisualizationStep(recorded.Nodes?.Count ?? 0, out _);
            lfsLayout.VisualizeRecordedRoute(plid, recorded, detailStep);
        }

        /// <summary>
        /// Update the selected visualization route name and refresh UI selection.
        /// </summary>
        public void SetVisualizationRouteSelection(string routeName)
        {
            visualizationRouteName = routeLibrary.NormalizeRouteName(
                string.IsNullOrWhiteSpace(routeName) ? "main_loop" : routeName);
            mainUI?.UpdateVisualizationRouteSelection(visualizationRouteName);
            ClearLayoutSelection();
        }

        /// <summary>
        /// Handle layout editor selection changes to map layout objects back to route nodes.
        /// </summary>
        public void OnLayoutSelectionChanged(ObjectInfo selectedObject)
        {
            if (selectedObject == null)
            {
                ClearLayoutSelection();
                return;
            }

            var route = GetEditableRoute();
            if (route == null || route.Nodes == null || route.Nodes.Count == 0)
            {
                ClearLayoutSelection();
                insim.Send(new IS_MST { Msg = "No recorded route loaded for layout selection." });
                return;
            }

            var index = FindClosestRouteNodeIndex(route, selectedObject, out var distanceMeters);
            selectedRouteNodeIndex = index;

            var node = route.Nodes[index];
            var speed = node.SpeedLimit ?? node.Speed;
            var status = $"Node {index} @ {speed:F0} km/h";
            mainUI?.UpdateLayoutSelectionStatus(status);
            insim.Send(new IS_MST
            {
                Msg = $"Selected node {index} ({distanceMeters:F1}m) on {route.Metadata.Name}."
            });
        }

        /// <summary>
        /// Persist moved layout object coordinates back into the selected route node.
        /// </summary>
        public void OnLayoutObjectMoved(ObjectInfo movedObject)
        {
            if (movedObject == null) return;

            var route = GetEditableRoute();
            if (!TryGetSelectedNodeIndex(route, out var index)) return;

            route.Nodes[index].X = movedObject.X / 16.0;
            route.Nodes[index].Y = movedObject.Y / 16.0;
            route.Nodes[index].Z = movedObject.Zbyte / 4.0;
            routeLibrary.Save(route);

            var status = $"Node {index} moved to ({route.Nodes[index].X:F1}, {route.Nodes[index].Y:F1})";
            mainUI?.UpdateLayoutSelectionStatus(status);
            insim.Send(new IS_MST
            {
                Msg = $"{route.Metadata.Name}: saved moved node {index}."
            });
        }

        /// <summary>
        /// Update the speed limit on the currently selected node in the layout editor.
        /// </summary>
        public void UpdateSelectedNodeSpeed(double speedKmh)
        {
            var route = GetEditableRoute();
            if (!TryGetSelectedNodeIndex(route, out var index)) return;

            var clamped = Math.Max(0, speedKmh);
            route.Nodes[index].SpeedLimit = clamped;
            routeLibrary.Save(route);

            mainUI?.UpdateLayoutSelectionStatus($"Node {index} @ {clamped:F0} km/h");
            insim.Send(new IS_MST { Msg = $"Set node {index} speed to {clamped:F0} km/h." });
        }

        /// <summary>
        /// Assign the selected node as the attach-to-main index for the route metadata.
        /// </summary>
        public void SetSelectedNodeAsAttachIndex()
        {
            ApplySelectedNodeMetadata((route, index) => route.Metadata.AttachMainIndex = index, "attach");
        }

        /// <summary>
        /// Assign the selected node as the rejoin-to-main index for the route metadata.
        /// </summary>
        public void SetSelectedNodeAsRejoinIndex()
        {
            ApplySelectedNodeMetadata((route, index) => route.Metadata.RejoinMainIndex = index, "rejoin");
        }

        /// <summary>
        /// Clear layout selection state and reset UI label.
        /// </summary>
        private void ClearLayoutSelection()
        {
            selectedRouteNodeIndex = null;
            mainUI?.UpdateLayoutSelectionStatus("No node selected");
        }

        /// <summary>
        /// Apply a metadata update to the selected node and persist to disk.
        /// </summary>
        private void ApplySelectedNodeMetadata(Action<RecordedRoute, int> updater, string label)
        {
            var route = GetEditableRoute();
            if (!TryGetSelectedNodeIndex(route, out var index)) return;

            updater(route, index);
            routeLibrary.Save(route);
            insim.Send(new IS_MST { Msg = $"Set {label} node to {index} for {route.Metadata.Name}." });
        }

        /// <summary>
        /// Get the recorded route currently selected for visualization and editing.
        /// </summary>
        private RecordedRoute GetEditableRoute()
        {
            var route = waypointManager.GetRecordedRoute(visualizationRouteName);
            if (route != null) return route;

            waypointManager.LoadTrafficRoute(visualizationRouteName);
            return waypointManager.GetRecordedRoute(visualizationRouteName);
        }

        /// <summary>
        /// Try to resolve the selected node index from the current layout selection state.
        /// </summary>
        private bool TryGetSelectedNodeIndex(RecordedRoute route, out int index)
        {
            index = 0;
            if (route == null || route.Nodes == null || route.Nodes.Count == 0)
            {
                insim.Send(new IS_MST { Msg = "No recorded route data available." });
                return false;
            }

            if (!selectedRouteNodeIndex.HasValue)
            {
                insim.Send(new IS_MST { Msg = "Select a node in the layout editor first." });
                return false;
            }

            index = selectedRouteNodeIndex.Value;
            if (index < 0 || index >= route.Nodes.Count)
            {
                insim.Send(new IS_MST { Msg = "Selected node index is out of range." });
                return false;
            }

            return true;
        }

        /// <summary>
        /// Find the closest recorded route node to a selected layout object.
        /// </summary>
        private int FindClosestRouteNodeIndex(RecordedRoute route, ObjectInfo selectedObject, out double distanceMeters)
        {
            var targetX = selectedObject.X / 16.0;
            var targetY = selectedObject.Y / 16.0;

            var bestIndex = 0;
            var bestDistance = double.MaxValue;

            for (var i = 0; i < route.Nodes.Count; i++)
            {
                var node = route.Nodes[i];
                var dx = node.X - targetX;
                var dy = node.Y - targetY;
                var distance = Math.Sqrt(dx * dx + dy * dy);

                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestIndex = i;
                }
            }

            distanceMeters = bestDistance;
            return bestIndex;
        }

        /// <summary>
        /// Adjust the visualization detail step and optionally refresh the active layout.
        /// </summary>
        public void AdjustVisualizationDetail(bool increaseDetail, byte viewPlid)
        {
            var currentIndex = Array.IndexOf(VisualizationDetailSteps, visualizationDetailStep);
            if (currentIndex < 0) currentIndex = 0;

            var nextIndex = increaseDetail ? Math.Max(0, currentIndex - 1) : Math.Min(VisualizationDetailSteps.Length - 1,
                currentIndex + 1);
            visualizationDetailStep = VisualizationDetailSteps[nextIndex];
            mainUI?.UpdateVisualizationDetail(visualizationDetailStep);

            if (lfsLayout.WaypointsVisualized) VisualizeSelectedRoute(viewPlid);
        }

        /// <summary>
        /// Visualize the currently selected recorded route with the configured detail level.
        /// </summary>
        public void VisualizeSelectedRoute(byte viewPlid)
        {
            EnqueueVisualization(() => VisualizeSelectedRouteInternal(viewPlid));
        }

        /// <summary>
        /// Run the recorded route visualization without blocking the calling thread.
        /// </summary>
        private void VisualizeSelectedRouteInternal(byte viewPlid)
        {
            var effectivePlid = viewPlid == 0 ? GetDefaultVisualizationPlid() : viewPlid;
            if (effectivePlid == 0)
            {
                logger.LogWarning("Cannot visualize routes: no player in view");
                insim.Send(new IS_MST { Msg = "Select a car/view to place layout objects." });
                return;
            }

            waypointManager.LoadTrafficRoute(visualizationRouteName);
            var recorded = waypointManager.GetRecordedRoute(visualizationRouteName);
            if (recorded == null)
            {
                logger.LogWarning($"No recorded route found for visualization: {visualizationRouteName}");
                insim.Send(new IS_MST { Msg = $"No recorded route found for {visualizationRouteName}." });
                return;
            }

            var detailStep = GetVisualizationStep(recorded.Nodes?.Count ?? 0, out var clamped);
            lfsLayout.VisualizeRecordedRoute(effectivePlid, recorded, detailStep, true);
            lfsLayout.WaypointsVisualized = true;

            if (clamped)
            {
                insim.Send(new IS_MST
                {
                    Msg = $"Detail clamped to every {detailStep} node(s) to stay under {LFSLayout.MaxVisibleWaypoints} objects."
                });
            }
            else
            {
                insim.Send(new IS_MST { Msg = $"Visualized {visualizationRouteName} (x{detailStep})." });
            }
        }

        /// <summary>
        /// Calculate the effective detail step needed to stay under object limits.
        /// </summary>
        private int GetVisualizationStep(int nodeCount, out bool clamped)
        {
            var requiredStep = Math.Max(1, (int)Math.Ceiling(nodeCount / (double)LFSLayout.MaxVisibleWaypoints));
            var effective = Math.Max(visualizationDetailStep, requiredStep);
            clamped = effective != visualizationDetailStep;
            return effective;
        }
    }
}
