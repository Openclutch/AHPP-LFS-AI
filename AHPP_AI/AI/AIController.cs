using System;
using System.Collections.Generic;
using System.Diagnostics;
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
        private readonly Dictionary<byte, string> playerNames = new Dictionary<byte, string>();
        private readonly AIConfig config;

        // Debug UI
        private readonly DebugUI? debugUI;
        private readonly AIDriver driver;

        // Engine state tracking
        private readonly Dictionary<byte, bool> engineStateMap = new Dictionary<byte, bool>();
        private readonly GearboxController gearboxController;
        private readonly InSimClient insim;
        private readonly LFSLayout lfsLayout;
        private readonly Logger logger;
        private readonly AppConfig? appConfig;
        private readonly WaypointFollower waypointFollower;
        private readonly WaypointManager waypointManager;
        private readonly PathManager pathManager;
        private readonly RouteRecorder routeRecorder;
        private readonly RouteLibrary routeLibrary;
        private readonly AILightController lightController;
        private readonly Dictionary<string, RouteMetadata> routePresets = new Dictionary<string, RouteMetadata>(StringComparer.OrdinalIgnoreCase);
        private readonly MainUI mainUI;
        private readonly AIPopulationManager populationManager;
        private readonly bool insimDebugLogging;
        private readonly bool activeWaypointMarkersEnabled;
        private readonly int activeWaypointIntervalMs;
        private readonly bool laneChangeDetailedLogging;
        private readonly Dictionary<byte, string> aiAssignedRoutes = new Dictionary<byte, string>();
        private readonly object assignmentLock = new object();
        private readonly Queue<string> plannedSpawnRoutes = new Queue<string>();
        private readonly object plannedSpawnLock = new object();
        private readonly object aiPlidLock = new object();
        private readonly Dictionary<byte, string> currentRoute = new Dictionary<byte, string>();
        private readonly Dictionary<byte, BranchRouteInfo> activeBranchSelections = new Dictionary<byte, BranchRouteInfo>();
        private readonly Dictionary<byte, string> aiNames = new Dictionary<byte, string>();
        private readonly Dictionary<byte, DateTime> lastActiveWaypointUpdate = new Dictionary<byte, DateTime>();
        private readonly Random branchRandom = new Random();
        private readonly Dictionary<byte, DateTime> laneChangeLastCheck = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, DateTime> laneChangeLastMerge = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, LaneChangeRequest> pendingLaneChanges = new Dictionary<byte, LaneChangeRequest>();
        private readonly Dictionary<byte, ActiveLaneChange> activeLaneChanges = new Dictionary<byte, ActiveLaneChange>();
        private readonly Dictionary<byte, DateTime> laneChangeSignalStart = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, LaneChangeSkipInfo> laneChangeSkipLog = new Dictionary<byte, LaneChangeSkipInfo>();
        private readonly Dictionary<byte, RouteAssignmentLogInfo> routeAssignmentLog = new Dictionary<byte, RouteAssignmentLogInfo>();
        private readonly Dictionary<byte, DateTime> passByCooldowns = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, PassByProximityState> passByProximityStates =
            new Dictionary<byte, PassByProximityState>();
        private readonly SemaphoreSlim spawnSemaphore = new SemaphoreSlim(1, 1);
        private readonly SemaphoreSlim visualizationSemaphore = new SemaphoreSlim(1, 1);
        private readonly OutGauge outGauge;
        private readonly PerformanceTracker aiiPerformanceTracker = new PerformanceTracker();
        private readonly PerformanceTracker mciPerformanceTracker = new PerformanceTracker();
        private readonly bool performanceLogging;
        private readonly Random passByRandom = new Random();
        private bool autoVisualizeWaypoints;
        private bool debugButtonsVisible = true;
        private string recordingRouteName = "main_loop";
        private string visualizationRouteName = "main_loop";
        private int visualizationDetailStep = 2;
        private static readonly int[] VisualizationDetailSteps = { 1, 2, 4, 8, 16 };
        private int? selectedRouteNodeIndex;

        private float playerThrottle;
        private float playerBrake;

        private byte playerPLID;
        private DateTime lastPerformanceLog = DateTime.UtcNow;
        private static readonly TimeSpan PerformanceLogWindow = TimeSpan.FromSeconds(5);
        private const double PerformanceOnTimeMaxLoad = 0.20;
        private const double PerformanceSlowingMaxLoad = 0.35;
        private PerformanceHealth lastPerformanceHealth = PerformanceHealth.Waiting;
        private double lastPerformanceLoad;

        // Car tracking
        private readonly object carStateLock = new object();
        private readonly Dictionary<byte, CompCar> carStates = new Dictionary<byte, CompCar>();
        private CompCar[] allCars = Array.Empty<CompCar>();
        private volatile TrafficCarSnapshot[] trafficSnapshot = Array.Empty<TrafficCarSnapshot>();
        private bool debugUIInitialized;

        private enum PerformanceHealth
        {
            Waiting,
            OnTime,
            Slowing,
            RunningSlow
        }

        /// <summary>
        ///     Tracks the last nearby human target for pass-by reactions per AI.
        /// </summary>
        private class PassByProximityState
        {
            public byte TargetPlid { get; set; }
            public DateTime LastSeen { get; set; }
        }

        /// <summary>
        ///     Initialize the AI controller with dependencies
        /// </summary>
        public AIController(
            InSimClient insim,
            Logger logger,
            WaypointManager waypointManager,
            LFSLayout lfsLayout,
            RouteLibrary routeLibrary,
            bool debugEnabled = false,
            AppConfig? appConfig = null,
            bool autoVisualizeWaypointsEnabled = false,
            bool insimDebugLogging = false,
            bool activeWaypointMarkersEnabled = true,
            int activeWaypointIntervalMs = 500,
            bool performanceLoggingEnabled = true)
        {
            this.insim = insim;
            this.logger = logger;
            this.waypointManager = waypointManager;
            this.lfsLayout = lfsLayout;
            this.routeLibrary = routeLibrary;
            this.appConfig = appConfig;
            autoVisualizeWaypoints = autoVisualizeWaypointsEnabled;
            this.insimDebugLogging = insimDebugLogging;
            this.activeWaypointMarkersEnabled = activeWaypointMarkersEnabled;
            this.activeWaypointIntervalMs = activeWaypointIntervalMs;
            performanceLogging = performanceLoggingEnabled;
            debugButtonsVisible = appConfig?.GetBool("DebugAI", "ShowDebugButtons", true) ?? true;
            laneChangeDetailedLogging = appConfig?.GetBool("DebugAI", "LaneChangeDetailedLogging", true) ?? true;

            // Create configuration
            config = new AIConfig { DebugEnabled = debugEnabled };

            pathManager = new PathManager(waypointManager, logger, routeLibrary);
            pathManager.LoadRoutes(config);

            // Initialize debug UI if enabled
            if (debugEnabled)
            {
                debugUI = new DebugUI(insim, logger);
                debugUI.SetDebugButtonsVisible(debugButtonsVisible);
            }

            // Create component hierarchy
            var steeringCalculator = new SteeringCalculator(logger);
            waypointFollower = new WaypointFollower(config, logger, steeringCalculator);
            gearboxController = new GearboxController(config, logger);
            lightController = new AILightController(insim, logger);
            driver = new AIDriver(config, logger, waypointFollower, gearboxController, insim, lightController);
            driver.SetRecoveryFailedHandler(ResetAI);
            mainUI = new MainUI(insim, logger);
            mainUI.SetDebugButtonsVisible(debugButtonsVisible);
            routeRecorder = new RouteRecorder(logger, lfsLayout, debugUI, routeLibrary, mainUI);
            outGauge = new OutGauge();
            outGauge.PacketReceived += OnOutGauge;
            InitializeRoutePresets();
            populationManager = new AIPopulationManager(this, pathManager, routeLibrary, config, logger);
        }

        /// <summary>
        /// Wrap AI names with quotes when needed for chat/admin commands.
        /// </summary>
        private static string FormatNameForCommand(string name)
        {
            return string.IsNullOrWhiteSpace(name) ? string.Empty : name;
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
                DefaultSpeedLimit = metadata.DefaultSpeedLimit,
                AiTargetCount = metadata.AiTargetCount,
                AiTargetPercent = metadata.AiTargetPercent,
                AiWeight = metadata.AiWeight,
                AiEnabled = metadata.AiEnabled
            };
        }

        /// <summary>
        /// Represents a scheduled lane change including its timing and target route.
        /// </summary>
        private class LaneChangeRequest
        {
            public BranchRouteInfo? TargetRoute { get; set; }
            public List<Util.Waypoint> TargetPath { get; set; } = new List<Util.Waypoint>();
            public bool ToAlternate { get; set; }
            public DateTime ExecuteAt { get; set; }
            public string Description { get; set; } = string.Empty;
            public int ScheduledFromIndex { get; set; }
        }

        /// <summary>
        /// Tracks an in-progress lane change that is following a generated transition path.
        /// </summary>
        private class ActiveLaneChange
        {
            public BranchRouteInfo? TargetRoute { get; set; }
            public bool ToAlternate { get; set; }
            public int TargetEntryIndex { get; set; }
            public int TransitionPathCount { get; set; }
        }

        /// <summary>
        /// Throttles repeated lane-change skip logs so the reasons are visible without spamming the log.
        /// </summary>
        private class LaneChangeSkipInfo
        {
            public DateTime LastLogged { get; set; }
            public string Reason { get; set; } = string.Empty;
        }

        /// <summary>
        /// Throttles route assignment logs so assignment decisions are visible without spamming the log.
        /// </summary>
        private class RouteAssignmentLogInfo
        {
            public DateTime LastLogged { get; set; }
            public string Reason { get; set; } = string.Empty;
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
            if (!metadata.AiWeight.HasValue || metadata.AiWeight <= 0) metadata.AiWeight = 1.0;
            if (metadata.AiTargetPercent.HasValue)
                metadata.AiTargetPercent = Math.Max(0, Math.Min(1.0, metadata.AiTargetPercent.Value));

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
                debugUI.SetDebugButtonsVisible(debugButtonsVisible);
                debugUI.Initialize();
                debugUIInitialized = true;
                logger.Log("Debug UI initialized");
                mainUI.Show();
                mainUI.SetDebugButtonsVisible(debugButtonsVisible);
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

            lock (aiPlidLock)
            {
                if (aiPLIDs.Contains(viewPlid))
                {
                    debugUI.SetAIPLID(viewPlid);
                }
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
            visualizationRouteName = recordingRouteName;
            mainUI?.UpdateVisualizationRouteSelection(visualizationRouteName);
            RefreshRouteOptions(recordingRouteName);
        }

        /// <summary>
        /// Refresh available route selection buttons based on the current track/layout context.
        /// </summary>
        public void RefreshRouteOptions(string? preferredSelection = null)
        {
            var selection = string.IsNullOrWhiteSpace(preferredSelection)
                ? recordingRouteName
                : routeLibrary.NormalizeRouteName(preferredSelection);

            var recordedLookup = BuildRecordedRouteLookup();
            var options = new List<(string name, bool recorded)>();
            AddRouteOption(options, config.MainRouteName, recordedLookup);
            AddRouteOption(options, config.MainAlternateRouteName, recordedLookup);
            AddRouteOption(options, config.SpawnRouteName, recordedLookup);
            if (config.BranchRouteNames != null)
            {
                foreach (var branch in config.BranchRouteNames)
                {
                    AddRouteOption(options, branch, recordedLookup);
                }
            }

            foreach (var recorded in recordedLookup.OrderBy(r => r.Key, StringComparer.OrdinalIgnoreCase))
            {
                AddRouteOption(options, recorded.Key, recordedLookup, recorded.Value);
            }

            AddRouteOption(options, selection, recordedLookup);
            mainUI?.SetRouteOptions(options, selection);

            var visualizationOptions = new List<string>();
            foreach (var recorded in recordedLookup.OrderBy(r => r.Key, StringComparer.OrdinalIgnoreCase))
            {
                if (recorded.Value) AddOption(visualizationOptions, recorded.Key);
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
        private static void AddOption(List<string> options, string? name)
        {
            if (options == null) return;
            if (string.IsNullOrWhiteSpace(name)) return;
            if (options.Exists(o => o.Equals(name, StringComparison.OrdinalIgnoreCase))) return;
            options.Add(name);
        }

        /// <summary>
        /// Add a route option with recorded status, ensuring uniqueness by name.
        /// </summary>
        private static void AddRouteOption(
            List<(string name, bool recorded)> options,
            string? name,
            Dictionary<string, bool> recordedLookup,
            bool? recordedOverride = null)
        {
            if (options == null || string.IsNullOrWhiteSpace(name)) return;
            if (options.Exists(o => o.name.Equals(name, StringComparison.OrdinalIgnoreCase))) return;

            var recorded = recordedOverride ??
                           (recordedLookup != null &&
                            recordedLookup.TryGetValue(name, out var hasRecording) &&
                            hasRecording);
            options.Add((name, recorded));
        }

        /// <summary>
        /// Build a lookup of recorded routes and whether they contain points for the current track/layout.
        /// </summary>
        private Dictionary<string, bool> BuildRecordedRouteLookup()
        {
            var lookup = new Dictionary<string, bool>(StringComparer.OrdinalIgnoreCase);

            try
            {
                var routes = routeLibrary.ListRoutes(out var duplicateNames);
                foreach (var duplicate in duplicateNames)
                {
                    logger.LogWarning(
                        $"Duplicate recorded route name \"{duplicate}\" detected while refreshing UI options.");
                }

                foreach (var route in routes)
                {
                    var name = route?.Metadata?.Name;
                    if (string.IsNullOrWhiteSpace(name)) continue;
                    var hasPoints = route?.Nodes != null && route.Nodes.Count > 0;
                    lookup[name] = hasPoints;
                }
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to build recorded route lookup");
            }

            return lookup;
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
        /// Configure how long the clutch stays fully pressed after a completed shift before releasing.
        /// </summary>
        public void SetClutchHoldAfterShiftMs(int holdMs)
        {
            config.ClutchHoldAfterShiftMs = Math.Max(0, holdMs);
        }

        /// <summary>
        /// Configure base throttle/brake values and steering range/center for AI inputs.
        /// </summary>
        public void ConfigureControlInputs(int throttleBase, int brakeBase, int minSteering, int maxSteering,
            int steeringCenter)
        {
            var clampedThrottle = Math.Max(0, Math.Min(65535, throttleBase));
            var clampedBrake = Math.Max(0, Math.Min(65535, brakeBase));
            var clampedMinSteering = Math.Max(0, Math.Min(65535, minSteering));
            var clampedMaxSteering = Math.Max(0, Math.Min(65535, maxSteering));

            if (clampedMaxSteering < clampedMinSteering)
            {
                var swap = clampedMaxSteering;
                clampedMaxSteering = clampedMinSteering;
                clampedMinSteering = swap;
            }

            var clampedCenter = Math.Max(clampedMinSteering, Math.Min(clampedMaxSteering, steeringCenter));

            config.ThrottleBase = clampedThrottle;
            config.BrakeBase = clampedBrake;
            config.MinSteering = clampedMinSteering;
            config.MaxSteering = clampedMaxSteering;
            config.SteeringCenter = clampedCenter;
        }

        /// <summary>
        /// Configure brake smoothing thresholds and ramp steps.
        /// </summary>
        public void ConfigureBrakeSmoothing(double coastSpeedErrorKmh, double brakeSpeedErrorKmh, int riseStep,
            int releaseStep)
        {
            var clampedCoast = Math.Max(0.0, coastSpeedErrorKmh);
            var clampedBrake = Math.Max(clampedCoast, brakeSpeedErrorKmh);

            config.BrakeCoastSpeedErrorKmh = clampedCoast;
            config.BrakeApplySpeedErrorKmh = clampedBrake;
            config.BrakeRiseStep = Math.Max(0, riseStep);
            config.BrakeReleaseStep = Math.Max(0, releaseStep);
        }

        /// <summary>
        /// Configure gearbox timing, hysteresis, and speed thresholds for shifts.
        /// </summary>
        public void ConfigureGearbox(int shiftDelayMs, int upshiftHysteresisKmh, int downshiftHysteresisKmh,
            int gearShiftMinIntervalMs, double[] gearSpeedThresholdsKmh)
        {
            config.ShiftDelayMs = Math.Max(0, shiftDelayMs);
            config.GearUpshiftHysteresisKmh = Math.Max(0, upshiftHysteresisKmh);
            config.GearDownshiftHysteresisKmh = Math.Max(0, downshiftHysteresisKmh);
            config.GearShiftMinIntervalMs = Math.Max(0, gearShiftMinIntervalMs);

            if (gearSpeedThresholdsKmh != null && gearSpeedThresholdsKmh.Length > 0)
            {
                var cleaned = gearSpeedThresholdsKmh
                    .Where(value => value > 0.0)
                    .OrderBy(value => value)
                    .ToArray();

                if (cleaned.Length > 0)
                    config.GearSpeedThresholds = cleaned;
            }
        }

        /// <summary>
        /// Configure clutch timing, range, and stall-prevention behavior.
        /// </summary>
        public void ConfigureClutch(int clutchFullyPressed, int clutchReleased, int clutchPressDelayMs,
            int clutchHoldAfterShiftMs, int clutchReleaseSteps, int clutchReleaseIntervalMs, int stallPreventionRpm,
            int stallPreventionReleaseRpm, int stallPreventionHoldMs)
        {
            var clampedPressed = Math.Max(0, Math.Min(65535, clutchFullyPressed));
            var clampedReleased = Math.Max(0, Math.Min(clampedPressed, clutchReleased));
            var clampedStallRpm = Math.Max(0, stallPreventionRpm);

            config.ClutchFullyPressed = clampedPressed;
            config.ClutchReleased = clampedReleased;
            config.ClutchPressDelayMs = Math.Max(0, clutchPressDelayMs);
            config.ClutchHoldAfterShiftMs = Math.Max(0, clutchHoldAfterShiftMs);
            config.ClutchReleaseSteps = Math.Max(1, clutchReleaseSteps);
            config.ClutchReleaseIntervalMs = Math.Max(1, clutchReleaseIntervalMs);
            config.StallPreventionRpm = clampedStallRpm;
            config.StallPreventionReleaseRpm = Math.Max(clampedStallRpm, stallPreventionReleaseRpm);
            config.StallPreventionHoldMs = Math.Max(0, stallPreventionHoldMs);
        }

        /// <summary>
        /// Configure waypoint threshold tuning used for proximity checks.
        /// </summary>
        public void ConfigureWaypointThresholds(double minThresholdMeters, double maxThresholdMeters,
            double speedFactor)
        {
            var clampedMin = Math.Max(0.1, minThresholdMeters);
            var clampedMax = Math.Max(clampedMin, maxThresholdMeters);

            config.WaypointMinThreshold = clampedMin;
            config.WaypointMaxThreshold = clampedMax;
            config.WaypointThresholdSpeedFactor = Math.Max(0.0, speedFactor);
        }

        /// <summary>
        /// Configure recovery limits and progress checks used when validating path tracking.
        /// </summary>
        public void ConfigureRecoveryLimits(bool wallRecoveryEnabled, int waypointTimeoutSeconds,
            int progressCheckIntervalMs, double minRequiredProgress, int maxRecoveryAttempts,
            int maxFailedRecoveryCycles, double minSpeedThreshold, int stationaryCheckCount)
        {
            config.WallRecoveryEnabled = wallRecoveryEnabled;
            config.WaypointTimeoutSeconds = Math.Max(1, waypointTimeoutSeconds);
            config.ProgressCheckIntervalMs = Math.Max(100, progressCheckIntervalMs);
            config.MinRequiredProgress = Math.Max(0.0, minRequiredProgress);
            config.MaxRecoveryAttempts = Math.Max(1, maxRecoveryAttempts);
            config.MaxFailedRecoveryCycles = Math.Max(1, maxFailedRecoveryCycles);
            config.MinSpeedThreshold = Math.Max(0.0, minSpeedThreshold);
            config.StationaryCheckCount = Math.Max(1, stationaryCheckCount);
        }

        /// <summary>
        /// Set build/version string for startup announcements.
        /// </summary>
        public void SetBuildVersion(string version)
        {
            config.BuildVersion = string.IsNullOrWhiteSpace(version) ? "dev" : version.Trim();
        }

        /// <summary>
        /// Configure collision detection envelope when scanning for cars ahead.
        /// </summary>
        public void ConfigureCollisionDetection(
            double detectionRangeMeters,
            double detectionAngleDegrees,
            double minimumSafetyDistanceMeters,
            double collisionHalfWidthMeters)
        {
            config.CollisionDetectionRangeM = Math.Max(1.0, detectionRangeMeters);
            config.CollisionDetectionAngle = Math.Max(0.1, detectionAngleDegrees);
            config.MinimumSafetyDistanceM = Math.Max(0.1, minimumSafetyDistanceMeters);
            config.CollisionDetectionHalfWidthM = Math.Max(0.1, collisionHalfWidthMeters);
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
        /// Configure lane change behavior between the main and alternate routes.
        /// </summary>
        public void ConfigureLaneChange(
            double initialCheckIntervalSeconds,
            double postCooldownIntervalSeconds,
            double mergeChance,
            double cooldownSeconds,
            double safetyCheckDistanceMeters,
            double safetyCheckHalfWidthMeters,
            double maxParallelDistanceMeters,
            double maxParallelHeadingDegrees,
            double parallelLookaheadSeconds,
            double parallelLookaheadMinMeters,
            double maxMergeSpeedKmh,
            double signalLeadTimeSeconds,
            double signalMinimumDurationSeconds,
            double transitionLengthMeters,
            int transitionPointCount,
            int targetAheadWaypoints,
            double rearCheckDistanceMeters,
            double rearCheckTtcSeconds,
            double mergeGapFactor)
        {
            config.LaneChangeInitialCheckIntervalSeconds = Math.Max(1.0, initialCheckIntervalSeconds);
            config.LaneChangePostCooldownIntervalSeconds = Math.Max(1.0, postCooldownIntervalSeconds);
            config.LaneChangeMergeChance = Math.Max(0.0, Math.Min(1.0, mergeChance));
            config.LaneChangeCooldownSeconds = Math.Max(1.0, cooldownSeconds);
            config.LaneChangeSafetyCheckDistanceMeters = Math.Max(1.0, safetyCheckDistanceMeters);
            config.LaneChangeSafetyCheckHalfWidthMeters = Math.Max(0.1, safetyCheckHalfWidthMeters);
            config.LaneChangeMaxParallelDistanceMeters = Math.Max(0.1, maxParallelDistanceMeters);
            config.LaneChangeMaxParallelHeadingDegrees = Math.Max(0.0, maxParallelHeadingDegrees);
            config.LaneChangeParallelLookaheadSeconds = Math.Max(0.1, parallelLookaheadSeconds);
            config.LaneChangeParallelLookaheadMinMeters = Math.Max(1.0, parallelLookaheadMinMeters);
            config.LaneChangeMaxMergeSpeedKmh = Math.Max(1.0, maxMergeSpeedKmh);
            config.LaneChangeSignalLeadTimeSeconds = Math.Max(0.0, signalLeadTimeSeconds);
            config.LaneChangeSignalMinimumDurationSeconds = Math.Max(0.5, signalMinimumDurationSeconds);
            config.LaneChangeTransitionLengthMeters = Math.Max(1.0, transitionLengthMeters);
            config.LaneChangeTransitionPointCount = Math.Max(4, transitionPointCount);
            config.LaneChangeTargetAheadWaypoints = Math.Max(1, targetAheadWaypoints);
            config.LaneChangeRearCheckDistanceMeters = Math.Max(1.0, rearCheckDistanceMeters);
            config.LaneChangeRearCheckTtcSeconds = Math.Max(0.1, rearCheckTtcSeconds);
            config.LaneChangeMergeGapFactor = Math.Max(0.1, mergeGapFactor);
        }

        /// <summary>
        /// Configure traffic-aware spacing and pit merge yielding behavior.
        /// </summary>
        public void ConfigureTrafficAwareness(
            double pathLoopClosureMeters,
            double laneHalfWidthMeters,
            double baseGapMeters,
            double timeHeadwaySeconds,
            double aiSpacingFactor,
            double humanSpacingFactor,
            double lookaheadSeconds,
            double lookaheadMinMeters,
            double brakeTtcSeconds,
            double emergencyTtcSeconds,
            int spawnMergeHoldLookaheadWaypoints,
            double spawnMergeHoldDistanceMeters)
        {
            config.PathLoopClosureDistanceMeters = Math.Max(0.1, pathLoopClosureMeters);
            config.TrafficLaneHalfWidthMeters = Math.Max(0.1, laneHalfWidthMeters);
            config.TrafficBaseGapMeters = Math.Max(0.1, baseGapMeters);
            config.TrafficTimeHeadwaySeconds = Math.Max(0.1, timeHeadwaySeconds);
            config.TrafficAiSpacingFactor = Math.Max(0.1, aiSpacingFactor);
            config.TrafficHumanSpacingFactor = Math.Max(0.1, humanSpacingFactor);
            config.TrafficLookaheadSeconds = Math.Max(0.1, lookaheadSeconds);
            config.TrafficLookaheadMinMeters = Math.Max(0.1, lookaheadMinMeters);
            config.TrafficBrakeTtcSeconds = Math.Max(0.1, brakeTtcSeconds);
            config.TrafficEmergencyTtcSeconds = Math.Max(0.1, emergencyTtcSeconds);
            config.SpawnMergeHoldLookaheadWaypoints = Math.Max(1, spawnMergeHoldLookaheadWaypoints);
            config.SpawnMergeHoldDistanceMeters = Math.Max(1.0, spawnMergeHoldDistanceMeters);
        }

        /// <summary>
        /// Configure the AI reaction when a fast player drives nearby.
        /// </summary>
        public void ConfigurePassByReactions(
            bool enabled,
            double chance,
            double speedThresholdKmh,
            double durationSeconds,
            double distanceMeters,
            AIConfig.PassByReactionMode mode,
            double cooldownMinSeconds,
            double cooldownMaxSeconds,
            double proximityResetSeconds)
        {
            config.PassByReactionEnabled = enabled;
            config.PassByReactionChance = Math.Max(0.0, Math.Min(1.0, chance));
            config.PassBySpeedThresholdKmh = Math.Max(0.0, speedThresholdKmh);
            config.PassByReactionDurationSeconds = Math.Max(0.1, durationSeconds);
            config.PassByReactionDistanceMeters = Math.Max(1.0, distanceMeters);
            config.PassByMode = mode;
            config.PassByCooldownMinSeconds = Math.Max(0.0, cooldownMinSeconds);
            config.PassByCooldownMaxSeconds = Math.Max(config.PassByCooldownMinSeconds, cooldownMaxSeconds);
            config.PassByProximityResetSeconds = Math.Max(0.1, proximityResetSeconds);
        }

        /// <summary>
        /// Configure unified recovery timings and success thresholds.
        /// </summary>
        public void ConfigureRecovery(
            int shortReverseMs,
            int longReverseMs,
            int cooldownMs,
            int validationWindowMs,
            double successDistanceMeters,
            double successSpeedKmh,
            double positionChangeThreshold,
            double progressStallThreshold,
            int stuckCheckIntervalMs,
            double lowSpeedThresholdKmh,
            int detectionsBeforeAction,
            int maxFailureCount,
            int stallReverseTrigger)
        {
            config.RecoveryShortReverseMs = Math.Max(200, shortReverseMs);
            config.RecoveryLongReverseMs = Math.Max(config.RecoveryShortReverseMs, longReverseMs);
            config.RecoveryCooldownMs = Math.Max(100, cooldownMs);
            config.RecoveryValidationWindowMs = Math.Max(200, validationWindowMs);
            config.RecoverySuccessDistanceMeters = Math.Max(0.1, successDistanceMeters);
            config.RecoverySuccessSpeedKmh = Math.Max(0.0, successSpeedKmh);
            config.RecoveryPositionChangeThreshold = Math.Max(0.1, positionChangeThreshold);
            config.RecoveryProgressStallThreshold = Math.Max(0.0, progressStallThreshold);
            config.RecoveryStuckCheckIntervalMs = Math.Max(200, stuckCheckIntervalMs);
            config.RecoveryLowSpeedThresholdKmh = Math.Max(0.1, lowSpeedThresholdKmh);
            config.RecoveryDetectionsBeforeAction = Math.Max(1, detectionsBeforeAction);
            config.RecoveryMaxFailureCount = Math.Max(1, maxFailureCount);
            config.RecoveryStallReverseTrigger = Math.Max(1, stallReverseTrigger);
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
            populationManager?.RequestReconcile("route config");
        }

        /// <summary>
        /// Stop route recording and refresh UI state to reflect any saved data.
        /// </summary>
        public void StopRecording()
        {
            routeRecorder.Stop();
            RefreshRouteOptions(recordingRouteName);
        }

        /// <summary>
        /// Reset layout visuals.
        /// </summary>
        public void ResetLayout()
        {
            lfsLayout.ClearAllVisualizations();
            lfsLayout.WaypointsVisualized = false;
            mainUI?.UpdateLayoutToggleState(false);
            insim.SendPrivateMessage("Layout cleared.");
        }

        /// <summary>
        /// Reset a single AI by spectating it and spawning a new one.
        /// </summary>
        private void ResetAI(byte plid)
        {
            lock (aiPlidLock)
            {
                if (!aiPLIDs.Contains(plid))
                    return;
            }

            var name = aiNames.TryGetValue(plid, out var storedName) ? storedName : $"AI {plid}";
            var nameForCmd = FormatNameForCommand(name);

            // Spectate by name (PLID is not accepted), then rejoin.
            insim.Send(new IS_MST { Msg = $"/spec {nameForCmd}" });
            insim.Send(new IS_MST { Msg = $"/join {nameForCmd}" });

            ForgetAi(plid);
            insim.Send(new IS_MST { Msg = "/ai" });
            logger.Log($"Reset AI {plid} after failed recovery; sent to pits/spectate and spawned replacement");
        }

        /// <summary>
        /// Toggle the visualization of recorded routes for the current view, with a fallback to a default player.
        /// </summary>
        public void ToggleRouteVisualization(byte viewPlid)
        {
            var effectivePlid = viewPlid == 0 ? GetDefaultVisualizationPlid() : viewPlid;
            if (effectivePlid == 0)
            {
                logger.LogWarning("Cannot visualize routes: no player in view");
                insim.SendPrivateMessage("Select a car/view to place layout objects.");
                return;
            }

            if (lfsLayout.WaypointsVisualized)
            {
                lfsLayout.ClearAllVisualizations();
                lfsLayout.WaypointsVisualized = false;
                mainUI?.UpdateLayoutToggleState(false);
                PersistWaypointVisualizationPreference(false);
                insim.SendPrivateMessage("Layout visualization hidden.");
                return;
            }

            EnqueueVisualization(() => VisualizeLayoutAndWaypoints(effectivePlid));
        }

        /// <summary>
        /// Visualize the selected recorded route and AI waypoint cones together.
        /// </summary>
        private void VisualizeLayoutAndWaypoints(byte viewPlid)
        {
            VisualizeWaypointConesInternal(viewPlid);
            VisualizeSelectedRouteInternal(viewPlid, false);

            if (!lfsLayout.WaypointsVisualized) return;

            mainUI?.UpdateLayoutToggleState(true);
            PersistWaypointVisualizationPreference(true);
        }

        /// <summary>
        /// Render waypoint cones for the active AI path tied to the supplied player.
        /// </summary>
        private void VisualizeWaypointConesInternal(byte plid)
        {
            var paths = GetAIPaths();
            if (paths == null || paths.Count == 0)
            {
                logger.LogWarning("Cannot visualize waypoint cones: no AI paths available");
                insim.SendPrivateMessage("No AI paths available for waypoint cones.");
                return;
            }

            if (!paths.ContainsKey(plid))
            {
                var fallbackPlid = paths.Keys.FirstOrDefault();
                if (fallbackPlid == 0)
                {
                    logger.LogWarning("Cannot visualize waypoint cones: no valid PLID found");
                    insim.SendPrivateMessage("No AI cars available for waypoint cones.");
                    return;
                }

                plid = fallbackPlid;
            }

            if (!paths.TryGetValue(plid, out var path) || path == null || path.Count == 0)
            {
                logger.LogWarning($"Cannot visualize waypoint cones: path empty for PLID {plid}");
                insim.SendPrivateMessage("No waypoints to visualize for the selected AI.");
                return;
            }

            lfsLayout.VisualizeWaypoints(plid, paths);
        }

        /// <summary>
        /// Persist the waypoint visualization preference so it survives restarts.
        /// </summary>
        private void PersistWaypointVisualizationPreference(bool enabled)
        {
            autoVisualizeWaypoints = enabled;
            try
            {
                appConfig?.SetBool("DebugAI", "AutoVisualizeWaypoints", enabled, true);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to persist waypoint visualization preference");
            }
        }

        /// <summary>
        /// Toggle visibility of the debug HUD buttons and persist the preference.
        /// </summary>
        public void ToggleDebugButtonsVisibility()
        {
            if (!config.DebugEnabled || debugUI == null) return;

            debugButtonsVisible = !debugButtonsVisible;
            debugUI.SetDebugButtonsVisible(debugButtonsVisible);
            mainUI.SetDebugButtonsVisible(debugButtonsVisible);
            PersistDebugButtonsPreference(debugButtonsVisible);

            var status = debugButtonsVisible ? "shown" : "hidden";
            insim.SendPrivateMessage($"Debug buttons {status}.");
        }

        /// <summary>
        /// Persist the debug HUD visibility preference to the config file.
        /// </summary>
        /// <param name="visible">True when debug buttons should be shown.</param>
        private void PersistDebugButtonsPreference(bool visible)
        {
            try
            {
                appConfig?.SetBool("DebugAI", "ShowDebugButtons", visible, true);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to persist debug buttons preference");
            }
        }

        public bool IsRecording => routeRecorder.IsRecording;

        public string MainRouteName => config.MainRouteName;

        /// <summary>
        ///     Get the first AI car's player ID
        /// </summary>
        public byte GetFirstAICarID()
        {
            lock (aiPlidLock)
            {
                return aiPLIDs.Count > 0 ? aiPLIDs[0] : (byte)0;
            }
        }

        /// <summary>
        ///     Get an AI's name formatted for chat/admin commands (returns empty when unknown).
        /// </summary>
        public string GetAINameForCommand(byte plid)
        {
            lock (aiPlidLock)
            {
                if (!aiPLIDs.Contains(plid))
                    return string.Empty;
            }

            var name = aiNames.TryGetValue(plid, out var storedName) ? storedName : $"AI {plid}";
            return FormatNameForCommand(name);
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
        /// Find the index of the waypoint closest to the given world position.
        /// </summary>
        private static int FindClosestIndex(List<Util.Waypoint> path, double xMeters, double yMeters)
        {
            if (path == null || path.Count == 0) return 0;

            var closestIndex = 0;
            var minDistance = double.MaxValue;

            for (var i = 0; i < path.Count; i++)
            {
                var wpX = path[i].Position.X / 65536.0;
                var wpY = path[i].Position.Y / 65536.0;
                var dx = wpX - xMeters;
                var dy = wpY - yMeters;
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
        /// Calculate the distance between two waypoint points in meters.
        /// </summary>
        private static double DistanceMeters(global::SixLabors.ImageSharp.Point a, global::SixLabors.ImageSharp.Point b)
        {
            var dx = (a.X - b.X) / 65536.0;
            var dy = (a.Y - b.Y) / 65536.0;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// Get a point along a path a set distance ahead of a start index, including the index used for heading.
        /// </summary>
        private (double x, double y, int index) GetPathPointAhead(
            List<Util.Waypoint> path,
            int startIndex,
            double distanceMeters)
        {
            if (path == null || path.Count == 0) return (0, 0, 0);

            var clampedStart = ClampIndex(path, startIndex);
            var remaining = Math.Max(0.0, distanceMeters);
            var currentIndex = clampedStart;
            var currentPoint = path[currentIndex].Position;

            var steps = 0;
            while (remaining > 0 && steps <= path.Count)
            {
                var nextIndex = (currentIndex + 1) % path.Count;
                var nextPoint = path[nextIndex].Position;
                var segmentDistance = DistanceMeters(currentPoint, nextPoint);

                if (segmentDistance >= remaining && segmentDistance > 0.0001)
                {
                    var ratio = remaining / segmentDistance;
                    var curX = currentPoint.X / 65536.0;
                    var curY = currentPoint.Y / 65536.0;
                    var nextX = nextPoint.X / 65536.0;
                    var nextY = nextPoint.Y / 65536.0;
                    return (
                        curX + (nextX - curX) * ratio,
                        curY + (nextY - curY) * ratio,
                        currentIndex);
                }

                remaining -= segmentDistance;
                currentIndex = nextIndex;
                currentPoint = nextPoint;
                steps++;
            }

            return (currentPoint.X / 65536.0, currentPoint.Y / 65536.0, currentIndex);
        }

        /// <summary>
        /// Calculate heading delta between two direction vectors in degrees.
        /// </summary>
        private static double CalculateHeadingDeltaDegrees((double x, double y) first, (double x, double y) second)
        {
            var magFirst = Math.Sqrt(first.x * first.x + first.y * first.y);
            var magSecond = Math.Sqrt(second.x * second.x + second.y * second.y);
            if (magFirst < 0.0001 || magSecond < 0.0001) return 180.0;

            var dot = first.x * second.x + first.y * second.y;
            var cos = dot / (magFirst * magSecond);
            cos = Math.Max(-1.0, Math.Min(1.0, cos));
            return Math.Abs(Math.Acos(cos) * 180.0 / Math.PI);
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
        /// Check if a branch represents the alternate main lane based on metadata or configured route name.
        /// </summary>
        private bool IsAlternateMainBranch(BranchRouteInfo? branchInfo)
        {
            if (branchInfo == null) return false;
            if (branchInfo.Metadata?.Type == RouteType.AlternateMain) return true;
            if (pathManager.MainAlternateRoute == null) return false;
            return branchInfo.Name.Equals(pathManager.MainAlternateRoute.Name, StringComparison.OrdinalIgnoreCase);
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
            var headingRadians =
                (carHeading + CoordinateUtils.QUARTER_CIRCLE) * 2 * Math.PI / CoordinateUtils.FULL_CIRCLE;
            var forwardVector = (x: Math.Cos(headingRadians), y: Math.Sin(headingRadians));

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
                var forwardProjection = dx * forwardVector.x + dy * forwardVector.y;
                var desiredHeading = CoordinateUtils.CalculateHeadingToTarget(dx, dy);
                var headingError = Math.Abs(CoordinateUtils.CalculateHeadingError(carHeading, desiredHeading));
                var headingErrorDeg = Math.Abs(CoordinateUtils.HeadingToDegrees(headingError));

                // Also compare against the path’s forward vector to avoid penalizing parallel lanes that
                // sit beside us but whose waypoint position is slightly behind/ahead.
                var pathDir = GetDirectionVector(path, i);
                var pathHeadingErrorDeg = 180.0;
                if (Math.Abs(pathDir.x) > 0.0001 || Math.Abs(pathDir.y) > 0.0001)
                {
                    var pathHeading = CoordinateUtils.CalculateHeadingToTarget(pathDir.x, pathDir.y);
                    var pathHeadingError = Math.Abs(CoordinateUtils.CalculateHeadingError(carHeading, pathHeading));
                    pathHeadingErrorDeg = Math.Abs(CoordinateUtils.HeadingToDegrees(pathHeadingError));
                }

                headingErrorDeg = forwardProjection >= 0
                    ? Math.Min(headingErrorDeg, pathHeadingErrorDeg)
                    : headingErrorDeg;
                if (headingErrorDeg > 180) headingErrorDeg = 360 - headingErrorDeg;

                var aheadPenalty = forwardProjection < 0 ? Math.Abs(forwardProjection) * 10.0 : 0.0;
                var distanceScore = distance;
                var headingScore = headingErrorDeg * 0.1;
                var backwardPenalty = headingErrorDeg > 135 ? 50.0 : 0.0;
                var indexPenalty = Math.Abs(i - preferredIndex) * 0.5;
                var score = distanceScore + headingScore + backwardPenalty + indexPenalty + aheadPenalty;

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
        /// Determine if a lane change check should run based on cooldowns and timers.
        /// </summary>
        private bool ShouldRunLaneChangeCheck(byte plid)
        {
            var now = DateTime.Now;
            var hasMergedBefore = laneChangeLastMerge.TryGetValue(plid, out var lastMerge);
            var cooldown = TimeSpan.FromSeconds(config.LaneChangeCooldownSeconds);

            if (hasMergedBefore && now - lastMerge < cooldown)
            {
                var remaining = Math.Max(0, (cooldown - (now - lastMerge)).TotalSeconds);
                LogLaneChangeSkip(plid, $"cooldown active ({remaining:F1}s remaining)");
                return false;
            }

            var intervalSeconds = hasMergedBefore
                ? config.LaneChangePostCooldownIntervalSeconds
                : config.LaneChangeInitialCheckIntervalSeconds;
            var interval = TimeSpan.FromSeconds(intervalSeconds);

            if (!laneChangeLastCheck.TryGetValue(plid, out var lastCheck))
            {
                laneChangeLastCheck[plid] = now;
                LogLaneChangeSkip(plid, $"seeding timer; first check after {interval.TotalSeconds:F1}s");
                return false;
            }

            if (now - lastCheck < interval)
            {
                var remaining = Math.Max(0, (interval - (now - lastCheck)).TotalSeconds);
                LogLaneChangeSkip(plid, $"waiting interval ({remaining:F1}s remaining)");
                return false;
            }

            laneChangeLastCheck[plid] = now;

            if (hasMergedBefore && branchRandom.NextDouble() > config.LaneChangeMergeChance)
            {
                LogLaneChangeSkip(plid, $"post-merge random skip (chance {config.LaneChangeMergeChance:P0})");
                return false;
            }

            return true;
        }

        /// <summary>
        /// Ensure the target lane stays parallel over a speed-scaled lookahead window before allowing a merge.
        /// </summary>
        private bool HasParallelLaneWindow(
            CompCar car,
            List<Util.Waypoint> fromPath,
            int fromIndex,
            List<Util.Waypoint> targetPath)
        {
            if (car == null || car.PLID == 0) return false;
            if (fromPath == null || fromPath.Count == 0) return false;
            if (targetPath == null || targetPath.Count == 0) return false;

            var speedKmh = 360.0 * car.Speed / 32768.0;
            var lookaheadMeters = Math.Max(
                config.LaneChangeParallelLookaheadMinMeters,
                (speedKmh / 3.6) * config.LaneChangeParallelLookaheadSeconds);

            var samples = new[] { 0.25, 0.5, 0.75, 1.0 };
            foreach (var fraction in samples)
            {
                var distanceAhead = lookaheadMeters * fraction;
                var (sampleX, sampleY, sampleIndex) = GetPathPointAhead(fromPath, fromIndex, distanceAhead);
                var targetIndex = FindClosestIndex(targetPath, sampleX, sampleY);
                var targetPoint = targetPath[targetIndex].Position;

                var dx = targetPoint.X / 65536.0 - sampleX;
                var dy = targetPoint.Y / 65536.0 - sampleY;
                var separation = Math.Sqrt(dx * dx + dy * dy);
                if (separation > config.LaneChangeMaxParallelDistanceMeters)
                    return false;

                var fromDir = GetDirectionVector(fromPath, sampleIndex);
                var targetDir = GetDirectionVector(targetPath, targetIndex);
                var headingDelta = CalculateHeadingDeltaDegrees(fromDir, targetDir);
                if (headingDelta > config.LaneChangeMaxParallelHeadingDegrees)
                    return false;
            }

            return true;
        }

        /// <summary>
        /// Verify that the target lane segment is free of nearby cars before merging.
        /// </summary>
        private bool IsTargetLaneClear(
            byte plid,
            CompCar mergingCar,
            List<Util.Waypoint> targetPath,
            int targetIndex,
            TrafficCarSnapshot[] snapshot)
        {
            if (targetPath == null || targetPath.Count == 0) return false;

            var geometry = PathProjection.GetGeometry(targetPath, config.PathLoopClosureDistanceMeters);
            if (geometry == null) return false;

            var targetPoint = GetWaypointPosition(targetPath, targetIndex);
            if (!PathProjection.TryProjectToPath(targetPath, geometry, targetPoint.x, targetPoint.y,
                    out var mergeProjection))
                return false;

            var checkDistance = Math.Max(1.0, config.LaneChangeSafetyCheckDistanceMeters);
            var rearCheckDistance = Math.Max(checkDistance, config.LaneChangeRearCheckDistanceMeters);
            var halfWidth = Math.Max(config.TrafficLaneHalfWidthMeters, config.LaneChangeSafetyCheckHalfWidthMeters);

            var mergeSpeedMps = (100.0 * mergingCar.Speed) / 32768.0;
            var mergeDir = GetHeadingVector(mergingCar.Direction);
            var mergeForwardSpeed =
                mergeSpeedMps * (mergeDir.x * mergeProjection.DirectionX + mergeDir.y * mergeProjection.DirectionY);
            if (mergeForwardSpeed < 0) mergeForwardSpeed = 0;

            if (snapshot == null || snapshot.Length == 0) return true;

            foreach (var car in snapshot)
            {
                if (car.PLID == 0 || car.PLID == plid) continue;

                if (!PathProjection.TryProjectToPath(targetPath, geometry, car.XMeters, car.YMeters,
                        out var otherProjection))
                    continue;

                if (Math.Abs(otherProjection.LateralOffsetMeters) > halfWidth) continue;

                var forwardDistance = PathProjection.GetForwardDistance(
                    geometry,
                    mergeProjection.DistanceAlongPathMeters,
                    otherProjection.DistanceAlongPathMeters);
                var backwardDistance = PathProjection.GetBackwardDistance(
                    geometry,
                    mergeProjection.DistanceAlongPathMeters,
                    otherProjection.DistanceAlongPathMeters);

                var spacingFactor = car.IsAi ? config.TrafficAiSpacingFactor : config.TrafficHumanSpacingFactor;
                var desiredGap =
                    (config.TrafficBaseGapMeters + mergeSpeedMps * config.TrafficTimeHeadwaySeconds) *
                    spacingFactor * Math.Max(0.1, config.LaneChangeMergeGapFactor);

                if (forwardDistance >= 0 &&
                    forwardDistance <= checkDistance &&
                    forwardDistance < desiredGap)
                {
                    if (laneChangeDetailedLogging)
                        logger.Log(
                            $"PLID={plid} Lane change blocked: car ahead (gap={forwardDistance:F1}m < {desiredGap:F1}m)");
                    return false;
                }

                if (backwardDistance >= 0 && backwardDistance <= rearCheckDistance)
                {
                    var otherDir = GetHeadingVector(car.Direction);
                    var otherForwardSpeed =
                        car.SpeedMps *
                        (otherDir.x * otherProjection.DirectionX + otherDir.y * otherProjection.DirectionY);
                    if (otherForwardSpeed < 0) otherForwardSpeed = 0;

                    var closingSpeed = otherForwardSpeed - mergeForwardSpeed;
                    var rearTtc = closingSpeed > 0.1 ? backwardDistance / closingSpeed : double.PositiveInfinity;
                    var rearTtcLimit = config.LaneChangeRearCheckTtcSeconds * spacingFactor;

                    if (backwardDistance < desiredGap || rearTtc < rearTtcLimit)
                    {
                        if (laneChangeDetailedLogging)
                            logger.Log(
                                $"PLID={plid} Lane change blocked: car behind (gap={backwardDistance:F1}m, ttc={rearTtc:F1}s)");
                        return false;
                    }
                }
            }

            return true;
        }

        /// <summary>
        /// Determine whether the AI should stop on the spawn route until a safe merge opens.
        /// </summary>
        private bool ShouldHoldForSpawnMerge(byte plid, CompCar car, int targetIndex, TrafficCarSnapshot[] snapshot)
        {
            if (pathManager.SpawnRoute == null || pathManager.SpawnRoute.Count < 2) return false;
            if (pathManager.MainRoute == null || pathManager.MainRoute.Count < 2) return false;
            if (!IsSpawnMergeWindow(car, targetIndex)) return false;

            var mergeIndex = FindClosestIndex(pathManager.MainRoute, car);
            return !IsTargetLaneClear(plid, car, pathManager.MainRoute, mergeIndex, snapshot);
        }

        /// <summary>
        /// Check whether the car is within the configured hold window near the end of the spawn route.
        /// </summary>
        private bool IsSpawnMergeWindow(CompCar car, int targetIndex)
        {
            if (pathManager.SpawnRoute == null || pathManager.SpawnRoute.Count < 2) return false;

            var holdLookahead = Math.Max(1, config.SpawnMergeHoldLookaheadWaypoints);
            if (targetIndex >= pathManager.SpawnRoute.Count - holdLookahead) return true;

            var geometry = PathProjection.GetGeometry(pathManager.SpawnRoute, config.PathLoopClosureDistanceMeters);
            if (geometry == null || geometry.TotalLength <= 0) return false;

            if (!PathProjection.TryProjectToPath(
                    pathManager.SpawnRoute,
                    geometry,
                    car.X / 65536.0,
                    car.Y / 65536.0,
                    out var projection))
                return false;

            if (geometry.IsLoop) return false;

            var distanceToEnd = geometry.TotalLength - projection.DistanceAlongPathMeters;
            return distanceToEnd <= config.SpawnMergeHoldDistanceMeters;
        }

        /// <summary>
        /// Build a smooth Bezier transition between the current and target lanes using the AI's live position and heading.
        /// </summary>
        private List<Util.Waypoint> BuildLaneChangeTransition(
            CompCar car,
            List<Util.Waypoint> fromPath,
            int fromIndex,
            List<Util.Waypoint> targetPath,
            int targetIndex)
        {
            var transition = new List<Util.Waypoint>();
            if (car.PLID == 0 || targetPath == null || targetPath.Count == 0) return transition;

            var startX = car.X / 65536.0;
            var startY = car.Y / 65536.0;
            var endPoint = GetWaypointPosition(targetPath, targetIndex);
            var startDirection = GetDirectionVector(fromPath, fromIndex);
            if (Math.Abs(startDirection.x) < 0.001 && Math.Abs(startDirection.y) < 0.001)
                startDirection = NormalizeVector(endPoint.x - startX, endPoint.y - startY);
            var endDirection = GetDirectionVector(targetPath, targetIndex);
            if (Math.Abs(endDirection.x) < 0.001 && Math.Abs(endDirection.y) < 0.001)
                endDirection = NormalizeVector(endPoint.x - startX, endPoint.y - startY);

            var dx = endPoint.x - startX;
            var dy = endPoint.y - startY;
            var straightDistance = Math.Max(0.1, Math.Sqrt(dx * dx + dy * dy));
            var handleLength = Math.Max(
                3.0,
                Math.Min(config.LaneChangeTransitionLengthMeters, straightDistance + config.LaneChangeTransitionLengthMeters * 0.5));

            var control1 = (x: startX + startDirection.x * handleLength, y: startY + startDirection.y * handleLength);
            var control2 = (x: endPoint.x - endDirection.x * handleLength, y: endPoint.y - endDirection.y * handleLength);

            var points = Math.Max(4, config.LaneChangeTransitionPointCount);
            var speedLimit = Math.Min(
                config.LaneChangeMaxMergeSpeedKmh,
                Math.Min(GetWaypointSpeedLimit(fromPath, fromIndex), GetWaypointSpeedLimit(targetPath, targetIndex)));
            var routeIndex = targetPath[targetIndex].RouteIndex;

            for (var i = 0; i < points; i++)
            {
                var t = i / (double)(points - 1);
                var mt = 1 - t;
                var x = mt * mt * mt * startX +
                        3 * mt * mt * t * control1.x +
                        3 * mt * t * t * control2.x +
                        t * t * t * endPoint.x;
                var y = mt * mt * mt * startY +
                        3 * mt * mt * t * control1.y +
                        3 * mt * t * t * control2.y +
                        t * t * t * endPoint.y;

                transition.Add(new Util.Waypoint(x, y, speedLimit, routeIndex));
            }

            return transition;
        }

        /// <summary>
        /// Pick a forward-looking waypoint on the target lane so merges aim several nodes ahead instead of snapping backward.
        /// </summary>
        private (int entryIndex, double distance, double headingErrorDeg) SelectLaneChangeTargetIndex(
            CompCar car,
            List<Util.Waypoint> fromPath,
            int fromIndex,
            List<Util.Waypoint> targetPath)
        {
            if (car == null || car.PLID == 0) return (0, double.MaxValue, 180);
            if (fromPath == null || fromPath.Count == 0) return (0, double.MaxValue, 180);
            if (targetPath == null || targetPath.Count == 0) return (0, double.MaxValue, 180);

            var lookaheadPoints = Math.Max(1, config.LaneChangeTargetAheadWaypoints);
            var anchorIndex = ClampIndex(fromPath, fromIndex + lookaheadPoints);
            var anchorPoint = GetWaypointPosition(fromPath, anchorIndex);
            var preferredIndex = FindClosestIndex(targetPath, anchorPoint.x, anchorPoint.y);
            var searchRadius = Math.Max(lookaheadPoints * 2, 8);

            var (alignedIndex, distance, headingError) = FindAlignedWaypointIndex(
                targetPath,
                car,
                preferredIndex,
                searchRadius);

            var entryIndex = ClampIndex(targetPath, Math.Max(alignedIndex, preferredIndex));
            return (entryIndex, distance, headingError);
        }

        /// <summary>
        /// Begin signalling for a planned lane change.
        /// </summary>
        private void StartLaneChangeIndicator(
            byte plid,
            List<Util.Waypoint> fromPath,
            int fromIndex,
            List<Util.Waypoint> targetPath,
            int targetIndex,
            string description)
        {
            var indicator = DetermineIndicatorDirection(fromPath, fromIndex, targetPath, targetIndex);
            var holdSeconds = config.LaneChangeSignalLeadTimeSeconds + config.LaneChangeSignalMinimumDurationSeconds;
            lightController.SetIndicators(plid, indicator, TimeSpan.FromSeconds(holdSeconds));
            laneChangeSignalStart[plid] = DateTime.Now;
            logger.Log($"PLID={plid} ROUTE CHANGE: {description} using indicator {indicator}");
        }

        /// <summary>
        /// Clear indicators once the lane change has completed and the minimum duration elapsed.
        /// </summary>
        private void CompleteLaneChangeIndicators(byte plid)
        {
            if (!laneChangeSignalStart.TryGetValue(plid, out var started))
            {
                lightController.CancelIndicators(plid);
                return;
            }

            var minHold = TimeSpan.FromSeconds(config.LaneChangeSignalMinimumDurationSeconds);
            if (DateTime.Now - started >= minHold)
            {
                lightController.CancelIndicators(plid);
                laneChangeSignalStart.Remove(plid);
            }
        }

        /// <summary>
        /// Safely clamp a waypoint index to the bounds of a path.
        /// </summary>
        private static int ClampIndex(List<Util.Waypoint> path, int index)
        {
            if (path == null || path.Count == 0) return 0;
            return Math.Max(0, Math.Min(path.Count - 1, index));
        }

        /// <summary>
        /// Get a usable speed limit from a waypoint list with safe fallbacks.
        /// </summary>
        private static double GetWaypointSpeedLimit(List<Util.Waypoint> path, int index)
        {
            if (path == null || path.Count == 0) return 50.0;
            var clamped = ClampIndex(path, index);
            return path[clamped].SpeedLimit <= 0 ? 50.0 : path[clamped].SpeedLimit;
        }

        /// <summary>
        /// Kick off a pending lane change after the indicator lead time has elapsed, rebuilding the transition so it
        /// uses the AI’s current position and heading.
        /// </summary>
        private void StartLaneChangeTransition(byte plid, LaneChangeRequest request)
        {
            if (request == null || request.TargetPath == null || request.TargetPath.Count == 0)
            {
                pendingLaneChanges.Remove(plid);
                return;
            }

            var targetPath = request.TargetPath;
            var currentPath = waypointFollower.GetPath(plid);
            if (currentPath == null || currentPath.Count == 0)
                currentPath = request.ToAlternate ? pathManager.MainRoute : targetPath;

            var car = Array.Find(allCars, c => c.PLID == plid);
            if (car == null)
            {
                pendingLaneChanges.Remove(plid);
                return;
            }
            var fromIndex = ClampIndex(currentPath, FindClosestIndex(currentPath, car));
            var anchorIndex = Math.Max(fromIndex, ClampIndex(currentPath, request.ScheduledFromIndex));
            var (entryIndex, distance, headingError) = SelectLaneChangeTargetIndex(
                car,
                currentPath,
                anchorIndex,
                targetPath);

            if (distance > config.LaneChangeMaxParallelDistanceMeters ||
                headingError > config.LaneChangeMaxParallelHeadingDegrees)
            {
                LogLaneChangeSkip(plid,
                    $"transition rebuild blocked - distance {distance:F1}m or heading {headingError:F1}° beyond limits");
                pendingLaneChanges.Remove(plid);
                return;
            }

            var transitionPath = BuildLaneChangeTransition(car, currentPath, fromIndex, targetPath, entryIndex);
            if (transitionPath.Count == 0)
            {
                LogLaneChangeSkip(plid, "transition rebuild failed (no points)");
                pendingLaneChanges.Remove(plid);
                return;
            }

            waypointFollower.SetPath(plid, transitionPath, 0);
            var transitionSpeed = Math.Min(
                config.LaneChangeMaxMergeSpeedKmh,
                Math.Min(GetWaypointSpeedLimit(currentPath, fromIndex), GetWaypointSpeedLimit(targetPath, entryIndex)));
            waypointFollower.SetManualTargetSpeed(plid, transitionSpeed);
            activeLaneChanges[plid] = new ActiveLaneChange
            {
                TargetRoute = request.TargetRoute,
                ToAlternate = request.ToAlternate,
                TargetEntryIndex = entryIndex,
                TransitionPathCount = transitionPath.Count
            };

            pendingLaneChanges.Remove(plid);
            logger.Log(
                $"PLID={plid} Lane change executing: {request.Description} (from idx {fromIndex} to target {entryIndex}, {transitionPath.Count} pts)");
        }

        /// <summary>
        /// Complete a lane change by swapping from the transition path to the destination lane.
        /// </summary>
        private void CompleteLaneChangePath(byte plid, ActiveLaneChange activeChange)
        {
            if (activeChange == null) return;

            var targetPath = activeChange.ToAlternate
                ? activeChange.TargetRoute?.Path
                : pathManager.MainRoute;

            if (targetPath == null || targetPath.Count == 0) return;

            var startIndex = ClampIndex(targetPath, activeChange.TargetEntryIndex);
            waypointFollower.SetPath(plid, targetPath, startIndex);

            if (activeChange.ToAlternate)
            {
                currentRoute[plid] = "branch";
                if (activeChange.TargetRoute != null)
                    activeBranchSelections[plid] = activeChange.TargetRoute;
            }
            else
            {
                currentRoute[plid] = "main";
                activeBranchSelections.Remove(plid);
            }

            waypointFollower.ClearManualTargetSpeed(plid);
            laneChangeLastMerge[plid] = DateTime.Now;
            laneChangeLastCheck[plid] = DateTime.Now;
            CompleteLaneChangeIndicators(plid);
        }

        /// <summary>
        /// Advance pending and active lane change state machines. Returns true if a lane change is underway.
        /// </summary>
        private bool HandleLaneChangeState(byte plid)
        {
            if (pendingLaneChanges.TryGetValue(plid, out var pending) &&
                DateTime.Now >= pending.ExecuteAt)
            {
                StartLaneChangeTransition(plid, pending);
            }

            if (activeLaneChanges.TryGetValue(plid, out var active))
            {
                var (targetIndex, count, _, _) = waypointFollower.GetFollowerInfo(plid);
                if (count == 0)
                {
                    activeLaneChanges.Remove(plid);
                    return false;
                }

                if (count != active.TransitionPathCount)
                {
                    activeLaneChanges.Remove(plid);
                    return false;
                }

                if (targetIndex >= active.TransitionPathCount - 2)
                {
                    CompleteLaneChangePath(plid, active);
                    activeLaneChanges.Remove(plid);
                    return false;
                }

                return true;
            }

            return pendingLaneChanges.ContainsKey(plid);
        }

        /// <summary>
        /// Opportunistically merge onto the alternate main lane when nearby instead of forcing the switch at the branch start.
        /// </summary>
        private bool TryJoinInnerBranchLaneChange(byte plid, CompCar car, int mainRouteIndex)
        {
            if (!currentRoute.TryGetValue(plid, out var routeName) || routeName != "main")
                return false;

            if (pendingLaneChanges.ContainsKey(plid) || activeLaneChanges.ContainsKey(plid))
                return false;

            var innerBranch = pathManager.GetAlternateMainRoute();

            if (innerBranch == null || innerBranch.Path == null || innerBranch.Path.Count == 0)
            {
                LogLaneChangeSkip(plid, "no alternate lane loaded");
                return false;
            }

            if (!ShouldRunLaneChangeCheck(plid))
                return false;

            var speedKmh = 360.0 * car.Speed / 32768.0;
            var checkLabel = $"PLID={plid} Lane change main->alt";
            if (laneChangeDetailedLogging)
                logger.Log($"{checkLabel}: checking (speed={speedKmh:F1} km/h)");
            if (speedKmh > config.LaneChangeMaxMergeSpeedKmh)
            {
                if (laneChangeDetailedLogging)
                    logger.Log(
                        $"{checkLabel}: blocked - speed above limit {config.LaneChangeMaxMergeSpeedKmh:F1} km/h");
                return false;
            }

            if (!HasParallelLaneWindow(car, pathManager.MainRoute, mainRouteIndex, innerBranch.Path))
            {
                LogLaneChangeSkip(plid, "main->alt blocked - lanes not parallel over lookahead window");
                return false;
            }

            var (entryIndex, distance, headingError) = SelectLaneChangeTargetIndex(
                car,
                pathManager.MainRoute,
                mainRouteIndex,
                innerBranch.Path);

            if (distance > config.LaneChangeMaxParallelDistanceMeters ||
                headingError > config.LaneChangeMaxParallelHeadingDegrees)
            {
                if (laneChangeDetailedLogging)
                    logger.Log(
                        $"{checkLabel}: blocked - distance {distance:F1}m or heading {headingError:F1}° exceeds thresholds (max {config.LaneChangeMaxParallelDistanceMeters:F1}m / {config.LaneChangeMaxParallelHeadingDegrees:F1}°)");
                return false;
            }

            if (!IsTargetLaneClear(plid, car, innerBranch.Path, entryIndex, trafficSnapshot))
                return false;

            var request = new LaneChangeRequest
            {
                Description = $"Taking branch {innerBranch.Name}",
                TargetRoute = innerBranch,
                TargetPath = innerBranch.Path,
                ToAlternate = true,
                ExecuteAt = DateTime.Now.AddSeconds(config.LaneChangeSignalLeadTimeSeconds),
                ScheduledFromIndex = mainRouteIndex
            };

            logger.Log(
                $"{checkLabel}: scheduled merge to {innerBranch.Name} at entry {entryIndex} after {config.LaneChangeSignalLeadTimeSeconds:F1}s lead");
            pendingLaneChanges[plid] = request;
            StartLaneChangeIndicator(
                plid,
                pathManager.MainRoute,
                mainRouteIndex,
                innerBranch.Path,
                entryIndex,
                request.Description);
            return true;
        }

        /// <summary>
        /// Evaluate a branch end rejoin to the main lane with full safety checks and scheduled signaling.
        /// </summary>
        private bool TryScheduleBranchEndRejoin(
            byte plid,
            CompCar car,
            int branchIndex,
            List<Util.Waypoint> branchPath,
            BranchRouteInfo? branchInfo,
            int preferredMainIndex)
        {
            if (branchPath == null || branchPath.Count == 0) return false;
            if (pendingLaneChanges.ContainsKey(plid) || activeLaneChanges.ContainsKey(plid)) return true;

            var speedKmh = 360.0 * car.Speed / 32768.0;
            var checkLabel = $"PLID={plid} Branch end rejoin";

            var (mainIndex, distance, headingError) = FindAlignedWaypointIndex(
                pathManager.MainRoute,
                car,
                ClampIndex(pathManager.MainRoute, preferredMainIndex),
                8);

            if (speedKmh > config.LaneChangeMaxMergeSpeedKmh)
            {
                if (laneChangeDetailedLogging)
                    logger.Log(
                        $"{checkLabel}: holding - speed {speedKmh:F1} km/h above limit {config.LaneChangeMaxMergeSpeedKmh:F1} km/h");
                return false;
            }

            if (distance > config.LaneChangeMaxParallelDistanceMeters ||
                headingError > config.LaneChangeMaxParallelHeadingDegrees)
            {
                if (laneChangeDetailedLogging)
                    logger.Log(
                        $"{checkLabel}: holding - distance {distance:F1}m or heading {headingError:F1}° exceeds thresholds (max {config.LaneChangeMaxParallelDistanceMeters:F1}m / {config.LaneChangeMaxParallelHeadingDegrees:F1}°)");
                return false;
            }

            if (!IsTargetLaneClear(plid, car, pathManager.MainRoute, mainIndex, trafficSnapshot))
                return false;

            var request = new LaneChangeRequest
            {
                Description = $"Rejoining main from {branchInfo?.Name ?? "branch"}",
                TargetRoute = null,
                TargetPath = pathManager.MainRoute,
                ToAlternate = false,
                ExecuteAt = DateTime.Now.AddSeconds(config.LaneChangeSignalLeadTimeSeconds),
                ScheduledFromIndex = branchIndex
            };

            pendingLaneChanges[plid] = request;
            StartLaneChangeIndicator(
                plid,
                branchPath,
                branchIndex,
                pathManager.MainRoute,
                mainIndex,
                request.Description);

            logger.Log(
                $"{checkLabel}: scheduled rejoin to main at entry {mainIndex} after {config.LaneChangeSignalLeadTimeSeconds:F1}s lead");
            return true;
        }

        /// <summary>
        /// Attempt a lane change from the alternate branch back to the main lane with cooldowns.
        /// </summary>
        private bool TrySwapInnerToMainLane(byte plid, CompCar car, int branchTargetIndex, List<Util.Waypoint> branchPath)
        {
            if (branchPath == null || branchPath.Count == 0) return false;

            if (!activeBranchSelections.TryGetValue(plid, out var branchInfo) || !IsInnerBranch(branchInfo))
                return false;

            var isAlternateMain = branchInfo?.Metadata?.Type == RouteType.AlternateMain ||
                                  branchInfo?.Metadata?.IsLoop == true;

            if (isAlternateMain &&
                TryGetAssignedRoute(plid, out var assignedRoute) &&
                !string.IsNullOrWhiteSpace(assignedRoute) &&
                assignedRoute.Equals(branchInfo?.Name ?? string.Empty, StringComparison.OrdinalIgnoreCase))
            {
                LogLaneChangeSkip(plid, $"alt->main blocked - assigned to {branchInfo?.Name}");
                return false;
            }

            if (pendingLaneChanges.ContainsKey(plid) || activeLaneChanges.ContainsKey(plid))
                return false;

            if (!ShouldRunLaneChangeCheck(plid))
                return false;

            var speedKmh = 360.0 * car.Speed / 32768.0;
            var checkLabel = $"PLID={plid} Lane change alt->main";
            if (laneChangeDetailedLogging)
                logger.Log($"{checkLabel}: checking (speed={speedKmh:F1} km/h)");
            if (speedKmh > config.LaneChangeMaxMergeSpeedKmh)
            {
                if (laneChangeDetailedLogging)
                    logger.Log(
                        $"{checkLabel}: blocked - speed above limit {config.LaneChangeMaxMergeSpeedKmh:F1} km/h");
                return false;
            }

            if (!HasParallelLaneWindow(car, branchPath, branchTargetIndex, pathManager.MainRoute))
            {
                LogLaneChangeSkip(plid, "alt->main blocked - lanes not parallel over lookahead window");
                return false;
            }

            var (mainIndex, distance, headingError) = SelectLaneChangeTargetIndex(
                car,
                branchPath,
                branchTargetIndex,
                pathManager.MainRoute);

            if (distance > config.LaneChangeMaxParallelDistanceMeters ||
                headingError > config.LaneChangeMaxParallelHeadingDegrees)
            {
                if (laneChangeDetailedLogging)
                    logger.Log(
                        $"{checkLabel}: blocked - distance {distance:F1}m or heading {headingError:F1}° exceeds thresholds (max {config.LaneChangeMaxParallelDistanceMeters:F1}m / {config.LaneChangeMaxParallelHeadingDegrees:F1}°)");
                return false;
            }

            if (!IsTargetLaneClear(plid, car, pathManager.MainRoute, mainIndex, trafficSnapshot))
                return false;

            var request = new LaneChangeRequest
            {
                Description = $"Inner lane change to main",
                TargetRoute = null,
                TargetPath = pathManager.MainRoute,
                ToAlternate = false,
                ExecuteAt = DateTime.Now.AddSeconds(config.LaneChangeSignalLeadTimeSeconds),
                ScheduledFromIndex = branchTargetIndex
            };

            logger.Log(
                $"{checkLabel}: scheduled merge to main at entry {mainIndex} after {config.LaneChangeSignalLeadTimeSeconds:F1}s lead");
            pendingLaneChanges[plid] = request;
            StartLaneChangeIndicator(
                plid,
                branchPath,
                branchTargetIndex,
                pathManager.MainRoute,
                mainIndex,
                request.Description);
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
            var durationSeconds = Math.Max(3.0, config.LaneChangeSignalMinimumDurationSeconds);
            lightController.SetIndicators(plid, indicator, TimeSpan.FromSeconds(durationSeconds));
            logger.Log($"PLID={plid} ROUTE CHANGE: {description} using indicator {indicator}");
        }

        /// <summary>
        /// Convert a car heading into a normalized 2D direction vector.
        /// </summary>
        private static (double x, double y) GetHeadingVector(int heading)
        {
            var radians = (heading + CoordinateUtils.QUARTER_CIRCLE) * 2 * Math.PI / CoordinateUtils.FULL_CIRCLE;
            return (Math.Cos(radians), Math.Sin(radians));
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

        /// <summary>
        /// Emit a throttled log explaining why a lane-change attempt was skipped.
        /// </summary>
        private void LogLaneChangeSkip(byte plid, string reason)
        {
            if (!laneChangeDetailedLogging) return;

            var now = DateTime.Now;
            if (laneChangeSkipLog.TryGetValue(plid, out var info))
            {
                var sameReason = string.Equals(info.Reason, reason, StringComparison.OrdinalIgnoreCase);
                if (sameReason && now - info.LastLogged < TimeSpan.FromSeconds(10))
                    return;
            }

            laneChangeSkipLog[plid] = new LaneChangeSkipInfo
            {
                LastLogged = now,
                Reason = reason
            };

            logger.Log($"PLID={plid} Lane change check skipped: {reason}");
        }

        /// <summary>
        /// Emit a throttled log explaining why a route assignment was applied or deferred.
        /// </summary>
        private void LogRouteAssignmentDecision(byte plid, string reason)
        {
            var now = DateTime.Now;
            if (routeAssignmentLog.TryGetValue(plid, out var info))
            {
                var sameReason = string.Equals(info.Reason, reason, StringComparison.OrdinalIgnoreCase);
                if (sameReason && now - info.LastLogged < TimeSpan.FromSeconds(10))
                    return;
            }

            routeAssignmentLog[plid] = new RouteAssignmentLogInfo
            {
                LastLogged = now,
                Reason = reason
            };

            logger.Log($"PLID={plid} Route assignment: {reason}");
        }

        public void SetNumberOfAIs(int count)
        {
            config.NumberOfAIs = Math.Max(0, count);
        }

        /// <summary>
        /// Apply population manager configuration values from app settings.
        /// </summary>
        public void ConfigurePopulationManager(
            int maxPlayers,
            int reservedSlots,
            double aiFillRatio,
            int minAis,
            int maxAis,
            int adjustIntervalMs,
            int spawnBatchSize,
            int removeBatchSize)
        {
            config.MaxPlayers = maxPlayers;
            config.ReservedSlots = reservedSlots;
            config.AiFillRatio = aiFillRatio;
            config.MinAIs = minAis;
            config.MaxAIs = maxAis;
            config.AdjustIntervalMs = adjustIntervalMs;
            config.SpawnBatchSize = spawnBatchSize;
            config.RemoveBatchSize = removeBatchSize;
            populationManager?.RefreshSettings();
        }

        /// <summary>
        /// Enable or disable automatic AI population adjustments.
        /// </summary>
        public void SetAutoPopulationEnabled(bool enabled, bool reconcileNow = false)
        {
            config.AutoManagePopulation = enabled;
            populationManager?.SetAutoAdjustEnabled(enabled, reconcileNow);
            mainUI.SetAutoPopulationState(enabled);
            logger.Log(enabled ? "Auto AI population management enabled." : "Auto AI population management paused.");
        }

        /// <summary>
        /// Enable automatic AI population adjustments and reconcile immediately.
        /// </summary>
        public void EnableAutoPopulation()
        {
            SetAutoPopulationEnabled(true, true);
        }

        /// <summary>
        /// Report whether automatic AI population adjustments are active.
        /// </summary>
        public bool IsAutoPopulationEnabled => populationManager?.AutoAdjustEnabled ?? false;

        /// <summary>
        /// Request an immediate population reconciliation.
        /// </summary>
        public void RecalculatePopulationTargets()
        {
            populationManager?.RequestReconcile("manual");
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
            SpawnAICarsInternal(config.NumberOfAIs, true);
        }

        /// <summary>
        /// Spawn AI cars according to a planned route list for population balancing.
        /// </summary>
        public void SpawnPlannedAICars(List<string> routes, bool useConfiguredDelay)
        {
            if (routes == null || routes.Count == 0) return;

            lock (plannedSpawnLock)
            {
                foreach (var route in routes)
                {
                    var normalized = routeLibrary.NormalizeRouteName(
                        string.IsNullOrWhiteSpace(route) ? config.MainRouteName : route);
                    plannedSpawnRoutes.Enqueue(normalized);
                }
            }

            SpawnAICarsInternal(routes.Count, useConfiguredDelay);
        }

        private void SpawnAICarsInternal(int count, bool useConfiguredDelay)
        {
            if (count <= 0) return;
            if (!insim.IsConnected)
            {
                logger.LogWarning("Cannot spawn AI cars: InSim is not connected.");
                return;
            }

            if (!spawnSemaphore.Wait(0))
            {
                logger.LogWarning("AI spawn already in progress; ignoring duplicate request.");
                return;
            }

            Task.Run(async () =>
            {
                try
                {
                    if (insimDebugLogging)
                    {
                        logger.Log(
                            $"Spawn sequence starting: count={count}, useConfiguredDelay={useConfiguredDelay}, plannedRoutes={plannedSpawnRoutes.Count}");
                    }
                    await SpawnAICarsAsync(count, useConfiguredDelay).ConfigureAwait(false);
                    if (insimDebugLogging)
                    {
                        logger.Log("Spawn sequence completed");
                    }
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
        private async Task SpawnAICarsAsync(int count, bool useConfiguredDelay)
        {
            var delayMs = useConfiguredDelay ? config.WaitTimeToSpawn : Math.Min(1000, config.WaitTimeToSpawn);
            delayMs = Math.Max(100, delayMs);

            for (var i = 0; i < count; i++)
            {
                if (!insim.IsConnected)
                {
                    logger.LogWarning("Stopped spawning AI cars: InSim connection lost.");
                    return;
                }
                insim.Send(new IS_MST { Msg = "/ai" });
                logger.Log($"Spawned AI {i + 1} of {count}");
                if (i < count - 1)
                    await Task.Delay(delayMs).ConfigureAwait(false);
            }
        }

        /// <summary>
        /// Get the next planned route for a freshly spawned AI or default to the main route.
        /// </summary>
        private string DequeuePlannedRouteOrDefault()
        {
            lock (plannedSpawnLock)
            {
                if (plannedSpawnRoutes.Count > 0)
                    return plannedSpawnRoutes.Dequeue();

                return config.MainRouteName;
            }
        }

        public void RemoveLastAICar()
        {
            byte plid;
            lock (aiPlidLock)
            {
                if (aiPLIDs.Count == 0) return;
                plid = aiPLIDs[aiPLIDs.Count - 1];
            }
            RemoveAICar(plid);
        }

        public void RemoveAICar(byte plid, bool notifyPopulationManager = true)
        {
            lock (aiPlidLock)
            {
                if (!aiPLIDs.Contains(plid) || plid == 0) return;
            }
            var aiNameForCmd = GetAINameForCommand(plid);
            if (!string.IsNullOrWhiteSpace(aiNameForCmd))
            {
                insim.Send(new IS_MST { Msg = $"/spec {aiNameForCmd}" });
            }
            ForgetAi(plid, notifyPopulationManager);
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
            foreach (var plid in GetActiveAiPlids())
            {
                RemoveAICar(plid);
            }
        }

        /// <summary>
        /// Clear local tracking for an AI and optionally notify the population manager.
        /// </summary>
        private void ForgetAi(byte plid, bool notifyPopulationManager = true)
        {
            lock (aiPlidLock)
            {
                aiPLIDs.Remove(plid);
            }
            aiSpawnPositions.Remove(plid);
            engineStateMap.Remove(plid);
            currentRoute.Remove(plid);
            activeBranchSelections.Remove(plid);
            aiNames.Remove(plid);
            lock (assignmentLock)
            {
                aiAssignedRoutes.Remove(plid);
            }
            laneChangeLastCheck.Remove(plid);
            laneChangeLastMerge.Remove(plid);
            pendingLaneChanges.Remove(plid);
            activeLaneChanges.Remove(plid);
            laneChangeSignalStart.Remove(plid);
            laneChangeSkipLog.Remove(plid);
            passByCooldowns.Remove(plid);
            passByProximityStates.Remove(plid);
            RemoveCarState(plid);
            mainUI.UpdateAIList(GetAiTuples());
            if (notifyPopulationManager)
                populationManager?.OnAiLeft(plid);
        }

        public void StopAllAIs()
        {
            foreach (var plid in GetActiveAiPlids())
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
            foreach (var plid in GetActiveAiPlids())
            {
                waypointFollower.ClearManualTargetSpeed(plid);
                driver.StartCar(plid);
            }
        }

        public void SpectateAllAIs()
        {
            foreach (var (plid, name) in GetAiTuples())
            {
                var formattedName = FormatNameForCommand(name);
                if (string.IsNullOrWhiteSpace(formattedName)) continue;

                insim.Send(new IS_MST { Msg = $"/spec {formattedName}" });
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
            foreach (var plid in GetActiveAiPlids())
            {
                waypointFollower.SetManualTargetSpeed(plid, speed);
            }
        }

        /// <summary>
        /// Request a lane change onto a specific recorded branch route and signal accordingly.
        /// </summary>
        public bool TrySwitchToBranch(byte plid, string branchName)
        {
            lock (aiPlidLock)
            {
                if (!aiPLIDs.Contains(plid)) return false;
            }
            if (!pathManager.TryGetBranchByName(branchName, out var branchInfo))
            {
                logger.LogWarning($"No branch named {branchName} available for lane change.");
                return false;
            }
            if (branchInfo == null) return false;

            var car = Array.Find(allCars, c => c.PLID == plid);
            if (car == null || car.PLID == 0)
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
            playerNames[npl.PLID] = npl.PName;
            if ((npl.PType & PlayerTypes.PLT_AI) == PlayerTypes.PLT_AI &&
                !npl.PName.Contains("Track")) // and not tracks
            {
                var plid = npl.PLID;
                var isNewAi = false;

                lock (aiPlidLock)
                {
                    if (!aiPLIDs.Contains(plid))
                    {
                        aiPLIDs.Add(plid);
                        isNewAi = true;
                    }
                }

                if (isNewAi)
                {
                    // Add to our list of AIs
                    aiSpawnPositions[plid] = new Vec();
                    engineStateMap[plid] = true; // Assume engine starts running
                    aiNames[plid] = npl.PName;
                    var assignedRoute = DequeuePlannedRouteOrDefault();

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
                    SetAssignedRoute(plid, assignedRoute);
                    populationManager?.UnregisterHuman(0, npl.UCID);
                    populationManager?.OnAiJoined(plid);

                    // Set debug UI to track this AI if enabled
                    if (config.DebugEnabled && debugUI != null && debugUIInitialized)
                        debugUI.SetAIPLID(plid);

                    mainUI.UpdateAIList(GetAiTuples());
                }
            }
            else
            {
                populationManager?.RegisterHuman(npl.UCID, npl.PLID);
                if (config.DebugEnabled && debugUI != null && debugUIInitialized)
                {
                    // Track human player in debug UI
                    debugUI.SetPlayerPLID(npl.PLID);
                    playerPLID = npl.PLID;
                    logger.Log($"Human player detected: PLID={npl.PLID}, Name={npl.PName}");
                }
            }
        }

        /// <summary>
        /// Track player leave events for humans and AIs.
        /// </summary>
        public void OnPlayerLeave(IS_PLL pll)
        {
            if (pll == null) return;

            playerNames.Remove(pll.PLID);

            bool isAi;
            lock (aiPlidLock)
            {
                isAi = aiPLIDs.Contains(pll.PLID);
            }

            if (isAi)
            {
                ForgetAi(pll.PLID);
            }
            else
            {
                populationManager?.UnregisterHuman(pll.PLID);
                RemoveCarState(pll.PLID);
            }
        }

        /// <summary>
        /// Track new connections so idle humans are counted even without a car on track.
        /// </summary>
        public void OnNewConnection(IS_NCN ncn)
        {
            if (ncn == null) return;
            populationManager?.RegisterConnection(ncn.UCID);
        }

        /// <summary>
        /// Remove connection tracking when a user disconnects.
        /// </summary>
        public void OnConnectionLeave(IS_CNL cnl)
        {
            if (cnl == null) return;
            populationManager?.OnConnectionClosed(cnl.UCID);
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
            var stopwatch = Stopwatch.StartNew();
            try
            {
                if (mci?.Info == null) return;

                lock (carStateLock)
                {
                    foreach (var car in mci.Info)
                    {
                        carStates[car.PLID] = car;
                    }

                    allCars = carStates.Values.ToArray();
                }
                UpdateTrafficSnapshot();
                ApplyPendingRouteAssignments();

                // Record player route if active
                if (playerPLID != 0)
                {
                    var car = Array.Find(allCars, c => c.PLID == playerPLID);
                    if (car != null && car.PLID != 0 && routeRecorder != null)
                    {
                        try
                        {
                            var pos = new Vec(car.X, car.Y, car.Z);
                            var speed = 360.0 * car.Speed / 32768.0;
                            var angleDiff = (car.Heading - car.Direction) * 360f / 65536f;
                            routeRecorder.AddPoint(pos, speed, playerThrottle, playerBrake, angleDiff, car.Heading, playerPLID);
                        }
                        catch (Exception ex)
                        {
                            logger.LogException(ex, $"Route recording failed for PLID={playerPLID}");
                        }
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
                    var aiSnapshot = GetActiveAiPlids();

                    foreach (var plid in aiSnapshot)
                    {
                        try
                        {
                            var (targetIndex, _, targetSpeed, inRecovery) = waypointFollower.GetFollowerInfo(plid);
                            waypointIndices[plid] = targetIndex;
                            targetSpeeds[plid] = targetSpeed;
                            controlInfo[plid] = driver.GetControlInfo(plid);
                            var state = driver.GetStateDescription(plid);
                            aiStates[plid] = BuildAiStateLabel(plid, state, inRecovery);

                            // Place active waypoint marker if available
                        var path = waypointFollower.GetPath(plid);
                        if (activeWaypointMarkersEnabled && path != null && path.Count > 0)
                        {
                            var now = DateTime.UtcNow;
                            if (!lastActiveWaypointUpdate.TryGetValue(plid, out var lastUpdate) ||
                                (now - lastUpdate).TotalMilliseconds >= activeWaypointIntervalMs)
                            {
                                lastActiveWaypointUpdate[plid] = now;
                                var activeWaypoint = waypointFollower.GetLookaheadWaypoint(plid);
                                lfsLayout.VisualizeActiveWaypoint(plid, activeWaypoint);
                            }
                        }
                        }
                        catch (Exception ex)
                        {
                            logger.LogException(ex, $"Failed to update debug info for AI PLID={plid}");
                        }
                    }

                    // Get paths for all AIs
                    var paths = new Dictionary<byte, List<Util.Waypoint>>();
                    foreach (var plid in aiSnapshot) paths[plid] = waypointFollower.GetPath(plid);

                    debugUI.UpdateDebugInfo(allCars, waypointIndices, paths, targetSpeeds, controlInfo, aiStates);
                }

                EvaluatePassByReactions();
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "OnMCI failed");
            }
            finally
            {
                stopwatch.Stop();
                mciPerformanceTracker.Record(stopwatch.Elapsed);
                LogPerformanceSnapshots();
            }
        }

        /// <summary>
        /// Capture a lightweight traffic snapshot for path-aware spacing checks.
        /// </summary>
        private void UpdateTrafficSnapshot()
        {
            if (allCars == null || allCars.Length == 0)
            {
                trafficSnapshot = Array.Empty<TrafficCarSnapshot>();
                return;
            }

            var aiPlids = GetActiveAiPlids();
            var aiSet = new HashSet<byte>(aiPlids);
            var snapshots = new TrafficCarSnapshot[allCars.Length];
            var count = 0;

            foreach (var car in allCars)
            {
                if (car.PLID == 0) continue;

                snapshots[count++] = new TrafficCarSnapshot
                {
                    PLID = car.PLID,
                    IsAi = aiSet.Contains(car.PLID),
                    XMeters = car.X / 65536.0,
                    YMeters = car.Y / 65536.0,
                    SpeedMps = (100.0 * car.Speed) / 32768.0,
                    Direction = car.Direction,
                    Heading = car.Heading
                };
            }

            if (count == 0)
            {
                trafficSnapshot = Array.Empty<TrafficCarSnapshot>();
                return;
            }

            if (count < snapshots.Length)
                Array.Resize(ref snapshots, count);

            trafficSnapshot = snapshots;
        }

        /// <summary>
        /// Record a collision event with pathing context for later review.
        /// </summary>
        public void OnCollision(IS_CON collision)
        {
            if (collision == null) return;

            var closingSpeed = (collision.SpClose & 0x0FFF) / 10.0;
            var timestampSeconds = collision.Time.TotalSeconds;

            var aDetails = BuildCollisionParticipantLog(collision.A);
            var bDetails = BuildCollisionParticipantLog(collision.B);

            logger.Log(
                $"COLLISION t={timestampSeconds:F2}s close={closingSpeed:F1}m/s {aDetails} | {bDetails}");
        }

        /// <summary>
        /// Build a detailed collision log entry for a car contact.
        /// </summary>
        private string BuildCollisionParticipantLog(CarContact contact)
        {
            if (contact == null) return "PLID=0";

            var plid = contact.PLID;
            var name = playerNames.TryGetValue(plid, out var storedName) ? storedName : $"PLID {plid}";
            var isAi = IsAiPlid(plid);

            var posX = contact.X / 16.0;
            var posY = contact.Y / 16.0;
            var speedMps = contact.Speed;
            var speedKmh = speedMps * 3.6;
            var directionDeg = contact.Direction * 360.0 / 256.0;
            var headingDeg = contact.Heading * 360.0 / 256.0;

            var throttle = (contact.ThrBrk >> 4) & 0x0F;
            var brake = contact.ThrBrk & 0x0F;
            var clutch = (contact.CluHan >> 4) & 0x0F;
            var handbrake = contact.CluHan & 0x0F;
            var gear = (contact.GearSp >> 4) & 0x0F;

            var routeName = isAi ? ResolveAiRouteName(plid) : "human";

            var projectionLabel = "path=unknown";
            if (TryGetCollisionPathProjection(plid, posX, posY, isAi, out var pathName, out var projection,
                    out var pathSource))
            {
                projectionLabel =
                    $"path={pathName} src={pathSource} s={projection.DistanceAlongPathMeters:F1}m lat={projection.LateralOffsetMeters:F1}m off={projection.DistanceToPathMeters:F1}m";
            }

            var aiDetails = string.Empty;
            if (isAi)
            {
                var (targetIndex, waypointCount, targetSpeed, inRecovery) = waypointFollower.GetFollowerInfo(plid);
                var recoveryLabel = inRecovery ? " recovery" : string.Empty;
                aiDetails =
                    $" route={routeName} target={targetIndex}/{waypointCount} tSpeed={targetSpeed:F0}km/h{recoveryLabel}";
            }

            return
                $"PLID={plid} name=\"{name}\" {(isAi ? "AI" : "H")} pos=({posX:F1},{posY:F1}) spd={speedKmh:F1}km/h dir={directionDeg:F0}° head={headingDeg:F0}° thr={throttle} brk={brake} clu={clutch} hbk={handbrake} gear={gear} info={contact.Info}{aiDetails} {projectionLabel}";
        }

        /// <summary>
        /// Determine whether a PLID belongs to an AI car.
        /// </summary>
        private bool IsAiPlid(byte plid)
        {
            lock (aiPlidLock)
            {
                return aiPLIDs.Contains(plid);
            }
        }

        /// <summary>
        /// Resolve the most likely route name for an AI based on its current state.
        /// </summary>
        private string ResolveAiRouteName(byte plid)
        {
            if (!currentRoute.TryGetValue(plid, out var routeName))
                return "unknown";

            switch (routeName)
            {
                case "spawn":
                    return config.SpawnRouteName;
                case "main":
                    return config.MainRouteName;
                case "branch":
                    return activeBranchSelections.TryGetValue(plid, out var branch) && branch != null
                        ? branch.Name
                        : "branch";
                default:
                    return routeName;
            }
        }

        /// <summary>
        /// Project the collision position onto the most relevant path for logging.
        /// </summary>
        private bool TryGetCollisionPathProjection(
            byte plid,
            double posX,
            double posY,
            bool isAi,
            out string pathName,
            out PathProjectionResult projection,
            out string source)
        {
            pathName = string.Empty;
            source = string.Empty;
            projection = default;

            if (isAi)
            {
                if (activeLaneChanges.TryGetValue(plid, out var activeChange))
                {
                    pathName = activeChange.ToAlternate
                        ? activeChange.TargetRoute?.Name ?? config.MainAlternateRouteName
                        : config.MainRouteName;
                    source = "transition";

                    var transitionPath = waypointFollower.GetPath(plid);
                    if (TryProjectOnPath(transitionPath, posX, posY, out projection))
                        return true;
                }
                else
                {
                    pathName = ResolveAiRouteName(plid);
                    source = "ai_path";

                    var aiPath = waypointFollower.GetPath(plid);
                    if (TryProjectOnPath(aiPath, posX, posY, out projection))
                        return true;
                }
            }

            if (TryProjectToBestRoutePath(posX, posY, out var bestPath, out projection))
            {
                pathName = bestPath;
                source = "nearest_route";
                return true;
            }

            return false;
        }

        /// <summary>
        /// Project a position onto a specific path if possible.
        /// </summary>
        private bool TryProjectOnPath(List<Util.Waypoint> path, double posX, double posY,
            out PathProjectionResult projection)
        {
            projection = default;
            if (path == null || path.Count < 2) return false;

            var geometry = PathProjection.GetGeometry(path, config.PathLoopClosureDistanceMeters);
            if (geometry == null) return false;

            return PathProjection.TryProjectToPath(path, geometry, posX, posY, out projection);
        }

        /// <summary>
        /// Find the closest known route path to a collision position.
        /// </summary>
        private bool TryProjectToBestRoutePath(double posX, double posY, out string pathName,
            out PathProjectionResult projection)
        {
            pathName = string.Empty;
            projection = default;

            var bestDistance = double.MaxValue;
            var bestProjection = default(PathProjectionResult);
            var bestName = string.Empty;

            foreach (var candidate in GetCollisionPathCandidates())
            {
                if (candidate.path == null || candidate.path.Count < 2) continue;

                var geometry = PathProjection.GetGeometry(candidate.path, config.PathLoopClosureDistanceMeters);
                if (geometry == null) continue;

                if (!PathProjection.TryProjectToPath(candidate.path, geometry, posX, posY, out var candidateProjection))
                    continue;

                if (candidateProjection.DistanceToPathMeters < bestDistance)
                {
                    bestDistance = candidateProjection.DistanceToPathMeters;
                    bestProjection = candidateProjection;
                    bestName = candidate.name;
                }
            }

            if (bestDistance == double.MaxValue) return false;

            pathName = bestName;
            projection = bestProjection;
            return true;
        }

        /// <summary>
        /// Enumerate the known route paths for collision logging.
        /// </summary>
        private IEnumerable<(string name, List<Util.Waypoint> path)> GetCollisionPathCandidates()
        {
            if (pathManager.SpawnRoute != null && pathManager.SpawnRoute.Count > 0)
                yield return (config.SpawnRouteName, pathManager.SpawnRoute);

            if (pathManager.MainRoute != null && pathManager.MainRoute.Count > 0)
                yield return (config.MainRouteName, pathManager.MainRoute);

            if (pathManager.MainAlternateRoute != null &&
                pathManager.MainAlternateRoute.Path != null &&
                pathManager.MainAlternateRoute.Path.Count > 0)
                yield return (pathManager.MainAlternateRoute.Name, pathManager.MainAlternateRoute.Path);

            foreach (var branch in pathManager.GetBranches())
            {
                if (branch.Path == null || branch.Path.Count == 0) continue;
                yield return (branch.Name, branch.Path);
            }
        }

        /// <summary>
        /// Trigger light/horn reactions when a fast human driver passes close to an AI.
        /// </summary>
        private void EvaluatePassByReactions()
        {
            if (!config.PassByReactionEnabled || allCars == null || allCars.Length == 0) return;

            var aiSnapshot = GetActiveAiPlids();
            if (aiSnapshot.Count == 0) return;

            var now = DateTime.UtcNow;
            var aiSet = new HashSet<byte>(aiSnapshot);

            foreach (var aiPlid in aiSnapshot)
            {
                if (passByCooldowns.TryGetValue(aiPlid, out var cooldownUntil) && cooldownUntil > now)
                    continue;

                if (passByProximityStates.TryGetValue(aiPlid, out var staleState) &&
                    (now - staleState.LastSeen).TotalSeconds >= config.PassByProximityResetSeconds)
                {
                    passByProximityStates.Remove(aiPlid);
                }

                var aiCar = Array.Find(allCars, c => c.PLID == aiPlid);
                if (aiCar == null || aiCar.PLID == 0) continue;

                var aiX = aiCar.X / 65536.0;
                var aiY = aiCar.Y / 65536.0;

                var foundTarget = false;
                double targetDistance = double.MaxValue;
                byte targetPlid = 0;

                foreach (var car in allCars)
                {
                    if (car.PLID == 0 || aiSet.Contains(car.PLID)) continue;

                    var humanX = car.X / 65536.0;
                    var humanY = car.Y / 65536.0;
                    var dx = humanX - aiX;
                    var dy = humanY - aiY;
                    var distance = Math.Sqrt(dx * dx + dy * dy);
                    if (distance > config.PassByReactionDistanceMeters) continue;

                    var speedKmh = 360.0 * car.Speed / 32768.0;
                    if (speedKmh < config.PassBySpeedThresholdKmh) continue;

                    if (distance < targetDistance)
                    {
                        targetDistance = distance;
                        targetPlid = car.PLID;
                        foundTarget = true;
                    }
                }

                if (!foundTarget) continue;
                var isNewTarget = !passByProximityStates.TryGetValue(aiPlid, out var proximityState) ||
                                  proximityState.TargetPlid != targetPlid ||
                                  (now - proximityState.LastSeen).TotalSeconds >= config.PassByProximityResetSeconds;

                passByProximityStates[aiPlid] = new PassByProximityState
                {
                    TargetPlid = targetPlid,
                    LastSeen = now
                };

                if (!isNewTarget) continue;
                if (passByRandom.NextDouble() > config.PassByReactionChance) continue;

                TriggerPassByReaction(aiPlid);
                passByCooldowns[aiPlid] = now.AddSeconds(GetPassByCooldownSeconds());
            }
        }

        /// <summary>
        /// Send the configured flash and/or horn reaction for a passing player.
        /// </summary>
        private void TriggerPassByReaction(byte plid)
        {
            var durationHundredths = GetPassByDurationHundredths();

            switch (config.PassByMode)
            {
                case AIConfig.PassByReactionMode.Flash:
                    lightController.FlashHighBeams(plid, durationHundredths);
                    break;
                case AIConfig.PassByReactionMode.Horn:
                    lightController.Honk(plid, 1, durationHundredths);
                    break;
                default:
                    lightController.FlashHighBeams(plid, durationHundredths);
                    lightController.Honk(plid, 1, durationHundredths);
                    break;
            }
        }

        /// <summary>
        /// Convert configured reaction duration to hundredths of a second for InSim inputs.
        /// </summary>
        private byte GetPassByDurationHundredths()
        {
            var durationHundredths = (int)Math.Round(config.PassByReactionDurationSeconds * 100.0);
            return (byte)Math.Max(1, Math.Min(255, durationHundredths));
        }

        /// <summary>
        ///     Get a randomized cooldown between configured limits for pass-by reactions.
        /// </summary>
        private double GetPassByCooldownSeconds()
        {
            var min = Math.Max(0.0, config.PassByCooldownMinSeconds);
            var max = Math.Max(min, config.PassByCooldownMaxSeconds);
            if (Math.Abs(max - min) < 0.01) return min;

            var window = max - min;
            return min + passByRandom.NextDouble() * window;
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
        /// Assign an AI to a target recorded route for population balancing.
        /// </summary>
        public void SetAssignedRoute(byte plid, string routeName)
        {
            if (plid == 0) return;

            var normalized = routeLibrary.NormalizeRouteName(
                string.IsNullOrWhiteSpace(routeName) ? config.MainRouteName : routeName);

            string previous;
            bool assignmentChanged;
            lock (assignmentLock)
            {
                previous = aiAssignedRoutes.TryGetValue(plid, out var stored) ? stored : string.Empty;
                aiAssignedRoutes[plid] = normalized;
                assignmentChanged = !string.Equals(previous, normalized, StringComparison.OrdinalIgnoreCase);
            }

            if (assignmentChanged)
            {
                var previousLabel = string.IsNullOrWhiteSpace(previous) ? "none" : previous;
                LogRouteAssignmentDecision(plid, $"assignment updated {previousLabel} -> {normalized}");
            }

            if (normalized.Equals(config.MainRouteName, StringComparison.OrdinalIgnoreCase))
            {
                if (assignmentChanged)
                    TryReturnToMainRoute(plid);
                return;
            }

            TryAssignBranchIfNeeded(plid, normalized);
        }

        /// <summary>
        /// Attempt to switch an AI onto a target branch when it has been assigned one.
        /// </summary>
        private void TryAssignBranchIfNeeded(byte plid, string routeName)
        {
            if (string.IsNullOrWhiteSpace(routeName)) return;
            if (!pathManager.TryGetBranchByName(routeName, out var branchInfo)) return;
            if (branchInfo == null) return;
            if (currentRoute.TryGetValue(plid, out var routeLabel) && routeLabel == "spawn") return;
            if (branchInfo.Path == null || branchInfo.Path.Count == 0) return;

            var car = Array.Find(allCars, c => c.PLID == plid);
            if (car == null || car.PLID == 0) return;

            if (activeBranchSelections.TryGetValue(plid, out var activeBranch) &&
                activeBranch?.Name != null &&
                activeBranch.Name.Equals(routeName, StringComparison.OrdinalIgnoreCase))
                return;

            if (TrySwitchToBranch(plid, routeName))
                return;

            if (pathManager.MainRoute == null || pathManager.MainRoute.Count == 0) return;

            var (targetIndex, _, _, _) = waypointFollower.GetFollowerInfo(plid);
            if (targetIndex >= 0 && targetIndex <= pathManager.MainRoute.Count &&
                branchInfo.StartIndex == targetIndex)
            {
                var (bestIndex, _, _) = FindAlignedWaypointIndex(branchInfo.Path, car, 0, 8);
                waypointFollower.SetPath(plid, branchInfo.Path, bestIndex);
                currentRoute[plid] = "branch";
                activeBranchSelections[plid] = branchInfo;
            }
        }

        /// <summary>
        /// Bring an AI back to the main route when its assignment no longer requires a branch.
        /// </summary>
        private void TryReturnToMainRoute(byte plid)
        {
            if (pathManager.MainRoute == null || pathManager.MainRoute.Count == 0) return;
            if (!currentRoute.TryGetValue(plid, out var routeLabel) || routeLabel != "branch") return;

            var car = Array.Find(allCars, c => c.PLID == plid);
            if (car == null || car.PLID == 0) return;

            var bestIndex = FindClosestIndex(pathManager.MainRoute, car);
            waypointFollower.SetPath(plid, pathManager.MainRoute, bestIndex);
            activeBranchSelections.Remove(plid);
            laneChangeLastMerge[plid] = DateTime.Now;
            laneChangeLastCheck[plid] = DateTime.Now;
            currentRoute[plid] = "main";
        }

        /// <summary>
        /// Enforce assigned routes when new telemetry arrives.
        /// </summary>
        private void ApplyPendingRouteAssignments()
        {
            foreach (var plid in GetActiveAiPlids())
            {
                if (!TryGetAssignedRoute(plid, out var targetRoute) ||
                    string.IsNullOrWhiteSpace(targetRoute))
                    continue;

                if (targetRoute.Equals(config.MainRouteName, StringComparison.OrdinalIgnoreCase))
                {
                    if (currentRoute.TryGetValue(plid, out var routeLabel) &&
                        routeLabel == "branch" &&
                        activeBranchSelections.TryGetValue(plid, out var branchInfo) &&
                        IsAlternateMainBranch(branchInfo))
                    {
                        LogRouteAssignmentDecision(
                            plid,
                            $"main assignment deferred for alternate lane {branchInfo?.Name ?? "branch"}");
                        continue;
                    }

                    if (currentRoute.TryGetValue(plid, out routeLabel) && routeLabel == "branch")
                        LogRouteAssignmentDecision(plid, "main assignment forcing return to main");

                    TryReturnToMainRoute(plid);
                }
                else
                {
                    TryAssignBranchIfNeeded(plid, targetRoute);
                }
            }
        }

        /// <summary>
        /// Remove cached telemetry for a specific PLID so stale car data is not reused.
        /// </summary>
        private void RemoveCarState(byte plid)
        {
            lock (carStateLock)
            {
                if (carStates.Remove(plid))
                    allCars = carStates.Values.ToArray();
            }
        }

        /// <summary>
        ///     Get the paths for all active AI cars
        /// </summary>
        /// <returns>Dictionary of paths by PLID</returns>
        public Dictionary<byte, List<Util.Waypoint>> GetAIPaths()
        {
            var result = new Dictionary<byte, List<Util.Waypoint>>();

            foreach (var plid in GetActiveAiPlids())
            {
                var path = waypointFollower.GetPath(plid);
                if (path != null && path.Count > 0) result[plid] = path;
            }

            return result;
        }

        /// <summary>
        /// Get a snapshot of AI route assignments keyed by PLID.
        /// </summary>
        public Dictionary<byte, string> GetAssignedRoutesSnapshot()
        {
            lock (assignmentLock)
            {
                return new Dictionary<byte, string>(aiAssignedRoutes);
            }
        }

        /// <summary>
        /// Try to resolve the assigned route for a specific AI.
        /// </summary>
        private bool TryGetAssignedRoute(byte plid, out string routeName)
        {
            lock (assignmentLock)
            {
                if (aiAssignedRoutes.TryGetValue(plid, out var stored) && !string.IsNullOrWhiteSpace(stored))
                {
                    routeName = stored;
                    return true;
                }

                routeName = string.Empty;
                return false;
            }
        }

        /// <summary>
        /// Get the list of active AI PLIDs in the order they spawned.
        /// </summary>
        public List<byte> GetActiveAiPlids()
        {
            lock (aiPlidLock)
            {
                return aiPLIDs.Where(p => p > 0).ToList();
            }
        }

        /// <summary>
        ///     Return the number of active AI PLIDs currently tracked.
        /// </summary>
        private int GetActiveAiCount()
        {
            lock (aiPlidLock)
            {
                return aiPLIDs.Count(p => p > 0);
            }
        }

        /// <summary>
        ///     Handle AII packet (AI info)
        /// </summary>
        public void OnAII(IS_AII aii)
        {
            var stopwatch = Stopwatch.StartNew();
            try
            {
                var plid = aii.PLID;
                bool knownAi;
                lock (aiPlidLock)
                {
                    knownAi = aiPLIDs.Contains(plid);
                }
                if (!knownAi) return;
                if (allCars == null) return;

                // Check engine state - stalled if ignition is on but RPM is near zero
                var ignitionOn = (aii.Flags & AIFlags.AIFLAGS_IGNITION) != 0;
                var engineStalled = ignitionOn && aii.RPM < 0.1f;
                var engineRunning = ignitionOn && !engineStalled;

                // Update engine state in driver
                driver.UpdateEngineState(plid, engineRunning);

                // Find car data in MCI
                var car = Array.Find(allCars, c => c.PLID == plid);
                if (car == null || car.PLID == 0)
                {
                    logger.Log($"PLID={plid} No MCI data available yet, skipping control update");
                    return;
                }

                var (targetIndex, _, _, _) = waypointFollower.GetFollowerInfo(plid);
                var routeName = currentRoute.TryGetValue(plid, out var currentName) ? currentName : string.Empty;
                var holdForMerge = !string.IsNullOrWhiteSpace(routeName) &&
                                   routeName.Equals("spawn", StringComparison.OrdinalIgnoreCase) &&
                                   ShouldHoldForSpawnMerge(plid, car, targetIndex, trafficSnapshot);

                // Update AI controls based on current state
                driver.UpdateControls(plid, car, allCars, trafficSnapshot, waypointManager, config,
                    lfsLayout.layoutObjects, aii.RPM, holdForMerge);

                // Route management
                var (updatedTargetIndex, _, _, _) = waypointFollower.GetFollowerInfo(plid);
                var laneChangeActive = HandleLaneChangeState(plid);
                if (!string.IsNullOrWhiteSpace(routeName))
                {
                    if (laneChangeActive)
                        return;

                    if (routeName == "spawn" && updatedTargetIndex >= pathManager.SpawnRoute.Count - 1)
                    {
                        if (holdForMerge)
                            return;

                        var bestIndex = FindClosestIndex(pathManager.MainRoute, car);
                        waypointFollower.SetPath(plid, pathManager.MainRoute, bestIndex);
                        currentRoute[plid] = "main";
                    }
                    else if (routeName == "main")
                    {
                        TryGetAssignedRoute(plid, out var assignedRoute);
                        var hasAssignment = !string.IsNullOrWhiteSpace(assignedRoute);
                        var assignedIsMain = !hasAssignment ||
                                             assignedRoute.Equals(config.MainRouteName,
                                                 StringComparison.OrdinalIgnoreCase);

                        var allowInnerBranch = pathManager.MainAlternateRoute != null &&
                                               (!hasAssignment ||
                                                assignedRoute.Equals(config.MainRouteName,
                                                    StringComparison.OrdinalIgnoreCase) ||
                                                assignedRoute.Equals(pathManager.MainAlternateRoute.Name,
                                                    StringComparison.OrdinalIgnoreCase));
                        var switchedInner = allowInnerBranch && TryJoinInnerBranchLaneChange(plid, car, targetIndex);

                        if (pathManager.TryGetBranch(targetIndex, out var branchInfo) && branchInfo != null)
                        {
                            var innerBlocked = IsInnerBranch(branchInfo) &&
                                               (!hasAssignment ||
                                                !assignedRoute.Equals(branchInfo.Name,
                                                    StringComparison.OrdinalIgnoreCase));
                            var assignmentMismatch = hasAssignment &&
                                                     !assignedRoute.Equals(branchInfo.Name,
                                                         StringComparison.OrdinalIgnoreCase);
                            var randomSkip = !hasAssignment && branchRandom.NextDouble() >= 0.5;

                            var shouldSwitch = !switchedInner && !innerBlocked && !assignmentMismatch && !randomSkip;

                            if (shouldSwitch)
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
                        else if (!assignedIsMain)
                        {
                            TryAssignBranchIfNeeded(plid, assignedRoute);
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

                        var hasBranchInfo = activeBranchSelections.TryGetValue(plid, out var branchInfo);
                        var isInnerBranch = hasBranchInfo && IsInnerBranch(branchInfo);

                        if (isInnerBranch)
                        {
                            if (TrySwapInnerToMainLane(plid, car, targetIndex, path))
                                return;

                            // Stay on alternate/looping inner lanes until an explicit merge is requested.
                            if (branchInfo?.Metadata?.Type == RouteType.AlternateMain ||
                                branchInfo?.Metadata?.IsLoop == true)
                                return;
                        }

                        if (targetIndex >= path.Count - 1)
                        {
                            var bestIndex = FindClosestIndex(pathManager.MainRoute, car);
                            var approachIndex = Math.Max(0, path.Count - 2);

                            if (hasBranchInfo && branchInfo != null)
                            {
                                var hintedIndex = pathManager.MainRoute.Count == 0
                                    ? 0
                                    : Math.Max(
                                        0,
                                        Math.Min(pathManager.MainRoute.Count - 1, branchInfo.RejoinIndex));
                                bestIndex = hintedIndex;

                                if (TryScheduleBranchEndRejoin(plid, car, approachIndex, path, branchInfo, bestIndex))
                                    return;
                            }
                            else
                            {
                                if (TryScheduleBranchEndRejoin(plid, car, approachIndex, path, branchInfo, bestIndex))
                                    return;
                            }

                            if (laneChangeDetailedLogging)
                                logger.Log(
                                    $"PLID={plid} Holding rejoin from {branchInfo?.Name ?? "branch"} until lane change safety conditions pass");
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "OnAII failed");
            }
            finally
            {
                stopwatch.Stop();
                aiiPerformanceTracker.Record(stopwatch.Elapsed);
                LogPerformanceSnapshots();
            }
        }

        /// <summary>
        ///     Emit aggregated timing stats for AII and MCI handlers on a rolling window.
        /// </summary>
        private void LogPerformanceSnapshots()
        {
            var now = DateTime.UtcNow;
            if (now - lastPerformanceLog < PerformanceLogWindow) return;

            var hasAii = aiiPerformanceTracker.TryGetSnapshot(PerformanceLogWindow, out var aiiSnapshot);
            var hasMci = mciPerformanceTracker.TryGetSnapshot(PerformanceLogWindow, out var mciSnapshot);
            if (!hasAii && !hasMci) return;

            UpdatePerformanceStatusUi(
                hasAii ? (PerformanceSnapshot?)aiiSnapshot : null,
                hasMci ? (PerformanceSnapshot?)mciSnapshot : null);

            if (performanceLogging)
            {
                var parts = new List<string>();
                if (hasAii)
                {
                    parts.Add(
                        $"AII count={aiiSnapshot.Count} avg={aiiSnapshot.AverageMs:F2}ms max={aiiSnapshot.MaxMs:F2}ms total={aiiSnapshot.TotalMs:F1}ms rate={aiiSnapshot.RatePerSecond:F1}/s");
                }

                if (hasMci)
                {
                    parts.Add(
                        $"MCI count={mciSnapshot.Count} avg={mciSnapshot.AverageMs:F2}ms max={mciSnapshot.MaxMs:F2}ms total={mciSnapshot.TotalMs:F1}ms rate={mciSnapshot.RatePerSecond:F1}/s");
                }

                if (parts.Count > 0)
                {
                    var activeAiCount = GetActiveAiCount();
                    logger.Log(
                        $"PERF ({PerformanceLogWindow.TotalSeconds}s): activeAI={activeAiCount} | {string.Join(" | ", parts)}");
                }
            }

            lastPerformanceLog = now;
        }

        /// <summary>
        /// Update the main UI with a traffic-safe performance health indicator.
        /// </summary>
        private void UpdatePerformanceStatusUi(PerformanceSnapshot? aiiSnapshot, PerformanceSnapshot? mciSnapshot)
        {
            var load = GetWorstPerformanceLoad(aiiSnapshot, mciSnapshot);
            var health = EvaluatePerformanceHealth(aiiSnapshot.HasValue || mciSnapshot.HasValue, load);
            if (health == lastPerformanceHealth && Math.Abs(load - lastPerformanceLoad) < 0.01) return;

            lastPerformanceHealth = health;
            lastPerformanceLoad = load;
            mainUI.UpdatePerformanceStatus(BuildPerformanceStatusLabel(health, load));
        }

        /// <summary>
        /// Determine the worst handler load ratio across recent snapshots.
        /// </summary>
        private static double GetWorstPerformanceLoad(
            PerformanceSnapshot? aiiSnapshot,
            PerformanceSnapshot? mciSnapshot)
        {
            var worst = 0.0;
            if (aiiSnapshot.HasValue)
                worst = Math.Max(worst, GetSnapshotLoad(aiiSnapshot.Value));
            if (mciSnapshot.HasValue)
                worst = Math.Max(worst, GetSnapshotLoad(mciSnapshot.Value));
            return Math.Max(0.0, worst);
        }

        /// <summary>
        /// Convert a snapshot to a normalized load ratio for the window.
        /// </summary>
        private static double GetSnapshotLoad(PerformanceSnapshot snapshot)
        {
            var elapsedMs = Math.Max(1.0, snapshot.Elapsed.TotalMilliseconds);
            return snapshot.TotalMs / elapsedMs;
        }

        /// <summary>
        /// Map load ratio to a health label for the UI.
        /// </summary>
        private static PerformanceHealth EvaluatePerformanceHealth(bool hasSamples, double load)
        {
            if (!hasSamples) return PerformanceHealth.Waiting;
            if (load <= PerformanceOnTimeMaxLoad) return PerformanceHealth.OnTime;
            if (load <= PerformanceSlowingMaxLoad) return PerformanceHealth.Slowing;
            return PerformanceHealth.RunningSlow;
        }

        /// <summary>
        /// Build the color-coded UI label for the performance indicator.
        /// </summary>
        private static string BuildPerformanceStatusLabel(PerformanceHealth health, double load)
        {
            var percent = Math.Max(0, Math.Min(999, (int)Math.Round(load * 100)));
            switch (health)
            {
                case PerformanceHealth.OnTime:
                    return $"^2AI Perf: On Time ({percent}%)";
                case PerformanceHealth.Slowing:
                    return $"^3AI Perf: Slowing ({percent}%)";
                case PerformanceHealth.RunningSlow:
                    return $"^1AI Perf: Running Slow ({percent}%)";
                default:
                    return "^7AI Perf: waiting";
            }
        }

        private void OnOutGauge(object? sender, OutGaugeEventArgs e)
        {
            if (e.PLID == playerPLID)
            {
                playerThrottle = e.Throttle;
                playerBrake = e.Brake;
            }
        }

        private IEnumerable<(byte id, string name)> GetAiTuples()
        {
            lock (aiPlidLock)
            {
                foreach (var plid in aiPLIDs)
                {
                    var name = aiNames.TryGetValue(plid, out var stored) ? stored : $"AI {plid}";
                    yield return (plid, name);
                }
            }
        }

        /// <summary>
        /// Reload all route files from disk and reapply paths to active AIs.
        /// </summary>
        public void ReloadRoutes(bool silent = false)
        {
            waypointManager.ClearRoutes();
            pathManager.LoadRoutes(config);

            foreach (var plid in GetActiveAiPlids())
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
                insim.SendPrivateMessage("Routes reloaded from disk");
            }

            RefreshRouteOptions(recordingRouteName);
            NotifyRouteValidationIssues();
            logger.Log("Routes reloaded and reapplied to active AIs");
            populationManager?.RequestReconcile("routes reloaded");
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
            insim.SendPrivateMessage($"{label}: {summary}");
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
        public void OnLayoutSelectionChanged(ObjectInfo? selectedObject)
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
                insim.SendPrivateMessage("No recorded route loaded for layout selection.");
                return;
            }

            var index = FindClosestRouteNodeIndex(route, selectedObject, out var distanceMeters);
            selectedRouteNodeIndex = index;

            var node = route.Nodes[index];
            var speed = node.SpeedLimit ?? node.Speed;
            var status = $"Node {index} @ {speed:F0} km/h";
            mainUI?.UpdateLayoutSelectionStatus(status);
            insim.SendPrivateMessage($"Selected node {index} ({distanceMeters:F1}m) on {route.Metadata.Name}.");
        }

        /// <summary>
        /// Persist moved layout object coordinates back into the selected route node.
        /// </summary>
        public void OnLayoutObjectMoved(ObjectInfo? movedObject)
        {
            if (movedObject == null) return;

            var route = GetEditableRoute();
            if (route == null) return;
            if (!TryGetSelectedNodeIndex(route, out var index)) return;

            route.Nodes[index].X = movedObject.X / 16.0;
            route.Nodes[index].Y = movedObject.Y / 16.0;
            route.Nodes[index].Z = movedObject.Zbyte / 4.0;
            route.Nodes[index].Heading = ConvertLayoutHeadingToRouteHeading(movedObject.Heading);
            routeLibrary.Save(route);

            var headingDeg = CoordinateUtils.HeadingToDegrees(route.Nodes[index].Heading);
            var status =
                $"Node {index} moved to ({route.Nodes[index].X:F1}, {route.Nodes[index].Y:F1}) @ {headingDeg:F0}°";
            mainUI?.UpdateLayoutSelectionStatus(status);
            insim.SendPrivateMessage($"{route.Metadata.Name}: saved moved node {index}.");
        }

        /// <summary>
        /// Convert a layout object heading byte into the LFS heading stored on route nodes.
        /// </summary>
        private static int ConvertLayoutHeadingToRouteHeading(byte headingByte)
        {
            var normalizedByte = (headingByte - 128 + 256) % 256;
            return CoordinateUtils.NormalizeHeading(normalizedByte * 256);
        }

        /// <summary>
        /// Update the speed limit on the currently selected node in the layout editor.
        /// </summary>
        public void UpdateSelectedNodeSpeed(double speedKmh)
        {
            var route = GetEditableRoute();
            if (route == null) return;
            if (!TryGetSelectedNodeIndex(route, out var index)) return;

            var clamped = Math.Max(0, speedKmh);
            route.Nodes[index].SpeedLimit = clamped;
            routeLibrary.Save(route);

            mainUI?.UpdateLayoutSelectionStatus($"Node {index} @ {clamped:F0} km/h");
            insim.SendPrivateMessage($"Set node {index} speed to {clamped:F0} km/h.");
        }

        /// <summary>
        /// Delete the currently selected layout node and refresh the visualization.
        /// </summary>
        public void DeleteSelectedNode(byte viewPlid)
        {
            var route = GetEditableRoute();
            if (route == null || route.Nodes == null || route.Nodes.Count == 0)
            {
                insim.SendPrivateMessage("No recorded route loaded to delete from.");
                return;
            }

            if (!TryGetSelectedNodeIndex(route, out var index)) return;

            if (route.Nodes.Count <= 1)
            {
                insim.SendPrivateMessage("Cannot delete the last node of the route.");
                return;
            }

            route.Nodes.RemoveAt(index);
            var nextIndex = route.Nodes.Count == 0 ? -1 : Math.Min(index, route.Nodes.Count - 1);
            selectedRouteNodeIndex = nextIndex >= 0 ? nextIndex : (int?)null;
            routeLibrary.Save(route);

            if (selectedRouteNodeIndex.HasValue)
            {
                var node = route.Nodes[selectedRouteNodeIndex.Value];
                var speed = node.SpeedLimit ?? node.Speed;
                mainUI?.UpdateLayoutSelectionStatus($"Node {selectedRouteNodeIndex.Value} @ {speed:F0} km/h");
            }
            else
            {
                mainUI?.UpdateLayoutSelectionStatus("No node selected");
            }

            insim.SendPrivateMessage(
                $"Deleted node {index} from {route.Metadata.Name}. {route.Nodes.Count} nodes remain.");

            if (route.Nodes.Count == 0)
            {
                lfsLayout.ClearAllVisualizations();
                lfsLayout.WaypointsVisualized = false;
                mainUI?.UpdateLayoutToggleState(false);
                return;
            }

            if (lfsLayout.WaypointsVisualized) VisualizeSelectedRoute(viewPlid);
        }

        /// <summary>
        /// Start recording to extend the current route from the selected node, replacing later nodes.
        /// </summary>
        public void ExtendRecordingFromSelection(byte viewPlid)
        {
            var route = GetEditableRoute();
            if (route == null || route.Nodes == null || route.Nodes.Count == 0)
            {
                insim.SendPrivateMessage("No recorded route loaded to extend.");
                return;
            }

            if (!TryGetSelectedNodeIndex(route, out var index)) return;

            recordingRouteName = route.Metadata.Name;
            mainUI?.UpdateRecordingRouteSelection(recordingRouteName);
            visualizationRouteName = recordingRouteName;
            mainUI?.UpdateVisualizationRouteSelection(visualizationRouteName);

            routeRecorder.StartFromExisting(route, index);
            selectedRouteNodeIndex = Math.Min(index, route.Nodes.Count - 1);
            var preserved = route.Nodes?.Count ?? 0;
            mainUI?.UpdateLayoutSelectionStatus($"Extending from node {index} ({preserved} kept)");
            insim.SendPrivateMessage(
                $"Extending {route.Metadata.Name} from node {index}; later nodes will be re-recorded.");

            if (lfsLayout.WaypointsVisualized) VisualizeSelectedRoute(viewPlid);
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
            if (route == null) return;
            if (!TryGetSelectedNodeIndex(route, out var index)) return;

            updater(route, index);
            routeLibrary.Save(route);
            insim.SendPrivateMessage($"Set {label} node to {index} for {route.Metadata.Name}.");
        }

        /// <summary>
        /// Get the recorded route currently selected for visualization and editing.
        /// </summary>
        private RecordedRoute? GetEditableRoute()
        {
            var route = waypointManager.GetRecordedRoute(visualizationRouteName);
            if (route != null) return route;

            waypointManager.LoadTrafficRoute(visualizationRouteName);
            return waypointManager.GetRecordedRoute(visualizationRouteName);
        }

        /// <summary>
        /// Try to resolve the selected node index from the current layout selection state.
        /// </summary>
        private bool TryGetSelectedNodeIndex(RecordedRoute? route, out int index)
        {
            index = 0;
            if (route == null || route.Nodes == null || route.Nodes.Count == 0)
            {
                insim.SendPrivateMessage("No recorded route data available.");
                return false;
            }

            if (!selectedRouteNodeIndex.HasValue)
            {
                insim.SendPrivateMessage("Select a node in the layout editor first.");
                return false;
            }

            index = selectedRouteNodeIndex.Value;
            if (index < 0 || index >= route.Nodes.Count)
            {
                insim.SendPrivateMessage("Selected node index is out of range.");
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
        /// Reset the tracked player car to the selected layout node position.
        /// </summary>
        public void SpawnPlayerAtSelection()
        {
            var route = GetEditableRoute();
            if (route == null || route.Nodes == null || route.Nodes.Count == 0)
            {
                insim.SendPrivateMessage("No recorded route loaded to spawn from.");
                return;
            }

            if (!TryGetSelectedNodeIndex(route, out var index)) return;
            if (playerPLID == 0)
            {
                insim.SendPrivateMessage("No player tracked to spawn. Join the server to capture your PLID.");
                return;
            }

            var node = route.Nodes[index];
            var x = (int)Math.Round(node.X * 16.0);
            var y = (int)Math.Round(node.Y * 16.0);
            var xShort = (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, x));
            var yShort = (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, y));
            var zByte = (byte)Math.Max(0, Math.Min(255, Math.Round(node.Z * 4.0)));
            var normalizedHeading = CoordinateUtils.NormalizeHeading(node.Heading);
            var headingByte = (byte)((normalizedHeading / 256 + 128) % 256);

            var startPos = new ObjectInfo
            {
                X = xShort,
                Y = yShort,
                Zbyte = zByte,
                Heading = headingByte,
                Flags = 0x80,
                Index = 0
            };

            insim.Send(new IS_JRR
            {
                ReqI = 0,
                JRRAction = JrrAction.JRR_RESET_NO_REPAIR,
                PLID = playerPLID,
                UCID = 0,
                StartPos = startPos
            });

            insim.SendPrivateMessage(
                $"Spawned player at node {index} ({node.X:F1}, {node.Y:F1}, z={node.Z:F1}) heading {node.Heading}.");
        }

        /// <summary>
        /// Update the displayed InSim connection status and host.
        /// </summary>
        public void UpdateInSimStatus(string status, string host)
        {
            mainUI?.UpdateInSimStatus(status, host);
        }

        /// <summary>
        /// Sync the layout visualization toggle button with the current visualization state.
        /// </summary>
        public void SyncLayoutToggleState()
        {
            mainUI?.UpdateLayoutToggleState(lfsLayout.WaypointsVisualized);
        }

        /// <summary>
        /// Hide all debug UI buttons and leave a restore control in the top-left corner.
        /// </summary>
        public void HideUI()
        {
            mainUI?.HideUI();
        }

        /// <summary>
        /// Restore all debug UI buttons after they were hidden.
        /// </summary>
        public void ShowUI()
        {
            mainUI?.ShowUI();
            debugUI?.RestoreDebugButtons();
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
        private void VisualizeSelectedRouteInternal(byte viewPlid, bool clearExisting = true)
        {
            var effectivePlid = viewPlid == 0 ? GetDefaultVisualizationPlid() : viewPlid;
            if (effectivePlid == 0)
            {
                logger.LogWarning("Cannot visualize routes: no player in view");
                insim.SendPrivateMessage("Select a car/view to place layout objects.");
                return;
            }

            waypointManager.LoadTrafficRoute(visualizationRouteName);
            var recorded = waypointManager.GetRecordedRoute(visualizationRouteName);
            if (recorded == null)
            {
                logger.LogWarning($"No recorded route found for visualization: {visualizationRouteName}");
                insim.SendPrivateMessage($"No recorded route found for {visualizationRouteName}.");
                return;
            }

            var detailStep = GetVisualizationStep(recorded.Nodes?.Count ?? 0, out var clamped);
            lfsLayout.VisualizeRecordedRoute(effectivePlid, recorded, detailStep, clearExisting);
            lfsLayout.WaypointsVisualized = true;

            if (clamped)
            {
                insim.SendPrivateMessage(
                    $"Detail clamped to every {detailStep} node(s) to stay under {LFSLayout.MaxVisibleWaypoints} objects.");
            }
            else
            {
                insim.SendPrivateMessage($"Visualized {visualizationRouteName} (x{detailStep}).");
            }

            mainUI?.UpdateLayoutToggleState(true);
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
