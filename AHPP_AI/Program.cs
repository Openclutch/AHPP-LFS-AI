using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using AHPP_AI.AI;
using AHPP_AI.Debug;
using AHPP_AI.Waypoint;
using AHPP_AI.UI;
using AHPP_AI.Util;
using InSimDotNet;
using InSimDotNet.Packets;
using InSimClient = InSimDotNet.InSimClient;

namespace AHPP_AI
{
    /// <summary>
    ///     Main program class handling initialization and coordination of all components
    /// </summary>
    public class Program
    {
        // InSim connection
        private static readonly InSimClient insim = new InSimClient();

        // Core components
        private static readonly Logger logger = new Logger("log.txt");
        private static readonly AppConfig appConfig;
        private static readonly RouteLibrary routeLibrary;
        private static readonly WaypointManager waypointManager;
        private static readonly LFSLayout visualizer;
        private static readonly AIController aiController;

        // Debug settings
        private static readonly bool debugEnabled;
        private static readonly bool debugWaypoints;
        private static readonly bool debugCoordinateSystem;
        private static readonly bool autoSpawnAI;

        // Host settings
        private static readonly string onlineHost;
        private static readonly string localHost;
        private static string currentHost;
        private static readonly int port;
        private static readonly int outGaugePort;
        private static byte currentViewPLID = 0;
        private static string currentRecordingRoute = "main_loop";
        private static string pendingRouteName = "main_loop";
        private static readonly string spawnRouteName;
        private static readonly string mainRouteName;
        private static readonly string mainAlternateRouteName;
        private static readonly List<string> branchRouteNames = new List<string>();
        private static string currentTrackCode = "UnknownTrack";
        private static string currentLayoutName = "DefaultLayout";
        private static readonly int initialAiCount;
        private static readonly int spawnDelayMs;
        private static readonly int lookaheadWaypoints;
        private static readonly double recordingIntervalMeters;
        private static readonly double waypointProximityMultiplier;
        private static readonly double steeringResponseDamping;
        private static readonly double steeringDeadzoneDegrees;
        private static readonly bool purePursuitEnabled;
        private static readonly double purePursuitLookaheadMinMeters;
        private static readonly double purePursuitLookaheadMaxMeters;
        private static readonly double purePursuitLookaheadSpeedFactor;
        private static readonly double purePursuitWheelbaseMeters;
        private static readonly double purePursuitSteeringGain;
        private static readonly double purePursuitMaxSteerDegrees;
        private static readonly int throttleBase;
        private static readonly int brakeBase;
        private static readonly int minSteering;
        private static readonly int maxSteering;
        private static readonly int steeringCenter;
        private static readonly int shiftDelayMs;
        private static readonly int gearUpshiftHysteresisKmh;
        private static readonly int gearDownshiftHysteresisKmh;
        private static readonly int gearShiftMinIntervalMs;
        private static readonly double[] gearSpeedThresholdsKmh;
        private static readonly int clutchFullyPressed;
        private static readonly int clutchReleased;
        private static readonly int clutchPressDelayMs;
        private static readonly int clutchHoldAfterShiftMs;
        private static readonly int clutchReleaseSteps;
        private static readonly int clutchReleaseIntervalMs;
        private static readonly int stallPreventionRpm;
        private static readonly int stallPreventionReleaseRpm;
        private static readonly int stallPreventionHoldMs;
        private static readonly bool wallRecoveryEnabled;
        private static readonly int waypointTimeoutSeconds;
        private static readonly int progressCheckIntervalMs;
        private static readonly double minRequiredProgress;
        private static readonly int maxRecoveryAttempts;
        private static readonly int maxFailedRecoveryCycles;
        private static readonly double minSpeedThreshold;
        private static readonly int stationaryCheckCount;
        private static readonly double waypointMinThreshold;
        private static readonly double waypointMaxThreshold;
        private static readonly double waypointThresholdSpeedFactor;
        private static readonly double collisionDetectionRangeMeters;
        private static readonly double collisionDetectionAngleDegrees;
        private static readonly double minimumSafetyDistanceMeters;
        private static readonly double collisionDetectionHalfWidthMeters;
        private static readonly int recoveryShortReverseMs;
        private static readonly int recoveryLongReverseMs;
        private static readonly int recoveryCooldownMs;
        private static readonly int recoveryValidationWindowMs;
        private static readonly double recoverySuccessDistanceMeters;
        private static readonly double recoverySuccessSpeedKmh;
        private static readonly double recoveryPositionChangeThreshold;
        private static readonly double recoveryProgressStallThreshold;
        private static readonly int recoveryStuckCheckIntervalMs;
        private static readonly double recoveryLowSpeedThresholdKmh;
        private static readonly int recoveryDetectionsBeforeAction;
        private static readonly int recoveryMaxFailureCount;
        private static readonly int recoveryStallReverseTrigger;
        private static readonly int maxPlayers;
        private static readonly int reservedSlots;
        private static readonly double aiFillRatio;
        private static readonly int minAis;
        private static readonly int maxAis;
        private static readonly int adjustIntervalMs;
        private static readonly int spawnBatchSize;
        private static readonly int removeBatchSize;
        private static readonly bool autoManagePopulation;
        private static readonly bool insimDebugLogging;
        private static readonly bool activeWaypointMarkersEnabled;
        private static readonly int activeWaypointIntervalMs;
        private static readonly bool performanceLogging;
        private static readonly double laneChangeInitialCheckIntervalSeconds;
        private static readonly double laneChangePostCooldownIntervalSeconds;
        private static readonly double laneChangeMergeChance;
        private static readonly double laneChangeCooldownSeconds;
        private static readonly double laneChangeSafetyCheckDistanceMeters;
        private static readonly double laneChangeSafetyCheckHalfWidthMeters;
        private static readonly double laneChangeMaxParallelDistanceMeters;
        private static readonly double laneChangeMaxParallelHeadingDegrees;
        private static readonly double laneChangeParallelLookaheadSeconds;
        private static readonly double laneChangeParallelLookaheadMinMeters;
        private static readonly double laneChangeMaxMergeSpeedKmh;
        private static readonly double laneChangeSignalLeadTimeSeconds;
        private static readonly double laneChangeSignalMinimumDurationSeconds;
        private static readonly double laneChangeTransitionLengthMeters;
        private static readonly int laneChangeTransitionPointCount;
        private static readonly int laneChangeTargetAheadWaypoints;
        private static readonly double laneChangeRearCheckDistanceMeters;
        private static readonly double laneChangeRearCheckTtcSeconds;
        private static readonly double laneChangeMergeGapFactor;
        private static readonly double pathLoopClosureDistanceMeters;
        private static readonly double trafficLaneHalfWidthMeters;
        private static readonly double trafficBaseGapMeters;
        private static readonly double trafficTimeHeadwaySeconds;
        private static readonly double trafficAiSpacingFactor;
        private static readonly double trafficHumanSpacingFactor;
        private static readonly double trafficLookaheadSeconds;
        private static readonly double trafficLookaheadMinMeters;
        private static readonly double trafficBrakeTtcSeconds;
        private static readonly double trafficEmergencyTtcSeconds;
        private static readonly int spawnMergeHoldLookaheadWaypoints;
        private static readonly double spawnMergeHoldDistanceMeters;
        private static readonly bool passByReactionEnabled;
        private static readonly double passByReactionChance;
        private static readonly double passBySpeedThresholdKmh;
        private static readonly double passByReactionDurationSeconds;
        private static readonly double passByReactionDistanceMeters;
        private static readonly double passByCooldownMinSeconds;
        private static readonly double passByCooldownMaxSeconds;
        private static readonly double passByProximityResetSeconds;
        private static readonly AIConfig.PassByReactionMode passByReactionMode;
        private static readonly string buildVersion;

        /// <summary>
        ///     Static constructor for initialization of components
        /// </summary>
        static Program()
        {
            var basePath = AppDomain.CurrentDomain.BaseDirectory;
            appConfig = AppConfig.Load(Path.Combine(basePath, "config.ini"), logger);
            var logLevelText = appConfig.GetString("Logging", "Level", "Warn");
            logger.SetMinimumLevel(Logger.ParseLogLevel(logLevelText, Logger.LogLevel.Warn));

            onlineHost = appConfig.GetString("InSim", "HostOnline",
                appConfig.GetString("InSim", "Host", "10.211.55.4"));
            localHost = appConfig.GetString("InSim", "HostLocal", "127.0.0.1");
            currentHost = onlineHost;
            port = appConfig.GetInt("InSim", "Port", 29999);
            outGaugePort = appConfig.GetInt("InSim", "OutGaugePort", 30000);

            debugEnabled = appConfig.GetBool("DebugAI", "Enabled", true);
            autoSpawnAI = appConfig.GetBool("DebugAI", "AutoSpawnAI", true);
            debugWaypoints = appConfig.GetBool("DebugAI", "AutoVisualizeWaypoints", true);
            debugCoordinateSystem = appConfig.GetBool("DebugAI", "AutoVisualizeAxes", false);

            routeLibrary = new RouteLibrary(logger);
            mainRouteName = routeLibrary.NormalizeRouteName(appConfig.GetString("Routes", "Main", "main_loop"));
            spawnRouteName = routeLibrary.NormalizeRouteName(appConfig.GetString("Routes", "Spawn", "pit_entry"));
            var rawAlt = appConfig.GetString("Routes", "MainAlt", string.Empty);
            mainAlternateRouteName = string.IsNullOrWhiteSpace(rawAlt)
                ? string.Empty
                : routeLibrary.NormalizeRouteName(rawAlt);
            for (var i = 1; i <= 3; i++)
            {
                var detour = appConfig.GetString("Routes", $"Detour{i}", string.Empty);
                if (string.IsNullOrWhiteSpace(detour)) continue;
                var normalized = routeLibrary.NormalizeRouteName(detour);
                if (!branchRouteNames.Exists(n => n.Equals(normalized, StringComparison.OrdinalIgnoreCase)))
                    branchRouteNames.Add(normalized);
            }
            currentRecordingRoute = routeLibrary.NormalizeRouteName(mainRouteName);
            pendingRouteName = currentRecordingRoute;

            initialAiCount = appConfig.GetInt("AI", "NumberOfAIs", 0);
            spawnDelayMs = appConfig.GetInt("AI", "SpawnDelayMs", 10000);
            lookaheadWaypoints = appConfig.GetInt("AI", "LookaheadWaypoints", 2);
            waypointProximityMultiplier = appConfig.GetDouble("AI", "WaypointProximityMultiplier", 1.0);
            steeringResponseDamping = appConfig.GetDouble("AI", "SteeringDamping", 1.0);
            steeringDeadzoneDegrees = appConfig.GetDouble("AI", "SteeringDeadzoneDegrees", 0.0);
            purePursuitEnabled = appConfig.GetBool("AI", "PurePursuitEnabled", true);
            purePursuitLookaheadMinMeters = appConfig.GetDouble("AI", "PurePursuitLookaheadMinMeters", 6.0);
            purePursuitLookaheadMaxMeters = appConfig.GetDouble("AI", "PurePursuitLookaheadMaxMeters", 25.0);
            purePursuitLookaheadSpeedFactor = appConfig.GetDouble("AI", "PurePursuitLookaheadSpeedFactor", 0.35);
            purePursuitWheelbaseMeters = appConfig.GetDouble("AI", "PurePursuitWheelbaseMeters", 2.5);
            purePursuitSteeringGain = appConfig.GetDouble("AI", "PurePursuitSteeringGain", 1.0);
            purePursuitMaxSteerDegrees = appConfig.GetDouble("AI", "PurePursuitMaxSteerDegrees", 25.0);
            throttleBase = appConfig.GetInt("AI", "ThrottleBase", 0);
            brakeBase = appConfig.GetInt("AI", "BrakeBase", 10000);
            minSteering = appConfig.GetInt("AI", "MinSteering", 1);
            maxSteering = appConfig.GetInt("AI", "MaxSteering", 65535);
            steeringCenter = appConfig.GetInt("AI", "SteeringCenter", 32768);
            shiftDelayMs = appConfig.GetInt("AI", "ShiftDelayMs", 500);
            gearUpshiftHysteresisKmh = appConfig.GetInt("AI", "GearUpshiftHysteresisKmh", 5);
            gearDownshiftHysteresisKmh = appConfig.GetInt("AI", "GearDownshiftHysteresisKmh", 5);
            gearShiftMinIntervalMs = appConfig.GetInt("AI", "GearShiftMinIntervalMs", 900);
            gearSpeedThresholdsKmh = ParseGearSpeedThresholds(
                appConfig.GetString("AI", "GearSpeedThresholdsKmh", string.Empty));
            clutchFullyPressed = appConfig.GetInt("AI", "ClutchFullyPressed", 65535);
            clutchReleased = appConfig.GetInt("AI", "ClutchReleased", 0);
            clutchPressDelayMs = appConfig.GetInt("AI", "ClutchPressDelayMs", 100);
            clutchHoldAfterShiftMs = appConfig.GetInt("AI", "ClutchHoldAfterShiftMs", 75);
            clutchReleaseSteps = appConfig.GetInt("AI", "ClutchReleaseSteps", 5);
            clutchReleaseIntervalMs = appConfig.GetInt("AI", "ClutchReleaseIntervalMs", 100);
            stallPreventionRpm = appConfig.GetInt("AI", "StallPreventionRpm", 500);
            stallPreventionReleaseRpm = appConfig.GetInt("AI", "StallPreventionReleaseRpm", 900);
            stallPreventionHoldMs = appConfig.GetInt("AI", "StallPreventionHoldMs", 750);
            wallRecoveryEnabled = appConfig.GetBool("AI", "WallRecoveryEnabled", true);
            waypointTimeoutSeconds = appConfig.GetInt("AI", "WaypointTimeoutSeconds", 30);
            progressCheckIntervalMs = appConfig.GetInt("AI", "ProgressCheckIntervalMs", 5000);
            minRequiredProgress = appConfig.GetDouble("AI", "MinRequiredProgress", 5.0);
            maxRecoveryAttempts = appConfig.GetInt("AI", "MaxRecoveryAttempts", 5);
            maxFailedRecoveryCycles = appConfig.GetInt("AI", "MaxFailedRecoveryCycles", 2);
            minSpeedThreshold = appConfig.GetDouble("AI", "MinSpeedThreshold", 0.5);
            stationaryCheckCount = appConfig.GetInt("AI", "StationaryCheckCount", 3);
            waypointMinThreshold = appConfig.GetDouble("AI", "WaypointMinThreshold", 1.5);
            waypointMaxThreshold = appConfig.GetDouble("AI", "WaypointMaxThreshold", 5.0);
            waypointThresholdSpeedFactor = appConfig.GetDouble("AI", "WaypointThresholdSpeedFactor", 0.1);
            collisionDetectionRangeMeters = appConfig.GetDouble("AI", "CollisionDetectionRangeM", 30.0);
            collisionDetectionAngleDegrees = appConfig.GetDouble("AI", "CollisionDetectionAngle", 45.0);
            minimumSafetyDistanceMeters = appConfig.GetDouble("AI", "MinimumSafetyDistanceM", 10.0);
            collisionDetectionHalfWidthMeters = appConfig.GetDouble("AI", "CollisionDetectionHalfWidthM", 2.5);
            pathLoopClosureDistanceMeters = appConfig.GetDouble("AI", "PathLoopClosureDistanceMeters", 5.0);
            trafficLaneHalfWidthMeters = appConfig.GetDouble("AI", "TrafficLaneHalfWidthMeters", 3.0);
            trafficBaseGapMeters = appConfig.GetDouble("AI", "TrafficBaseGapMeters", 4.0);
            trafficTimeHeadwaySeconds = appConfig.GetDouble("AI", "TrafficTimeHeadwaySeconds", 1.5);
            trafficAiSpacingFactor = appConfig.GetDouble("AI", "TrafficAiSpacingFactor", 1.0);
            trafficHumanSpacingFactor = appConfig.GetDouble("AI", "TrafficHumanSpacingFactor", 1.4);
            trafficLookaheadSeconds = appConfig.GetDouble("AI", "TrafficLookaheadSeconds", 3.0);
            trafficLookaheadMinMeters = appConfig.GetDouble("AI", "TrafficLookaheadMinMeters", 15.0);
            trafficBrakeTtcSeconds = appConfig.GetDouble("AI", "TrafficBrakeTtcSeconds", 2.5);
            trafficEmergencyTtcSeconds = appConfig.GetDouble("AI", "TrafficEmergencyTtcSeconds", 1.0);
            spawnMergeHoldLookaheadWaypoints = appConfig.GetInt("AI", "SpawnMergeHoldLookaheadWaypoints", 3);
            spawnMergeHoldDistanceMeters = appConfig.GetDouble("AI", "SpawnMergeHoldDistanceMeters", 15.0);
            recoveryShortReverseMs = appConfig.GetInt("AI", "RecoveryShortReverseMs", 1200);
            recoveryLongReverseMs = appConfig.GetInt("AI", "RecoveryLongReverseMs", 2000);
            recoveryCooldownMs = appConfig.GetInt("AI", "RecoveryCooldownMs", 750);
            recoveryValidationWindowMs = appConfig.GetInt("AI", "RecoveryValidationWindowMs", 2000);
            recoverySuccessDistanceMeters = appConfig.GetDouble("AI", "RecoverySuccessDistanceMeters", 3.0);
            recoverySuccessSpeedKmh = appConfig.GetDouble("AI", "RecoverySuccessSpeedKmh", 8.0);
            recoveryPositionChangeThreshold = appConfig.GetDouble("AI", "RecoveryPositionChangeThreshold", 2.0);
            recoveryProgressStallThreshold = appConfig.GetDouble("AI", "RecoveryProgressStallThreshold", 0.2);
            recoveryStuckCheckIntervalMs = appConfig.GetInt("AI", "RecoveryStuckCheckIntervalMs", 1000);
            recoveryLowSpeedThresholdKmh = appConfig.GetDouble("AI", "RecoveryLowSpeedThresholdKmh", 5.0);
            recoveryDetectionsBeforeAction = appConfig.GetInt("AI", "RecoveryDetectionsBeforeAction", 2);
            recoveryMaxFailureCount = appConfig.GetInt("AI", "RecoveryMaxFailureCount", 3);
            recoveryStallReverseTrigger = appConfig.GetInt("AI", "RecoveryStallReverseTrigger", 3);
            recordingIntervalMeters = appConfig.GetDouble("Recording", "IntervalMeters", 5.0);
            maxPlayers = GetAiManagerInt("MaxPlayers", 48);
            reservedSlots = GetAiManagerInt("ReservedSlots", 10);
            aiFillRatio = GetAiManagerDouble("AiFillRatio", 0.80);
            minAis = GetAiManagerInt("MinAIs", 0);
            maxAis = GetAiManagerInt("MaxAIs", 38);
            adjustIntervalMs = GetAiManagerInt("AdjustIntervalMs", 10000);
            spawnBatchSize = GetAiManagerInt("SpawnBatchSize", 1);
            removeBatchSize = GetAiManagerInt("RemoveBatchSize", 1);
            autoManagePopulation = appConfig.GetBool("AI", "AutoManagePopulation", true);
            insimDebugLogging = appConfig.GetBool("DebugAI", "VerboseInSimLogging", false);
            activeWaypointMarkersEnabled = appConfig.GetBool("DebugAI", "ActiveWaypointMarkers", true);
            activeWaypointIntervalMs = Math.Max(100, appConfig.GetInt("DebugAI", "ActiveWaypointIntervalMs", 500));
            performanceLogging = appConfig.GetBool("DebugAI", "PerformanceLogging", true);
            var laneSection = "LaneChange";
            laneChangeInitialCheckIntervalSeconds =
                appConfig.GetDouble(laneSection, "InitialCheckIntervalSeconds", 30.0);
            laneChangePostCooldownIntervalSeconds =
                appConfig.GetDouble(laneSection, "PostCooldownCheckIntervalSeconds", 10.0);
            laneChangeMergeChance = appConfig.GetDouble(laneSection, "MergeChance", 0.30);
            laneChangeCooldownSeconds = appConfig.GetDouble(laneSection, "CooldownSeconds", 180.0);
            laneChangeSafetyCheckDistanceMeters =
                appConfig.GetDouble(laneSection, "SafetyCheckDistanceMeters", 25.0);
            laneChangeSafetyCheckHalfWidthMeters =
                appConfig.GetDouble(laneSection, "SafetyCheckHalfWidthMeters", 3.5);
            laneChangeMaxParallelDistanceMeters =
                appConfig.GetDouble(laneSection, "MaxParallelDistanceMeters", 12.0);
            laneChangeMaxParallelHeadingDegrees =
                appConfig.GetDouble(laneSection, "MaxParallelHeadingDegrees", 45.0);
            laneChangeParallelLookaheadSeconds =
                appConfig.GetDouble(laneSection, "ParallelLookaheadSeconds", 1.5);
            laneChangeParallelLookaheadMinMeters =
                appConfig.GetDouble(laneSection, "ParallelLookaheadMinMeters", 20.0);
            laneChangeMaxMergeSpeedKmh = appConfig.GetDouble(laneSection, "MaxMergeSpeedKmh", 60.0);
            laneChangeSignalLeadTimeSeconds = appConfig.GetDouble(laneSection, "SignalLeadTimeSeconds", 3.0);
            laneChangeSignalMinimumDurationSeconds =
                appConfig.GetDouble(laneSection, "SignalMinimumDurationSeconds", 5.0);
            laneChangeTransitionLengthMeters =
                appConfig.GetDouble(laneSection, "TransitionLengthMeters", 25.0);
            laneChangeTransitionPointCount = appConfig.GetInt(laneSection, "TransitionPointCount", 12);
            laneChangeTargetAheadWaypoints = appConfig.GetInt(laneSection, "TargetAheadWaypoints", 4);
            laneChangeRearCheckDistanceMeters = appConfig.GetDouble(laneSection, "RearCheckDistanceMeters", 35.0);
            laneChangeRearCheckTtcSeconds = appConfig.GetDouble(laneSection, "RearCheckTtcSeconds", 2.5);
            laneChangeMergeGapFactor = appConfig.GetDouble(laneSection, "MergeGapFactor", 1.2);
            passByReactionEnabled = appConfig.GetBool("AI", "PassByReactionEnabled", true);
            passByReactionChance = appConfig.GetDouble("AI", "PassByReactionChance", 0.10);
            passBySpeedThresholdKmh = appConfig.GetDouble("AI", "PassBySpeedThresholdKmh", 120.0);
            passByReactionDurationSeconds = appConfig.GetDouble("AI", "PassByReactionDurationSeconds", 2.0);
            passByReactionDistanceMeters = appConfig.GetDouble("AI", "PassByReactionDistanceMeters", 25.0);
            passByCooldownMinSeconds = appConfig.GetDouble("AI", "PassByCooldownMinSeconds", 60.0);
            passByCooldownMaxSeconds = appConfig.GetDouble("AI", "PassByCooldownMaxSeconds", 120.0);
            passByProximityResetSeconds = appConfig.GetDouble("AI", "PassByProximityResetSeconds", 5.0);
            passByReactionMode = ParsePassByReactionMode(
                appConfig.GetString("AI", "PassByReactionMode", "FlashAndHorn"));
            buildVersion = appConfig.GetString("AI", "BuildVersion", "dev");

            // Initialize components in the correct order
            waypointManager = new WaypointManager(logger, routeLibrary);
            visualizer = new LFSLayout(logger, insim);

            // Create AIController with dependencies
            aiController = new AIController(insim, logger, waypointManager, visualizer, routeLibrary, debugEnabled,
                appConfig, debugWaypoints, insimDebugLogging, activeWaypointMarkersEnabled, activeWaypointIntervalMs,
                performanceLogging);
            visualizer.LayoutSelectionChanged += aiController.OnLayoutSelectionChanged;
            visualizer.LayoutObjectMoved += aiController.OnLayoutObjectMoved;
            aiController.ApplyRouteConfig(spawnRouteName, mainRouteName, mainAlternateRouteName, branchRouteNames);
            aiController.SetNumberOfAIs(initialAiCount);
            aiController.SetSpawnDelayMs(spawnDelayMs);
            aiController.SetRecordingRouteSelection(currentRecordingRoute);
            aiController.SetLookaheadWaypoints(lookaheadWaypoints);
            aiController.SetRecordingInterval(recordingIntervalMeters);
            aiController.SetWaypointProximityMultiplier(waypointProximityMultiplier);
            aiController.SetSteeringResponseDamping(steeringResponseDamping);
            aiController.SetSteeringDeadzoneDegrees(steeringDeadzoneDegrees);
            aiController.ConfigureControlInputs(throttleBase, brakeBase, minSteering, maxSteering, steeringCenter);
            aiController.ConfigureWaypointThresholds(
                waypointMinThreshold,
                waypointMaxThreshold,
                waypointThresholdSpeedFactor);
            aiController.SetBuildVersion(buildVersion);
            aiController.ConfigureClutch(
                clutchFullyPressed,
                clutchReleased,
                clutchPressDelayMs,
                clutchHoldAfterShiftMs,
                clutchReleaseSteps,
                clutchReleaseIntervalMs,
                stallPreventionRpm,
                stallPreventionReleaseRpm,
                stallPreventionHoldMs);
            aiController.ConfigureGearbox(
                shiftDelayMs,
                gearUpshiftHysteresisKmh,
                gearDownshiftHysteresisKmh,
                gearShiftMinIntervalMs,
                gearSpeedThresholdsKmh);
            aiController.ConfigureCollisionDetection(
                collisionDetectionRangeMeters,
                collisionDetectionAngleDegrees,
                minimumSafetyDistanceMeters,
                collisionDetectionHalfWidthMeters);
            aiController.ConfigurePurePursuit(
                purePursuitEnabled,
                purePursuitLookaheadMinMeters,
                purePursuitLookaheadMaxMeters,
                purePursuitLookaheadSpeedFactor,
                purePursuitWheelbaseMeters,
                purePursuitSteeringGain,
                purePursuitMaxSteerDegrees);
            aiController.ConfigureLaneChange(
                laneChangeInitialCheckIntervalSeconds,
                laneChangePostCooldownIntervalSeconds,
                laneChangeMergeChance,
                laneChangeCooldownSeconds,
                laneChangeSafetyCheckDistanceMeters,
                laneChangeSafetyCheckHalfWidthMeters,
                laneChangeMaxParallelDistanceMeters,
                laneChangeMaxParallelHeadingDegrees,
                laneChangeParallelLookaheadSeconds,
                laneChangeParallelLookaheadMinMeters,
                laneChangeMaxMergeSpeedKmh,
                laneChangeSignalLeadTimeSeconds,
                laneChangeSignalMinimumDurationSeconds,
                laneChangeTransitionLengthMeters,
                laneChangeTransitionPointCount,
                laneChangeTargetAheadWaypoints,
                laneChangeRearCheckDistanceMeters,
                laneChangeRearCheckTtcSeconds,
                laneChangeMergeGapFactor);
            aiController.ConfigureTrafficAwareness(
                pathLoopClosureDistanceMeters,
                trafficLaneHalfWidthMeters,
                trafficBaseGapMeters,
                trafficTimeHeadwaySeconds,
                trafficAiSpacingFactor,
                trafficHumanSpacingFactor,
                trafficLookaheadSeconds,
                trafficLookaheadMinMeters,
                trafficBrakeTtcSeconds,
                trafficEmergencyTtcSeconds,
                spawnMergeHoldLookaheadWaypoints,
                spawnMergeHoldDistanceMeters);
            aiController.ConfigurePassByReactions(
                passByReactionEnabled,
                passByReactionChance,
                passBySpeedThresholdKmh,
                passByReactionDurationSeconds,
                passByReactionDistanceMeters,
                passByReactionMode,
                passByCooldownMinSeconds,
                passByCooldownMaxSeconds,
                passByProximityResetSeconds);
            aiController.ConfigureRecoveryLimits(
                wallRecoveryEnabled,
                waypointTimeoutSeconds,
                progressCheckIntervalMs,
                minRequiredProgress,
                maxRecoveryAttempts,
                maxFailedRecoveryCycles,
                minSpeedThreshold,
                stationaryCheckCount);
            aiController.ConfigureRecovery(
                recoveryShortReverseMs,
                recoveryLongReverseMs,
                recoveryCooldownMs,
                recoveryValidationWindowMs,
                recoverySuccessDistanceMeters,
                recoverySuccessSpeedKmh,
                recoveryPositionChangeThreshold,
                recoveryProgressStallThreshold,
                recoveryStuckCheckIntervalMs,
                recoveryLowSpeedThresholdKmh,
                recoveryDetectionsBeforeAction,
                recoveryMaxFailureCount,
                recoveryStallReverseTrigger);
            aiController.ConfigurePopulationManager(
                maxPlayers,
                reservedSlots,
                aiFillRatio,
                minAis,
                maxAis,
                adjustIntervalMs,
                spawnBatchSize,
                removeBatchSize);
            aiController.SetAutoPopulationEnabled(autoManagePopulation, false);
        }

        /// <summary>
        ///     Main entry point
        /// </summary>
        private static void Main(string[] args)
        {
            logger.Log("Starting AI Car Control Program");

            try
            {
                // Load recorded traffic routes if present
                foreach (var route in GetConfiguredRoutes())
                {
                    waypointManager.LoadTrafficRoute(route);
                }

                // Register event handlers
                RegisterEventHandlers();

                // Initialize connection to LFS
                InitializeInSim();
                aiController.ConnectOutGauge(currentHost, outGaugePort);

                // not sure if we need this, use logging to see if already call the layout, maybe on init
                /*
                insim.Send(new IS_TINY
                {
                    ReqI = 1,
                    SubT = TinyType.TINY_AXM // Request layout from server
                });
                */

                // Request all player information
                GetPlayers();

                // Now initialize debug UI (after InSim is connected)
                aiController.InitializeDebugUI();
                aiController.NotifyRouteValidationIssues();

                // Spawn AI cars automatically if debug is enabled
                if (autoSpawnAI && aiController.IsAutoPopulationEnabled)
                {
                    Thread.Sleep(250); // Wait for connection to stabilize
                    aiController.RecalculatePopulationTargets();
                    logger.Log("Auto-adjusted AI population for debugging");
                }

                // Auto-visualize coordinate system in debug mode
                if (debugCoordinateSystem)
                {
                    Thread.Sleep(3000); // Wait for AI to spawn
                    var plid = aiController.GetFirstAICarID();
                    if (plid > 0)
                    {
                        visualizer.VisualizeCoordinateSystem(plid);
                        logger.Log("Auto-visualized coordinate system for debugging");
                    }
                }

                // Auto-visualize waypoints in debug mode
                if (debugWaypoints)
                {
                    var plid = aiController.GetFirstAICarID();
                    if (plid > 0)
                    {
                        // Get AI paths and visualize them
                        var paths = aiController.GetAIPaths();
                        logger.Log($"Got AI paths with {paths.Count} entries");

                        if (paths.Count > 0)
                        {
                            if (paths.ContainsKey(plid))
                            {
                                if (paths[plid] != null && paths[plid].Count > 0)
                                {
                                    logger.Log($"Visualizing {paths[plid].Count} waypoints for PLID {plid}");
                                    visualizer.VisualizeWaypoints(plid, paths);
                                    visualizer.WaypointsVisualized = true;
                                }
                                else
                                {
                                    logger.LogError($"Path for PLID {plid} is empty");
                                }
                            }
                            else
                            {
                                // Try with the first available path instead
                                var firstPlid = paths.Keys.First();
                                logger.Log($"PLID {plid} not found in paths, using PLID {firstPlid} instead");

                                if (paths[firstPlid] != null && paths[firstPlid].Count > 0)
                                {
                                    logger.Log($"Visualizing {paths[firstPlid].Count} waypoints for PLID {firstPlid}");
                                    visualizer.VisualizeWaypoints(firstPlid, paths);
                                    visualizer.WaypointsVisualized = true;
                                }
                                else
                                {
                                    logger.LogError("No valid paths available to visualize");
                                }
                            }
                        }
                        else
                        {
                            logger.LogError("No AI paths found to visualize");
                        }
                    }
                    else
                {
                    logger.LogError("No AI cars found to visualize waypoints");
                }
            }

            aiController.SyncLayoutToggleState();

            // Keep program running and listen for the 'q' key
            logger.Log("Program is running. Press 'q' to exit cleanly or Ctrl+C to force exit.");

                // Create a thread to listen for the 'q' key
                var keyListenerThread = new Thread(() =>
                {
                    while (true)
                    {
                        var key = Console.ReadKey(true);
                        if (key.KeyChar == 'q' || key.KeyChar == 'Q')
                        {
                            PerformCleanShutdown();
                            break;
                        }
                    }
                });

                // Start the key listener thread as a background thread
                keyListenerThread.IsBackground = true;
                keyListenerThread.Start();

                // Wait indefinitely
                Thread.Sleep(Timeout.Infinite);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Unhandled exception in Main");
                Console.WriteLine("Press any key to exit...");
                Console.ReadKey();
            }
        }

        /// <summary>
        ///     Register event handlers for InSim packets
        /// </summary>
        private static void RegisterEventHandlers()
        {
            logger.Log("Registering event handlers");

            if (insimDebugLogging)
            {
                insim.Initialized += (sender, args) =>
                {
                    logger.Log("InSim client reports Initialized event");
                };
                insim.Disconnected += (sender, e) =>
                {
                    var reason = e?.Reason.ToString() ?? "Unknown";
                    logger.Log($"InSim disconnected (Reason={reason})");
                };
                insim.InSimError += (sender, e) =>
                {
                    if (e?.Exception != null)
                    {
                        logger.LogException(e.Exception, "InSimError raised by InSim client");
                    }
                    else
                    {
                        logger.LogError("InSimError raised by InSim client with no exception detail");
                    }
                };
            }

            insim.IS_NPL += (sender, e) => OnNewPlayer(e.Packet);
            insim.IS_PLL += (sender, e) => aiController.OnPlayerLeave(e.Packet);
            insim.IS_CNL += (sender, e) => aiController.OnConnectionLeave(e.Packet);
            insim.IS_NCN += (sender, e) => aiController.OnNewConnection(e.Packet);

            // Car telemetry
            insim.IS_AII += (sender, e) => aiController.OnAII(e.Packet);
            insim.IS_MCI += (sender, e) => aiController.OnMCI(e.Packet);
            insim.IS_CON += (sender, e) => aiController.OnCollision(e.Packet);

            // Button clicks
            insim.IS_BTC += (sender, e) => OnButtonClick(e.Packet);
            insim.IS_BTT += (sender, e) => OnButtonType(e.Packet);

            // Objects and layout
            insim.IS_AXM += (sender, e) => visualizer.OnAXM(e.Packet);
            insim.IS_CCH += (sender, e) => OnCameraChange(e.Packet);
            // System related
            insim.IS_TINY += (sender, e) => OnTiny(e.Packet);
            insim.IS_VER += (sender, e) => OnVersion(e.Packet);
            insim.IS_STA += (sender, e) => OnState(e.Packet);
            insim.IS_AXI += (sender, e) => OnAutoXInfo(e.Packet);
        }


        /// <summary>
        /// Handle button click responses from the UI.
        /// </summary>
        private static void OnButtonClick(IS_BTC btc)
        {
            if (aiController.TryGetRouteNameForButton(btc.ClickID, out var selectedRoute))
            {
                SetRecordingRoute(selectedRoute);
                aiController.SetVisualizationRouteSelection(selectedRoute);
                aiController.VisualizeSelectedRoute(currentViewPLID);
                return;
            }

            switch (btc.ClickID)
            {
                case MainUI.RecordToggleId:
                    if (aiController.IsRecording) aiController.StopRecording();
                    else aiController.StartRecording(currentRecordingRoute, GetRouteTypeFromName(currentRecordingRoute));
                    break;
                case MainUI.ReloadRoutesId:
                    aiController.ReloadRoutes();
                    break;
                case MainUI.ResetLayoutId:
                    aiController.ResetLayout();
                    break;
                case MainUI.ToggleLayoutId:
                    aiController.ToggleRouteVisualization(currentViewPLID);
                    break;
                case MainUI.StopAllAisId:
                    aiController.StopAllAIs();
                    break;
                case MainUI.StartAllAisId:
                    aiController.StartAllAIs();
                    break;
                case MainUI.StartAutoAisId:
                    aiController.EnableAutoPopulation();
                    break;
                case MainUI.PitAllAisId:
                    aiController.PitAllAIs();
                    break;
                case MainUI.HideUiButtonId:
                    aiController.HideUI();
                    break;
                case MainUI.ShowUiButtonId:
                    aiController.ShowUI();
                    break;
                case MainUI.ToggleDebugButtonsId:
                    aiController.ToggleDebugButtonsVisibility();
                    break;
                case MainUI.RefreshSelectionFeedId:
                    RequestLayoutSelectionFeed();
                    break;
                case MainUI.AddRouteButtonId:
                    SetRecordingRoute(pendingRouteName);
                    aiController.SetVisualizationRouteSelection(pendingRouteName);
                    aiController.VisualizeSelectedRoute(currentViewPLID);
                    break;
                case MainUI.LayoutAttachIndexId:
                    aiController.SetSelectedNodeAsAttachIndex();
                    break;
                case MainUI.LayoutRejoinIndexId:
                    aiController.SetSelectedNodeAsRejoinIndex();
                    break;
                case MainUI.LayoutDeleteNodeId:
                    aiController.DeleteSelectedNode(currentViewPLID);
                    break;
                case MainUI.LayoutExtendRouteId:
                    aiController.ExtendRecordingFromSelection(currentViewPLID);
                    break;
                case MainUI.LayoutSpawnHereId:
                    aiController.SpawnPlayerAtSelection();
                    break;
                case MainUI.VisualizationDetailMinusId:
                    aiController.AdjustVisualizationDetail(false, currentViewPLID);
                    break;
                case MainUI.VisualizationDetailPlusId:
                    aiController.AdjustVisualizationDetail(true, currentViewPLID);
                    break;
                default:
                    aiController.TryRemoveAiFromButton(btc.ClickID);
                    break;
            }
        }

        /// <summary>
        /// Handle text entry responses from InSim buttons.
        /// </summary>
        private static void OnButtonType(IS_BTT btt)
        {
            if (btt.ClickID == MainUI.AddAiDialogId)
            {
                if (int.TryParse(btt.Text, out var count))
                {
                    aiController.SetNumberOfAIs(Math.Max(0, count));
                    aiController.SpawnAICars();
                    insim.SendPrivateMessage(btt.UCID, $"Spawning {Math.Max(0, count)} AI(s)");
                }
                else
                {
                    insim.SendPrivateMessage(btt.UCID, "Enter a valid number of AIs.");
                }
                aiController.HideAddAIDialog();
                return;
            }

            if (btt.ClickID == MainUI.SpeedInputId)
            {
                if (double.TryParse(btt.Text, out var speed))
                {
                    var clampedSpeed = Math.Max(0, speed);
                    aiController.SetTargetSpeedForAll(clampedSpeed);
                    insim.SendPrivateMessage(btt.UCID, $"Set AI speed to {clampedSpeed:F0} km/h");
                }
                else
                {
                    insim.SendPrivateMessage(btt.UCID, "Enter a valid AI speed.");
                }
            }

            if (btt.ClickID == MainUI.RecordingIntervalId)
            {
                if (double.TryParse(btt.Text, out var meters))
                {
                    var interval = Math.Max(0.1, meters);
                    aiController.SetRecordingInterval(interval);
                    insim.SendPrivateMessage(btt.UCID, $"Recording every {interval:F1} m");
                }
                else
                {
                    insim.SendPrivateMessage(btt.UCID, "Enter a valid recording interval in meters.");
                }
            }

            if (btt.ClickID == MainUI.RouteNameInputId)
            {
                var normalized = routeLibrary.NormalizeRouteName(btt.Text);
                pendingRouteName = normalized;
                insim.SendPrivateMessage(btt.UCID,
                    $"Route name ready: {pendingRouteName}. Click Add Route to use it.");
                return;
            }

            if (btt.ClickID == MainUI.NodeSpeedInputId)
            {
                if (double.TryParse(btt.Text, out var speed))
                {
                    aiController.UpdateSelectedNodeSpeed(speed);
                }
                else
                {
                    insim.SendPrivateMessage(btt.UCID, "Enter a valid node speed.");
                }
            }
        }


        /// <summary>
        ///     Initialize InSim connection
        /// </summary>
        private static void InitializeInSim()
        {
            logger.Log($"Initializing InSim connection to {currentHost}:{port}");
            UpdateInSimStatus("InSim: connecting");

            try
            {
                insim.Initialize(new InSimSettings
                {
                    Host = currentHost,
                    Port = port,
                    Admin = "", // No admin password
                    Interval = 100, // Send NLI packet every 100ms
                    Flags = InSimFlags.ISF_MCI | InSimFlags.ISF_AXM_LOAD | InSimFlags.ISF_AXM_EDIT |
                            InSimFlags.ISF_LOCAL | InSimFlags.ISF_CON
                });

                // Request version information
                insim.Send(new IS_TINY { ReqI = 1, SubT = TinyType.TINY_VER });
                insim.Send(new IS_TINY { ReqI = 1, SubT = TinyType.TINY_SST });
                insim.Send(new IS_TINY { ReqI = 1, SubT = TinyType.TINY_AXI });

                // Request layout objects
                insim.Send(new IS_TINY { SubT = TinyType.TINY_AXM, ReqI = 1 });
                visualizer.LayoutObjectsRequested = true;

                RequestLayoutSelectionFeed();

                // Send welcome message
                insim.SendPrivateMessage($"AI Car Control initialized (build {buildVersion})");

                logger.Log("InSim initialized");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to initialize InSim");
                UpdateInSimStatus("InSim: failed to connect");
                throw; // Re-throw to stop program
            }
        }

        /// <summary>
        ///     Request all player information
        /// </summary>
        private static void GetPlayers()
        {
            logger.Log("Requesting all players information");
            insim.Send(new IS_TINY { ReqI = 255, SubT = TinyType.TINY_NPL }); // Request all players
        }

        /// <summary>
        ///     Handle new player events
        /// </summary>
        private static void OnNewPlayer(IS_NPL packet)
        {
            // Handle both individual player joins and responses to our TINY_NPL request
            logger.Log($"Player detected: {packet.PName} (PLID: {packet.PLID}, UCID: {packet.UCID})");

            // Forward to AI controller
            aiController.OnNewPlayer(packet);
        }

        /// <summary>
        ///     Performs a clean shutdown of the program
        /// </summary>
        private static void PerformCleanShutdown()
        {
            logger.Log("Performing clean shutdown...");

            try
            {
                // Clear visualizations
                visualizer.ClearAllVisualizations();

                logger.Log("Cleared all visualizations");

                // Find the first AI car ID
                var aiPlid = aiController.GetFirstAICarID();

                // Send command to spectate the AI car
                if (aiPlid > 0)
                {
                    var aiNameForCmd = aiController.GetAINameForCommand(aiPlid);
                    if (!string.IsNullOrWhiteSpace(aiNameForCmd))
                    {
                        insim.Send(new IS_MST { Msg = $"/spec {aiNameForCmd}" });
                        logger.Log($"Sent command to spectate AI car named {aiNameForCmd}");
                    }
                }

                // Send command to reset AI controls to stop the car
                if (aiPlid > 0)
                {
                    insim.Send(new IS_AIC(new List<AIInputVal>
                            { new AIInputVal { Input = AicInputType.CS_RESET_INPUTS, Time = 0, Value = 0 } })
                        { PLID = aiPlid });
                    logger.Log($"Reset AI controls for PLID={aiPlid}");
                }

                // Send goodbye message
                insim.SendPrivateMessage("AI Control Program shutting down. Press any key to restart.");

                // Close InSim connection gracefully
                insim.DisconnectSafe();

                logger.Log("InSim disconnected successfully");
                logger.Log("Program shutdown complete - exiting");

                // Exit the program
                Environment.Exit(0);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Error during shutdown");
                Environment.Exit(1);
            }
        }

        /// <summary>
        ///     Handle TINY packets
        /// </summary>
        private static void OnTiny(IS_TINY tiny)
        {
            if (tiny.SubT == TinyType.TINY_AXM && visualizer.LayoutObjectsRequested)
                logger.Log("Received TINY_AXM response");
        }

        /// <summary>
        /// Update the active recording route and refresh UI selection.
        /// </summary>
        private static void SetRecordingRoute(string routeName)
        {
            var normalized = routeLibrary.NormalizeRouteName(
                string.IsNullOrWhiteSpace(routeName) ? "main_loop" : routeName);
            currentRecordingRoute = normalized;
            pendingRouteName = normalized;
            aiController.SetRecordingRouteSelection(currentRecordingRoute);
            aiController.RefreshRouteOptions(currentRecordingRoute);
            insim.SendPrivateMessage($"Recording route set to {currentRecordingRoute}");
        }

        /// <summary>
        /// Map a route name to its RouteType for recording metadata.
        /// </summary>
        private static RouteType GetRouteTypeFromName(string routeName)
        {
            var name = (routeName ?? string.Empty).ToLowerInvariant();
            if (name.Contains("alt") || name.Contains("inner")) return RouteType.AlternateMain;
            if (name.Contains("pit")) return RouteType.PitEntry;
            if (name.Contains("detour")) return RouteType.Detour;
            if (name.Contains("main")) return RouteType.MainLoop;
            return RouteType.Unknown;
        }

        private static IEnumerable<string> GetConfiguredRoutes()
        {
            var names = new List<string>();
            if (!string.IsNullOrWhiteSpace(spawnRouteName)) names.Add(spawnRouteName);
            if (!string.IsNullOrWhiteSpace(mainRouteName)) names.Add(mainRouteName);
            if (!string.IsNullOrWhiteSpace(mainAlternateRouteName)) names.Add(mainAlternateRouteName);
            names.AddRange(branchRouteNames);
            return names.Distinct(StringComparer.OrdinalIgnoreCase);
        }

        private static string GetBranchRoute(int index, string fallback)
        {
            if (index >= 0 && index < branchRouteNames.Count)
                return branchRouteNames[index];
            return fallback;
        }

        private static int GetAiManagerInt(string key, int defaultValue)
        {
            return appConfig.GetInt("AIManager", key, appConfig.GetInt("AI", key, defaultValue));
        }

        private static double GetAiManagerDouble(string key, double defaultValue)
        {
            return appConfig.GetDouble("AIManager", key, appConfig.GetDouble("AI", key, defaultValue));
        }

        /// <summary>
        /// Parse comma-separated gear speed thresholds (km/h) from configuration.
        /// </summary>
        private static double[] ParseGearSpeedThresholds(string rawThresholds)
        {
            var defaultThresholds = new[] { 20.0, 40.0, 60.0, 80.0, 100.0 };
            if (string.IsNullOrWhiteSpace(rawThresholds))
                return defaultThresholds;

            var values = new List<double>();
            var parts = rawThresholds.Split(new[] { ',', ';' }, StringSplitOptions.RemoveEmptyEntries);
            foreach (var part in parts)
            {
                if (double.TryParse(part.Trim(), out var value))
                    values.Add(value);
            }

            if (values.Count == 0)
            {
                logger.LogWarning($"Invalid GearSpeedThresholdsKmh '{rawThresholds}', using defaults.");
                return defaultThresholds;
            }

            return values.ToArray();
        }

        /// <summary>
        /// Parse pass-by reaction mode text into a supported enum value.
        /// </summary>
        private static AIConfig.PassByReactionMode ParsePassByReactionMode(string rawMode)
        {
            if (Enum.TryParse(rawMode, true, out AIConfig.PassByReactionMode parsedMode))
                return parsedMode;

            logger.LogWarning($"Unknown PassByReactionMode '{rawMode}', defaulting to FlashAndHorn.");
            return AIConfig.PassByReactionMode.FlashAndHorn;
        }

        /// <summary>
        /// Track the active track/layout and inform downstream components when it changes.
        /// </summary>
        private static void UpdateRouteContext(string trackCode, string layoutName)
        {
            var track = string.IsNullOrWhiteSpace(trackCode) ? "UnknownTrack" : trackCode.Trim();
            var layout = string.IsNullOrWhiteSpace(layoutName) ? "DefaultLayout" : layoutName.Trim();

            if (track.Equals(currentTrackCode, StringComparison.OrdinalIgnoreCase) &&
                layout.Equals(currentLayoutName, StringComparison.OrdinalIgnoreCase))
                return;

            currentTrackCode = track;
            currentLayoutName = layout;

            aiController.SetTrackLayoutContext(currentTrackCode, currentLayoutName);
            aiController.SetRecordingRouteSelection(currentRecordingRoute);
            logger.Log($"Route context changed to Track={currentTrackCode}, Layout={currentLayoutName}");
        }

        private static void OnCameraChange(IS_CCH cch)
        {
            currentViewPLID = cch.PLID;
            aiController.UpdateUIForView(currentViewPLID);
        }

        /// <summary>
        ///     Handle version information
        /// </summary>
        private static void OnVersion(IS_VER ver)
        {
            logger.Log($"Connected to LFS {ver.Version} {ver.Product}, InSim version: {ver.InSimVer}");
            UpdateInSimStatus($"InSim: connected (v{ver.InSimVer})");
        }

        /// <summary>
        /// Handle state updates so we know which track/config is loaded.
        /// </summary>
        private static void OnState(IS_STA sta)
        {
            var track = (sta.Track ?? string.Empty).Trim();
            UpdateRouteContext(track, currentLayoutName);
        }

        /// <summary>
        /// Handle AutoX info packets to learn the current layout name.
        /// </summary>
        private static void OnAutoXInfo(IS_AXI axi)
        {
            var layout = (axi.LName ?? string.Empty).Trim();
            if (string.IsNullOrWhiteSpace(layout)) layout = currentLayoutName;
            UpdateRouteContext(currentTrackCode, layout);
        }

        /// <summary>
        /// Request TTC selection updates so we receive AXM PMO_SELECTION packets for Shift+U picks.
        /// </summary>
        private static void RequestLayoutSelectionFeed()
        {
            try
            {
                logger.Log("Requesting layout selection feed (TTC_SEL_START)");
                // Use a non-zero ReqI so LFS accepts the TTC request without warning
                insim.Send(new IS_TTC { SubT = TtcType.TTC_SEL_START, UCID = 0, ReqI = 1 });
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to request layout selection feed");
            }
        }

        /// <summary>
        /// Reconnect InSim and OutGauge to a target host so the layout editor selection can be tracked on the right instance.
        /// </summary>
        private static void ReconnectInSim(string targetHost)
        {
            if (string.IsNullOrWhiteSpace(targetHost))
            {
                logger.LogWarning("Reconnect requested without a valid host");
                return;
            }

            try
            {
                logger.Log($"Reconnecting InSim to {targetHost}:{port}");
                if (insim != null && insim.IsConnected)
                    insim.Disconnect();
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Error while closing InSim for reconnect");
            }

            currentHost = targetHost;
            InitializeInSim();
            aiController.ConnectOutGauge(currentHost, outGaugePort);
        }

        /// <summary>
        /// Update the main UI with the current InSim status text and host.
        /// </summary>
        private static void UpdateInSimStatus(string status)
        {
            aiController.UpdateInSimStatus(status, $"Host: {currentHost}:{port}");
        }
    }
}
