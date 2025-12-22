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
        private readonly Dictionary<string, RouteMetadata> routePresets = new Dictionary<string, RouteMetadata>(StringComparer.OrdinalIgnoreCase);
        private readonly MainUI mainUI;
        private readonly Dictionary<byte, string> currentRoute = new Dictionary<byte, string>();
        private readonly Random branchRandom = new Random();
        private readonly OutGauge outGauge;

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

            pathManager = new PathManager(waypointManager, logger);
            pathManager.LoadRoutes(config);

            // Initialize debug UI if enabled
            if (debugEnabled) debugUI = new DebugUI(insim, logger);

            // Create component hierarchy
            var steeringCalculator = new SteeringCalculator(logger);
            waypointFollower = new WaypointFollower(config, logger, steeringCalculator);
            gearboxController = new GearboxController(config, logger);
            driver = new AIDriver(config, logger, waypointFollower, gearboxController, insim);
            driver.SetRecoveryFailedHandler(ResetAI);
            routeRecorder = new RouteRecorder(logger, lfsLayout, debugUI, routeLibrary, mainUI);
            mainUI = new MainUI(insim, logger);
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
            if (metadata == null) return null;
            return new RouteMetadata
            {
                Name = metadata.Name,
                Type = metadata.Type,
                Description = metadata.Description,
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

            if (type != RouteType.Unknown) metadata.Type = type;
            if (metadata.Type == RouteType.MainLoop) metadata.IsLoop = true;
            if (attachIndex.HasValue) metadata.AttachMainIndex = attachIndex;
            if (rejoinIndex.HasValue) metadata.RejoinMainIndex = rejoinIndex;
            if (metadata.Type == RouteType.Unknown) metadata.Type = routeLibrary.GuessRouteType(metadata.Name);
            if (!metadata.DefaultSpeedLimit.HasValue) metadata.DefaultSpeedLimit = 60;

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
            routeRecorder.Start(metadata);
        }

        public void UpdateUIForView(byte viewPlid)
        {
            if (!config.DebugEnabled || debugUI == null) return;

            if (aiPLIDs.Contains(viewPlid))
            {
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
            mainUI?.UpdateRecordingRouteSelection(routeName);
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
            mainUI.UpdateAIList(GetAiTuples());
        }

        public void RemoveAllAICars()
        {
            foreach (var plid in aiPLIDs.Where(p => p > 0))
            {
                insim.Send(new IS_MST { Msg = $"/spec {plid}" });
            }
            aiPLIDs.Clear();
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

        public void SetTargetSpeedForAll(double speed)
        {
            foreach (var plid in aiPLIDs)
            {
                waypointFollower.SetManualTargetSpeed(plid, speed);
            }
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

                    // Assign spawn path
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

                foreach (var plid in aiPLIDs)
                {
                    var (targetIndex, _, targetSpeed, _) = waypointFollower.GetFollowerInfo(plid);
                    waypointIndices[plid] = targetIndex;
                    targetSpeeds[plid] = targetSpeed;
                    controlInfo[plid] = driver.GetControlInfo(plid);

                    // Place active waypoint marker
                    var path = waypointFollower.GetPath(plid);
                    if (path != null && targetIndex >= 0 && targetIndex < path.Count)
                        lfsLayout.VisualizeActiveWaypoint(plid, path[targetIndex]);
                }

                // Get paths for all AIs
                var paths = new Dictionary<byte, List<Util.Waypoint>>();
                foreach (var plid in aiPLIDs) paths[plid] = waypointFollower.GetPath(plid);

                debugUI.UpdateDebugInfo(allCars, waypointIndices, paths, targetSpeeds, controlInfo);
            }
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
                    waypointFollower.SetPath(plid, pathManager.MainRoute);
                    currentRoute[plid] = "main";
                }
                else if (routeName == "main")
                {
                    if (pathManager.TryGetBranch(targetIndex, out var branchPath) && branchRandom.NextDouble() < 0.5)
                    {
                        waypointFollower.SetPath(plid, branchPath);
                        currentRoute[plid] = "branch";
                    }
                }
                else if (routeName == "branch")
                {
                    var path = waypointFollower.GetPath(plid);
                    if (targetIndex >= path.Count - 1)
                    {
                        waypointFollower.SetPath(plid, pathManager.MainRoute);
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
        public void ReloadRoutes()
        {
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

            insim.Send(new IS_MST { Msg = "Routes reloaded from disk" });
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
