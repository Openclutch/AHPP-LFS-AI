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
            bool debugEnabled = false)
        {
            this.insim = insim;
            this.logger = logger;
            this.waypointManager = waypointManager;
            this.lfsLayout = lfsLayout;

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
            routeRecorder = new RouteRecorder(logger, lfsLayout, debugUI);
            mainUI = new MainUI(insim, logger);
            outGauge = new OutGauge();
            outGauge.PacketReceived += OnOutGauge;
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

        public void StartRecording(string name, string type = "main")
        {
            routeRecorder.Start(name, type);
        }

        public void UpdateUIForView(byte viewPlid)
        {
            if (!config.DebugEnabled || debugUI == null) return;

            if (viewPlid == playerPLID)
            {
                debugUI.ShowRecordingButtons(true);
                debugUI.ShowAIButtons(false);
            }
            else if (aiPLIDs.Contains(viewPlid))
            {
                debugUI.ShowRecordingButtons(false);
                debugUI.ShowAIButtons(true);
            }
            else
            {
                debugUI.ShowRecordingButtons(false);
                debugUI.ShowAIButtons(false);
            }
        }

        public void StopRecording()
        {
            routeRecorder.Stop();
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
            mainUI.ShowAddAIDialog();
        }

        public void HideAddAIDialog()
        {
            mainUI.HideAddAIDialog();
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
            foreach (var plid in aiPLIDs)
            {
                insim.Send(new IS_MST { Msg = $"/spec {plid}" });
            }
            aiPLIDs.Clear();
            mainUI.UpdateAIList(GetAiTuples());
        }

        public void StopAllAIs()
        {
            foreach (var plid in aiPLIDs)
            {
                driver.StopCar(plid);
                waypointFollower.SetManualTargetSpeed(plid, 0);
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
                // Ensure recording buttons are visible when the player joins
                debugUI.ShowRecordingButtons(true);
                debugUI.ShowAIButtons(false);
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
    }
}
