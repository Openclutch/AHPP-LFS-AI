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
        private static readonly RouteLibrary routeLibrary;
        private static readonly WaypointManager waypointManager;
        private static readonly LFSLayout visualizer;
        private static readonly AIController aiController;

        // Debug settings
        private static readonly bool debugEnabled = true; // enable debug UI buttons
        private static readonly bool debugWaypoints = true; // spawn ai to test waypoints
        private static readonly bool debugCoordinateSystem = false; // spawn objs to show path

        // Host settings
        private static readonly string host = "10.211.55.4"; // Local host
        private static readonly int port = 29999; // Default InSim port
        private static byte currentViewPLID = 0;

        /// <summary>
        ///     Static constructor for initialization of components
        /// </summary>
        static Program()
        {
            // Initialize components in the correct order
            routeLibrary = new RouteLibrary(logger);
            waypointManager = new WaypointManager(logger, routeLibrary);
            visualizer = new LFSLayout(logger, insim);

            // Create AIController with dependencies
            aiController = new AIController(insim, logger, waypointManager, visualizer, routeLibrary, debugEnabled);
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
                waypointManager.LoadTrafficRoute("pit_entry");
                waypointManager.LoadTrafficRoute("main_loop");
                waypointManager.LoadTrafficRoute("detour1");
                waypointManager.LoadTrafficRoute("detour2");
                waypointManager.LoadTrafficRoute("detour3");

                // Register event handlers
                RegisterEventHandlers();

                // Initialize connection to LFS
                InitializeInSim();
                aiController.ConnectOutGauge(host, 30000);

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

                // Spawn AI cars automatically if debug is enabled
                if (debugWaypoints)
                {
                    Thread.Sleep(250); // Wait for connection to stabilize
                    aiController.SpawnAICars();
                    logger.Log("Auto-spawned AI cars for debugging");
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

            insim.IS_NPL += (_, e) => OnNewPlayer(e.Packet);

            // Car telemetry
            insim.IS_AII += (_, e) => aiController.OnAII(e.Packet);
            insim.IS_MCI += (_, e) => aiController.OnMCI(e.Packet);

            // Button clicks
            insim.IS_BTC += (_, e) => OnButtonClick(e.Packet);
            insim.IS_BTT += (_, e) => OnButtonType(e.Packet);

            // Objects and layout
            insim.IS_AXM += (_, e) => visualizer.OnAXM(e.Packet);
            insim.IS_CCH += (_, e) => OnCameraChange(e.Packet);
            // System related
            insim.IS_TINY += (_, e) => OnTiny(e.Packet);
            insim.IS_VER += (_, e) => OnVersion(e.Packet);
        }


        private static void OnButtonClick(IS_BTC btc)
        {
            switch (btc.ClickID)
            {
                case 1:
                    if (aiController.IsRecording) aiController.StopRecording();
                    else aiController.StartRecording("main_loop", RouteType.MainLoop);
                    break;
                case 2:
                    aiController.ReloadRoutes();
                    break;
                case 5:
                    aiController.ToggleRouteVisualization(currentViewPLID);
                    break;
                case 101:
                    aiController.ShowAddAIDialog();
                    break;
                case 103:
                    aiController.StopAllAIs();
                    break;
                case 104:
                    aiController.SpectateAllAIs();
                    break;
                case 239:
                    if (aiController.IsRecording) aiController.StopRecording();
                    else aiController.StartRecording("main_loop", RouteType.MainLoop);
                    break;
                case 238:
                    if (aiController.IsRecording) aiController.StopRecording();
                    else aiController.StartRecording("pit_entry", RouteType.PitEntry);
                    break;
                case 237:
                    if (aiController.IsRecording) aiController.StopRecording();
                    else aiController.StartRecording("detour1", RouteType.Detour);
                    break;
                case 212:
                    aiController.SpawnAICars();
                    break;
                case 211:
                    aiController.RemoveLastAICar();
                    break;
                case 210:
                    aiController.RemoveAllAICars();
                    break;
                case 209:
                    aiController.StopAllAIs();
                    break;
                case 208:
                    aiController.SpectateAllAIs();
                    break;
                case 207:
                    aiController.SetTargetSpeedForAll(50);
                    break;
            }
        }

        private static void OnButtonType(IS_BTT btt)
        {
            if (btt.ClickID == MainUI.AddAiDialogId)
            {
                if (int.TryParse(btt.Text, out var count))
                {
                    aiController.SetNumberOfAIs(count);
                    aiController.SpawnAICars();
                }
                aiController.HideAddAIDialog();
            }
        }


        /// <summary>
        ///     Initialize InSim connection
        /// </summary>
        private static void InitializeInSim()
        {
            logger.Log($"Initializing InSim connection to {host}:{port}");

            try
            {
                insim.Initialize(new InSimSettings
                {
                    Host = host,
                    Port = port,
                    Admin = "", // No admin password
                    Interval = 100, // Send NLI packet every 100ms
                    Flags = InSimFlags.ISF_MCI | InSimFlags.ISF_AXM_LOAD | InSimFlags.ISF_AXM_EDIT |
                            InSimFlags.ISF_LOCAL | InSimFlags.ISF_CON
                });

                // Request version information
                insim.Send(new IS_TINY { ReqI = 1, SubT = TinyType.TINY_VER });

                // Request layout objects
                insim.Send(new IS_TINY { SubT = TinyType.TINY_AXM, ReqI = 1 });
                visualizer.LayoutObjectsRequested = true;

                // Send welcome message
                insim.Send(new IS_MST { Msg = "AI Car Control initialized" });

                logger.Log("InSim initialized");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to initialize InSim");
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
                    insim.Send(new IS_MST { Msg = $"/spec {aiPlid}" });
                    logger.Log($"Sent command to spectate AI car PLID={aiPlid}");
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
                insim.Send(new IS_MST { Msg = "AI Control Program shutting down. Press any key to restart." });

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
        }
    }
}
