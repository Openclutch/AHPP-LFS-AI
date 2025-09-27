using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using InSimDotNet;
using InSimDotNet.Packets;

namespace AHPP_Server
{
    internal class Program
    {
        // Ensure log file is written relative to the application directory so it
        // is created correctly when running as a Windows service.
        private static readonly string LogFilePath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "log.txt");
        private static readonly Logger logger = new Logger(LogFilePath);
        private static readonly InSim insim = new InSim();
        private static Dictionary<byte, string> aiPlayers = new Dictionary<byte, string>();
        private static Dictionary<byte, string> allConnections = new Dictionary<byte, string>(); // Track all connections by UCID
        private static readonly List<ObjectInfo> layoutObjects = new List<ObjectInfo>();
        private static bool layoutObjectsRequested;
        private static CompCar[] allCars = new CompCar[0];
        private const string MOD_ID = "209927";

        private static void Main(string[] args)
        {
            logger.Log("Starting Gen Server Manager");
            insim.Bind<IS_NPL>(OnNewPlayer);
            insim.Bind<IS_MCI>(OnMCI);
            //insim.Bind<IS_AXM>(OnAXM);
            insim.Bind<IS_TINY>(OnTiny);
            insim.Bind<IS_PLP>(PlayerLeftPit);
            insim.Bind<IS_MSO>(ProcessMessage);
            insim.Bind<IS_NCN>(OnNewConnection); // Track new connections
            insim.Bind<IS_CNL>(OnConnectionLeave); // Track connection leaves

            insim.Initialize(new InSimSettings
            {
                Host = "162.244.55.41",
                Port = 50096,
                Admin = "AHPP123456",
                Flags = InSimFlags.ISF_MCI | InSimFlags.ISF_AXM_LOAD | InSimFlags.ISF_AXM_EDIT | InSimFlags.ISF_MSO_COLS
            });

            insim.Send(new IS_SMALL { SubT = SmallType.SMALL_NLI, UVal = 100 });
            insim.Send(new IS_TINY { SubT = TinyType.TINY_AXM, ReqI = 1 });
            layoutObjectsRequested = true;
            logger.Log("Requested current layout objects with TINY_AXM");

            // Request all current connections on startup
            insim.Send(new IS_TINY { SubT = TinyType.TINY_NCN, ReqI = 255 });
            logger.Log("Requested all current connections with TINY_NCN");

            Thread.Sleep(Timeout.Infinite);
        }

        private static void OnNewConnection(InSim insim, IS_NCN ncn)
        {
            // Track all connections by UCID
            if (ncn.UCID != 0) // Exclude host (UCID 0)
            {
                allConnections[ncn.UCID] = ncn.UName; // Use username for spectating
                logger.Log($"New connection: UCID={ncn.UCID}, UName={ncn.UName}");
            }
        }

        private static void OnConnectionLeave(InSim insim, IS_CNL cnl)
        {
            // Remove connection from tracking when they disconnect
            if (allConnections.ContainsKey(cnl.UCID))
            {
                logger.Log($"Connection {allConnections[cnl.UCID]} (UCID: {cnl.UCID}) disconnected");
                allConnections.Remove(cnl.UCID);
            }
            // Check aiPlayers by PLID (if mapped elsewhere)
        }

        private static void OnNewPlayer(InSim insim, IS_NPL npl)
        {
            string playerName = npl.PName;
            byte plid = npl.PLID;

            if (IsAI(playerName))
            {
                Console.WriteLine($"AI detected: {playerName} (PLID: {plid})");
                aiPlayers[plid] = playerName;
                AssignModCarToAI(playerName, MOD_ID);
            }
            else
            {
                Console.WriteLine($"Human player joined: {playerName} (PLID: {plid})");
            }

            logger.Log($"New player spawned: PLID={plid}, Name={playerName}");
        }

        private static bool IsAI(string playerName)
        {
            return playerName.StartsWith("AI", StringComparison.OrdinalIgnoreCase) || playerName == "Track";
        }

        private static void AssignModCarToAI(string aiName, string modId)
        {
            string carCommand = $"/car {aiName} {modId}";
            insim.Send(new IS_MST
            {
                Msg = carCommand
            });
            Console.WriteLine($"Assigned mod car {modId} to AI: {aiName}");
            logger.Log($"Assigned mod car {modId} to AI: {aiName}");
        }

        private static void PlayerLeftPit(InSim insim, IS_PLP plp)
        {
            byte plid = plp.PLID;
            if (aiPlayers.ContainsKey(plid))
            {
                Console.WriteLine($"AI {aiPlayers[plid]} left the session.");
                aiPlayers.Remove(plid);
                logger.Log($"AI {aiPlayers[plid]} left the session.");
            }
        }

        private static void OnMCI(InSim insim, IS_MCI mci)
        {
            allCars = mci.Info.ToArray();
        }

        private static void OnTiny(InSim insim, IS_TINY tiny)
        {
            if (tiny.SubT == TinyType.TINY_AXM && layoutObjectsRequested)
                logger.Log("Received TINY_AXM response");
        }

        private static void OnAXM(InSim insim, IS_AXM axm)
        {
            logger.Log($"AXM received: PMOAction={axm.PMOAction}, NumO={axm.NumO}");
            if (axm.PMOAction == ActionFlags.PMO_LOADING_FILE || axm.PMOAction == ActionFlags.PMO_ADD_OBJECTS ||
                axm.PMOAction == ActionFlags.PMO_TINY_AXM)
                foreach (var obj in axm.Info)
                {
                    logger.Log(
                        $"Object: Index={obj.Index}, X={obj.X / 32.0:F2}m, Y={obj.Y / 32.0:F2}m, Z={obj.Zbyte / 4.0:F2}m, Heading={obj.Heading}, Flags={obj.Flags}");
                    layoutObjects.Add(obj);
                }
        }

        public static void ProcessMessage(InSim insim, IS_MSO mso)
        {
            logger.Log($"Received message: {mso.Msg} (UCID: {mso.UCID})");

            if (mso.UCID == 0 || mso.UserType == UserType.MSO_SYSTEM)
                return;

            string commandText = mso.Msg.Substring(mso.TextStart, mso.Msg.Length - mso.TextStart).Trim();

            if (!commandText.StartsWith("!"))
                return;

            string[] args = commandText.Substring(1).Split(' ');
            string command = args[0].ToLower();

            switch (command)
            {
                case "help":
                case "h":
                    insim.Send(mso.UCID, "^7–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–");
                    insim.Send(mso.UCID, "^7General commands:");
                    insim.Send(mso.UCID, "^7• ^2!help^7 or ^2!h^7 - shows this help message");
                    insim.Send(mso.UCID, "^7• ^2!spawntrack^7 - spawns AI track car with mod ID 3320A7");
                    logger.Log($"Sent help text to UCID {mso.UCID}");
                    break;

                case "spawntrack":
                    SpawnTrack();
                    insim.Send(mso.UCID, "^7Spawning AI track car...");
                    logger.Log($"SpawnTrack command issued by UCID {mso.UCID}");
                    break;

                default:
                    insim.Send(mso.UCID, $"^7Unknown command: ^2!{command}^7. Type ^2!help^7 for available commands.");
                    logger.Log($"Unknown command '!{command}' from UCID {mso.UCID}");
                    break;
            }
        }

        public static async void SpawnTrack()
        {
            insim.Send("Spectating all players to spawn track!");
            await Task.Delay(1000);

            // Spectate all connections to clear pit spots
            foreach (var connection in allConnections)
            {
                string username = connection.Value;
                insim.Send(new IS_MST { Msg = $"/spec {username}" });
                logger.Log($"Spectated connection: {username} (UCID: {connection.Key})");
                await Task.Delay(50); // Small delay to ensure commands process sequentially
            }

            // Additional delay to ensure all players are spectated
            await Task.Delay(500);

            // Spawn the AI in the first pit spot
            insim.Send(new IS_MST { Msg = "/ai Track" });
            logger.Log("Spawned AI 'Track' after spectating all connections");
        }

        /// <summary>
        /// Provides simple thread-safe file logging so that information can be
        /// recorded even when the application runs as a Windows service.
        /// </summary>
        private class Logger
        {
            private readonly string _filePath;
            private readonly object _lockObject = new object();

            /// <summary>
            /// Initializes the logger with the absolute path to the log file.
            /// </summary>
            /// <param name="filePath">Absolute path where log entries will be stored.</param>
            public Logger(string filePath)
            {
                _filePath = filePath;
            }

            /// <summary>
            /// Writes a timestamped message to the log file and standard output.
            /// </summary>
            /// <param name="message">Text to log.</param>
            public void Log(string message)
            {
                string logMessage = $"{DateTime.Now:HH:mm:ss.fff} {message}";
                Console.WriteLine(logMessage);
                lock (_lockObject)
                {
                    File.AppendAllText(_filePath, logMessage + Environment.NewLine);
                }
            }
        }
    }
}