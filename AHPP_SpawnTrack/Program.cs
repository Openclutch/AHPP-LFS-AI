using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Serialization;
using InSimDotNet;
using InSimDotNet.Packets;

namespace AHPP_SpawnTrack
{
    // Configuration class to store connection settings
    [Serializable]
    public class AppConfig
    {
        public string LocalHost { get; set; } = "127.0.0.1";
        public int LocalPort { get; set; } = 29999;
        public string LocalAdmin { get; set; } = "";

        public string OnlineHost { get; set; } = "162.244.55.41";
        public int OnlinePort { get; set; } = 12345;
        public string OnlineAdmin { get; set; } = "password";

        public int CheckIntervalMs { get; set; } = 10000;

        public string LogFilePath { get; set; } = "log.txt";

        public bool CommentsOnEntry { get; set; } = true;
    }

    internal class Program
    {
        private static AppConfig config;
        private static readonly string ConfigFilePath = "config.xml";
        private static Logger logger;
        private static readonly InSim insimLocal = new InSim();
        private static readonly InSim insimOnline = new InSim();
        private static readonly Dictionary<byte, string> aiPlayers = new Dictionary<byte, string>();

        private static readonly Dictionary<byte, string>
            allConnections = new Dictionary<byte, string>(); // Track all connections by UCID

        private static bool isRunning = true;
        private static bool isTrackAISpawned;
        private static readonly List<CompCar> allCars = new List<CompCar>();

        private static void Main(string[] args)
        {
            // Load or create config
            LoadOrCreateConfig();

            // Initialize logger with path from config
            logger = new Logger(config.LogFilePath);

            logger.Log("Starting Local and Online LFS InSim App");

            SetupInSim();

            // Main loop that checks if AI Track is spawned
            Task.Run(() => MainLoop());

            // Keep the application running
            Console.WriteLine("Press ESC to exit");
            while (isRunning)
            {
                if (Console.KeyAvailable)
                {
                    var key = Console.ReadKey(true);
                    if (key.Key == ConsoleKey.Escape) isRunning = false;
                }

                Thread.Sleep(100);
            }

            // Clean up
            logger.Log("Shutting down InSim connections");
            insimOnline.Disconnect();
            insimLocal.Disconnect();
        }

        private static void LoadOrCreateConfig()
        {
            if (File.Exists(ConfigFilePath))
            {
                try
                {
                    using (var fs = new FileStream(ConfigFilePath, FileMode.Open))
                    {
                        var serializer = new XmlSerializer(typeof(AppConfig));
                        config = (AppConfig)serializer.Deserialize(fs);
                        Console.WriteLine("Configuration loaded successfully");
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error loading config: {ex.Message}");
                    Console.WriteLine("Creating default configuration");
                    config = new AppConfig();
                    SaveConfig();
                }
            }
            else
            {
                Console.WriteLine("No configuration file found. Creating default configuration");
                config = new AppConfig();
                SaveConfig();
            }
        }

        private static void SaveConfig()
        {
            try
            {
                using (var fs = new FileStream(ConfigFilePath, FileMode.Create))
                {
                    var serializer = new XmlSerializer(typeof(AppConfig));
                    serializer.Serialize(fs, config);
                }

                Console.WriteLine("Configuration saved successfully");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error saving config: {ex.Message}");
            }
        }

        private static void SetupInSim()
        {
            insimOnline.Bind<IS_NPL>(OnNewPlayer);
            insimOnline.Bind<IS_MCI>(OnMCI);
            insimOnline.Bind<IS_TINY>(OnTiny);
            insimOnline.Bind<IS_PLP>(PlayerLeftPit);
            insimOnline.Bind<IS_MSO>(ProcessMessage);
            insimOnline.Bind<IS_NCN>(OnNewConnection);
            insimOnline.Bind<IS_CNL>(OnConnectionLeave);
            insimOnline.Bind<IS_UCO>(InsimCircle);

            // Initialize with Online connection settings from config
            insimOnline.Initialize(new InSimSettings
            {
                Host = config.OnlineHost,
                Port = config.OnlinePort,
                Admin = config.OnlineAdmin,
                Flags = InSimFlags.ISF_MCI | InSimFlags.ISF_MSO_COLS
            });

            logger.Log($"InSim initialized on online host: {config.OnlineHost}:{config.OnlinePort}");

            // Initialize with local connection settings from config
            insimLocal.Initialize(new InSimSettings
            {
                Host = config.LocalHost,
                Port = config.LocalPort,
                Admin = config.LocalAdmin
                //Flags = InSimFlags.ISF_MCI | InSimFlags.ISF_MSO_COLS
            });

            logger.Log($"InSim initialized on localhost: {config.LocalHost}:{config.LocalPort}");

            // Request all current connections and players
            insimOnline.Send(new IS_TINY { SubT = TinyType.TINY_NCN, ReqI = 255 });
            logger.Log("Requested all current connections");

            insimOnline.Send(new IS_TINY { SubT = TinyType.TINY_NPL, ReqI = 254 });
            logger.Log("Requested all current players");
        }

        private static async Task MainLoop()
        {
            logger.Log("Starting main monitoring loop");

            while (isRunning)
                try
                {
                    // Check if AI Track is spawned
                    CheckAndSpawnTrackAI();

                    // Wait before next check - use interval from config
                    await Task.Delay(config.CheckIntervalMs);
                }
                catch (Exception ex)
                {
                    logger.Log($"Error in main loop: {ex.Message}");
                    await Task.Delay(5000); // Wait a bit longer if there was an error
                }
        }

        private static void CheckAndSpawnTrackAI()
        {
            // Reset flag before checking
            isTrackAISpawned = false;

            // Check if "Track" AI is in the list of AI players
            foreach (var aiPlayer in aiPlayers)
                if (aiPlayer.Value == "Track")
                {
                    isTrackAISpawned = true;
                    break;
                }

            if (!isTrackAISpawned)
            {
                logger.Log("AI Track not detected, spawning now...");
                SpawnTrack();
            }
            else
            {
                logger.Log("AI Track is already spawned");
            }
        }

        private static void OnNewConnection(InSim insim, IS_NCN ncn)
        {
            // Track all connections by UCID
            if (ncn.UCID != 0) // Exclude host (UCID 0)
            {
                allConnections[ncn.UCID] = ncn.UName;
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
        }

        private static void OnNewPlayer(InSim insim, IS_NPL npl)
        {
            var playerName = npl.PName;
            var plid = npl.PLID;

            if (IsAI(playerName))
            {
                logger.Log($"AI detected: {playerName} (PLID: {plid})");
                aiPlayers[plid] = playerName;

                if (playerName == "Track")
                {
                    isTrackAISpawned = true;
                    logger.Log("AI Track has been spawned");
                }
            }
            else
            {
                logger.Log($"Human player joined: {playerName} (PLID: {plid})");
            }
        }

        private static bool IsAI(string playerName)
        {
            return playerName.StartsWith("AI", StringComparison.OrdinalIgnoreCase) || playerName == "Track";
        }

        private static void PlayerLeftPit(InSim insim, IS_PLP plp)
        {
            var plid = plp.PLID;
            if (aiPlayers.ContainsKey(plid))
            {
                var aiName = aiPlayers[plid];
                logger.Log($"AI {aiName} left the pit");

                // If it was the Track AI, note that in logs but keep it in our list
                if (aiName == "Track") logger.Log("AI Track left the pit");
            }
        }

        public static void InsimCircle(InSim insim, IS_UCO uco)
        {
        }

        private static void OnMCI(InSim insim, IS_MCI mci)
        {
            // Update our list of cars on track
            allCars.Clear();
            allCars.AddRange(mci.Info);

            // Check if any AI has disconnected but wasn't properly removed
            var disconnectedAIs = new List<byte>();
            foreach (var ai in aiPlayers)
            {
                var found = false;
                foreach (var car in allCars)
                    if (car.PLID == ai.Key)
                    {
                        found = true;
                        break;
                    }

                if (!found)
                {
                    disconnectedAIs.Add(ai.Key);

                    // If it was the Track AI, set the flag to false
                    if (ai.Value == "Track")
                    {
                        isTrackAISpawned = false;
                        logger.Log("AI Track is no longer on track");
                    }
                }
            }

            // Remove disconnected AIs from our list
            foreach (var plid in disconnectedAIs)
            {
                logger.Log($"Removing disconnected AI: {aiPlayers[plid]} (PLID: {plid})");
                aiPlayers.Remove(plid);
            }
        }

        private static void OnTiny(InSim insim, IS_TINY tiny)
        {
            // Handle packet confirmations if needed
        }

        public static void ProcessMessage(InSim insim, IS_MSO mso)
        {
            if (mso.UCID == 0 || mso.UserType == UserType.MSO_SYSTEM)
                return;

            var commandText = mso.Msg.Substring(mso.TextStart, mso.Msg.Length - mso.TextStart).Trim();

            if (!commandText.StartsWith("!"))
                return;

            var args = commandText.Substring(1).Split(' ');
            var command = args[0].ToLower();

            switch (command)
            {
                case "help":
                case "h":
                    insimOnline.Send(mso.UCID, "^7–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–—–");
                    insimOnline.Send(mso.UCID, "^7Commands:");
                    insimOnline.Send(mso.UCID, "^7• ^2!help^7 or ^2!h^7 - shows this help message");
                    insimOnline.Send(mso.UCID, "^7• ^2!spawntrack^7 - spawns AI track car");
                    insimOnline.Send(mso.UCID, "^7• ^2!status^7 - shows if Track AI is spawned");
                    insimOnline.Send(mso.UCID, "^7• ^2!reload^7 - reloads the configuration file");
                    logger.Log($"Sent help text to UCID {mso.UCID}");
                    break;

                case "spawntrack":
                    SpawnTrack();
                    insimOnline.Send(mso.UCID, "^7Spawning AI track car...");
                    logger.Log($"SpawnTrack command issued by UCID {mso.UCID}");
                    break;

                case "status":
                    var status = isTrackAISpawned ? "^2AI Track is spawned" : "^1AI Track is not spawned";
                    insimOnline.Send(mso.UCID, status);
                    logger.Log($"Status command issued by UCID {mso.UCID}");
                    break;

                case "reload":
                    LoadOrCreateConfig();
                    insimOnline.Send(mso.UCID, "^7Configuration reloaded");
                    logger.Log($"Reload config command issued by UCID {mso.UCID}");
                    break;

                default:
                    insimOnline.Send(mso.UCID,
                        $"^7Unknown command: ^2!{command}^7. Type ^2!help^7 for available commands.");
                    logger.Log($"Unknown command '!{command}' from UCID {mso.UCID}");
                    break;
            }
        }

        public static async void SpawnTrack()
        {
            insimOnline.Send(new IS_MST { Msg = "Spectating all players to spawn track!" });
            logger.Log("Starting Track AI spawn sequence");
            await Task.Delay(500);

            // Spectate all connections to clear pit spots
            foreach (var connection in allConnections)
            {
                var username = connection.Value;
                insimOnline.Send(new IS_MST { Msg = $"/spec {username}" });
                logger.Log($"Spectated connection: {username} (UCID: {connection.Key})");
                await Task.Delay(100); // Small delay to ensure commands process sequentially
            }

            // Additional delay to ensure all players are spectated
            await Task.Delay(500);

            // Spawn the AI in the first pit spot
            insimLocal.Send(new IS_MST { Msg = "/ai Track" });
            logger.Log("Spawned AI 'Track' after spectating all connections");

            // Don't set isTrackAISpawned = true here
            // We'll wait for the NPL packet to confirm it was spawned
        }

        private class Logger
        {
            private readonly string _filePath;
            private readonly object _lockObject = new object();

            public Logger(string filePath)
            {
                _filePath = filePath;
                try
                {
                    File.WriteAllText(_filePath, $"=== Log started at {DateTime.Now} ===\r\n");
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error initializing log file: {ex.Message}");
                }
            }

            public void Log(string message)
            {
                var logMessage = $"{DateTime.Now:HH:mm:ss.fff} {message}";
                Console.WriteLine(logMessage);
                try
                {
                    lock (_lockObject)
                    {
                        File.AppendAllText(_filePath, logMessage + Environment.NewLine);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error writing to log: {ex.Message}");
                }
            }
        }
    }
}