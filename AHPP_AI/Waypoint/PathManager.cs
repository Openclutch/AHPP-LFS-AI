using System;
using System.Collections.Generic;
using System.Linq;
using AHPP_AI.AI;
using AHPP_AI.Debug;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    /// Describes a detour branch with metadata and start/end indices on the main route.
    /// </summary>
    public class BranchRouteInfo
    {
        public string Name { get; set; }
        public List<Util.Waypoint> Path { get; set; }
        public int StartIndex { get; set; }
        public int RejoinIndex { get; set; }
        public RouteMetadata Metadata { get; set; }
    }

    /// <summary>
    /// Maintains the different traffic routes for AI including spawn, main loop and optional branches.
    /// </summary>
    public class PathManager
    {
        private readonly WaypointManager waypointManager;
        private readonly Logger logger;
        private readonly Random random = new Random();
        private readonly RouteLibrary routeLibrary;

        public List<Util.Waypoint> SpawnRoute { get; private set; } = new List<Util.Waypoint>();
        public List<Util.Waypoint> MainRoute { get; private set; } = new List<Util.Waypoint>();

        private readonly Dictionary<string, BranchRouteInfo> branches =
            new Dictionary<string, BranchRouteInfo>(StringComparer.OrdinalIgnoreCase);

        public PathManager(WaypointManager waypointManager, Logger logger, RouteLibrary routeLibrary)
        {
            this.waypointManager = waypointManager;
            this.logger = logger;
            this.routeLibrary = routeLibrary;
        }

        /// <summary>
        /// Load all routes defined in the config.
        /// </summary>
        public void LoadRoutes(AIConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));

            // Reset existing routes before reloading from disk.
            SpawnRoute = new List<Util.Waypoint>();
            MainRoute = new List<Util.Waypoint>();
            branches.Clear();

            waypointManager.LoadTrafficRoute(config.SpawnRouteName);
            SpawnRoute = waypointManager.GetTrafficRoute(config.SpawnRouteName);
            var spawnMeta = waypointManager.GetRecordedRoute(config.SpawnRouteName)?.Metadata;
            if (spawnMeta != null && spawnMeta.Type == RouteType.Unknown) spawnMeta.Type = RouteType.PitEntry;
            logger.Log(
                $"Loaded spawn route {config.SpawnRouteName} ({spawnMeta?.Type ?? RouteType.PitEntry}) with {SpawnRoute.Count} points");

            waypointManager.LoadTrafficRoute(config.MainRouteName);
            MainRoute = waypointManager.GetTrafficRoute(config.MainRouteName);
            var mainMeta = waypointManager.GetRecordedRoute(config.MainRouteName)?.Metadata;
            if (mainMeta != null && mainMeta.Type == RouteType.Unknown) mainMeta.Type = RouteType.MainLoop;
            logger.Log(
                $"Loaded main route {config.MainRouteName} ({mainMeta?.Type ?? RouteType.MainLoop}) with {MainRoute.Count} points");

            var branchNames = BuildBranchList(config);
            config.BranchRouteNames = branchNames;

            foreach (var name in branchNames)
            {
                waypointManager.LoadTrafficRoute(name);
                var path = waypointManager.GetTrafficRoute(name);
                if (path.Count == 0) continue;
                var metadata = waypointManager.GetRecordedRoute(name)?.Metadata;
                var startIndex = metadata?.AttachMainIndex ?? FindNearestIndex(MainRoute, path[0]);
                var rejoinIndex = metadata?.RejoinMainIndex ?? FindNearestIndex(MainRoute, path[path.Count - 1]);
                branches[name] = new BranchRouteInfo
                {
                    Name = name,
                    Path = path,
                    StartIndex = startIndex,
                    RejoinIndex = rejoinIndex,
                    Metadata = metadata
                };
                logger.Log($"Loaded branch {name} ({metadata?.Type ?? RouteType.Detour}) with {path.Count} points " +
                           $"starting near index {startIndex} and rejoining near {rejoinIndex}");
            }
        }

        /// <summary>
        /// Find the nearest waypoint index on a path to the given waypoint.
        /// </summary>
        private static int FindNearestIndex(List<Util.Waypoint> path, Util.Waypoint point)
        {
            var minDist = double.MaxValue;
            var best = 0;
            for (var i = 0; i < path.Count; i++)
            {
                var dx = (path[i].Position.X - point.Position.X) / 65536.0;
                var dy = (path[i].Position.Y - point.Position.Y) / 65536.0;
                var dist = Math.Sqrt(dx * dx + dy * dy);
                if (dist < minDist)
                {
                    minDist = dist;
                    best = i;
                }
            }
            return best;
        }

        /// <summary>
        /// Check if there is a branch starting at the given main route index.
        /// </summary>
        public bool TryGetBranch(int mainIndex, out BranchRouteInfo branchInfo)
        {
            foreach (var b in branches.Values)
            {
                if (b.StartIndex == mainIndex)
                {
                    branchInfo = b;
                    return true;
                }
            }
            branchInfo = null;
            return false;
        }

        /// <summary>
        /// Find a branch by name for manual selection (case-insensitive).
        /// </summary>
        public bool TryGetBranchByName(string name, out BranchRouteInfo branchInfo)
        {
            if (string.IsNullOrWhiteSpace(name))
            {
                branchInfo = null;
                return false;
            }

            return branches.TryGetValue(name, out branchInfo);
        }

        /// <summary>
        /// Pick a random branch path from the available branches.
        /// </summary>
        public List<Util.Waypoint> GetRandomBranch()
        {
            if (branches.Count == 0) return new List<Util.Waypoint>();
            var values = branches.Values.ToList();
            var branch = values[random.Next(values.Count)];
            return branch.Path;
        }

        /// <summary>
        /// Build the list of detour branches from config and any recorded detour routes.
        /// </summary>
        private List<string> BuildBranchList(AIConfig config)
        {
            var names = new List<string>();
            if (config.BranchRouteNames != null)
            {
                foreach (var name in config.BranchRouteNames)
                {
                    AddUnique(names, name);
                }
            }

            try
            {
                var recorded = routeLibrary.ListRoutes();
                foreach (var route in recorded)
                {
                    var meta = route?.Metadata;
                    if (meta == null) continue;
                    if (string.IsNullOrWhiteSpace(meta.Name)) continue;
                    if (meta.Name.Equals(config.MainRouteName, StringComparison.OrdinalIgnoreCase)) continue;
                    if (meta.Name.Equals(config.SpawnRouteName, StringComparison.OrdinalIgnoreCase)) continue;

                    var type = meta.Type == RouteType.Unknown
                        ? routeLibrary.GuessRouteType(meta.Name)
                        : meta.Type;

                    if (type == RouteType.Unknown) type = RouteType.Detour;

                    if (type == RouteType.Detour)
                        AddUnique(names, meta.Name);
                }
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to discover recorded detour routes");
            }

            return names;
        }

        /// <summary>
        /// Add a string to a list if it is non-empty and not already present (case-insensitive).
        /// </summary>
        private static void AddUnique(List<string> list, string value)
        {
            if (string.IsNullOrWhiteSpace(value)) return;
            if (list.Exists(v => v.Equals(value, StringComparison.OrdinalIgnoreCase))) return;
            list.Add(value);
        }
    }
}
