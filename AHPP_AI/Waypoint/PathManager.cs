using System;
using System.Collections.Generic;
using System.Linq;
using AHPP_AI.AI;
using AHPP_AI.Debug;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    /// Maintains the different traffic routes for AI including spawn, main loop and optional branches.
    /// </summary>
    public class PathManager
    {
        private readonly WaypointManager waypointManager;
        private readonly Logger logger;
        private readonly Random random = new Random();

        public List<Util.Waypoint> SpawnRoute { get; private set; } = new List<Util.Waypoint>();
        public List<Util.Waypoint> MainRoute { get; private set; } = new List<Util.Waypoint>();

        private readonly Dictionary<string, BranchRoute> branches = new Dictionary<string, BranchRoute>();

        private class BranchRoute
        {
            public string Name { get; set; }
            public List<Util.Waypoint> Path { get; set; }
            public int StartIndex { get; set; }
        }

        public PathManager(WaypointManager waypointManager, Logger logger)
        {
            this.waypointManager = waypointManager;
            this.logger = logger;
        }

        /// <summary>
        /// Load all routes defined in the config.
        /// </summary>
        public void LoadRoutes(AIConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));

            waypointManager.LoadTrafficRoute(config.SpawnRouteName);
            SpawnRoute = waypointManager.GetTrafficRoute(config.SpawnRouteName);
            logger.Log($"Loaded spawn route {config.SpawnRouteName} with {SpawnRoute.Count} points");

            waypointManager.LoadTrafficRoute(config.MainRouteName);
            MainRoute = waypointManager.GetTrafficRoute(config.MainRouteName);
            logger.Log($"Loaded main route {config.MainRouteName} with {MainRoute.Count} points");

            foreach (var name in config.BranchRouteNames)
            {
                waypointManager.LoadTrafficRoute(name);
                var path = waypointManager.GetTrafficRoute(name);
                if (path.Count == 0) continue;
                var startIndex = FindNearestIndex(MainRoute, path[0]);
                branches[name] = new BranchRoute { Name = name, Path = path, StartIndex = startIndex };
                logger.Log($"Loaded branch {name} with {path.Count} points starting near index {startIndex}");
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
        public bool TryGetBranch(int mainIndex, out List<Util.Waypoint> branchPath)
        {
            foreach (var b in branches.Values)
            {
                if (b.StartIndex == mainIndex)
                {
                    branchPath = b.Path;
                    return true;
                }
            }
            branchPath = null;
            return false;
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
    }
}
