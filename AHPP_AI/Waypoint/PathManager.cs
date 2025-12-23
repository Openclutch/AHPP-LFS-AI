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
        private readonly List<RouteValidationIssue> validationIssues = new List<RouteValidationIssue>();

        public List<Util.Waypoint> SpawnRoute { get; private set; } = new List<Util.Waypoint>();
        public List<Util.Waypoint> MainRoute { get; private set; } = new List<Util.Waypoint>();
        public IReadOnlyList<RouteValidationIssue> ValidationIssues => validationIssues;

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
            validationIssues.Clear();

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
                if (path.Count == 0)
                {
                    AddValidationIssue(RouteValidationSeverity.Warning,
                        $"Branch route {name} has no points and will be skipped.");
                    continue;
                }
                var metadata = waypointManager.GetRecordedRoute(name)?.Metadata;
                var nearestStart = FindNearestIndex(MainRoute, path[0]);
                var nearestRejoin = FindNearestIndex(MainRoute, path[path.Count - 1]);
                var startIndex = metadata?.AttachMainIndex ?? nearestStart;
                var rejoinIndex = metadata?.RejoinMainIndex ?? nearestRejoin;
                startIndex = ValidateBranchIndex(name, "attach", startIndex, nearestStart, MainRoute.Count);
                rejoinIndex = ValidateBranchIndex(name, "rejoin", rejoinIndex, nearestRejoin, MainRoute.Count);
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

            ValidateRoutes(config, spawnMeta, mainMeta);
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
                    TryAddUnique(names, name, "config");
                }
            }

            try
            {
                var recorded = routeLibrary.ListRoutes(out var duplicateNames);
                foreach (var duplicate in duplicateNames)
                {
                    AddValidationIssue(RouteValidationSeverity.Warning,
                        $"Duplicate recorded route name \"{duplicate}\" detected. Only the first instance is used.");
                }

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
                        TryAddUnique(names, meta.Name, "recorded file");
                }
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to discover recorded detour routes");
            }

            return names;
        }

        /// <summary>
        /// Validate loaded routes so the user can be notified of problems.
        /// </summary>
        private void ValidateRoutes(AIConfig config, RouteMetadata spawnMeta, RouteMetadata mainMeta)
        {
            ValidateRouteBasics(config.SpawnRouteName, SpawnRoute, spawnMeta, "spawn");
            ValidateRouteBasics(config.MainRouteName, MainRoute, mainMeta, "main");
            ValidateMainLoop(config.MainRouteName, mainMeta);

            foreach (var branch in branches.Values)
            {
                ValidateRouteBasics(branch.Name, branch.Path, branch.Metadata, "branch");
                ValidateBranchIndices(branch);
            }
        }

        /// <summary>
        /// Ensure the main loop is flagged correctly and forms a closed path.
        /// </summary>
        private void ValidateMainLoop(string routeName, RouteMetadata metadata)
        {
            if (MainRoute == null || MainRoute.Count == 0) return;

            if (MainRoute.Count < 3)
            {
                AddValidationIssue(RouteValidationSeverity.Error,
                    $"Main route {routeName} has too few points to form a loop.");
                return;
            }

            if (metadata == null || !metadata.IsLoop)
            {
                AddValidationIssue(RouteValidationSeverity.Warning,
                    $"Main route {routeName} is not flagged as a loop.");
            }

            var distance = CalculateDistanceMeters(MainRoute[0], MainRoute[MainRoute.Count - 1]);
            if (distance > 5.0)
            {
                AddValidationIssue(RouteValidationSeverity.Warning,
                    $"Main route {routeName} start/end are {distance:F1}m apart; route may not be closed.");
            }
        }

        /// <summary>
        /// Validate a single route for required metadata and points.
        /// </summary>
        private void ValidateRouteBasics(string routeName, List<Util.Waypoint> path, RouteMetadata metadata, string role)
        {
            if (string.IsNullOrWhiteSpace(routeName))
            {
                AddValidationIssue(RouteValidationSeverity.Warning, $"Route name for {role} route is missing.");
            }

            if (metadata == null)
            {
                AddValidationIssue(RouteValidationSeverity.Error,
                    $"Route {routeName} has no metadata; defaults will be used.");
            }
            else if (metadata.Type == RouteType.Unknown)
            {
                AddValidationIssue(RouteValidationSeverity.Warning,
                    $"Route {routeName} type is unknown; guessing at runtime.");
            }

            if (path == null || path.Count == 0)
            {
                var severity = role == "main" ? RouteValidationSeverity.Error : RouteValidationSeverity.Warning;
                AddValidationIssue(severity, $"Route {routeName} has no recorded points.");
            }
        }

        /// <summary>
        /// Validate and clamp branch indices to the bounds of the main route.
        /// </summary>
        private void ValidateBranchIndices(BranchRouteInfo branch)
        {
            if (branch?.Path == null || branch.Path.Count == 0) return;
            if (MainRoute == null || MainRoute.Count == 0)
            {
                AddValidationIssue(RouteValidationSeverity.Error,
                    $"Branch {branch?.Name} cannot attach because the main route is empty.");
                branch.StartIndex = 0;
                branch.RejoinIndex = 0;
                return;
            }

            var fallbackAttach = FindNearestIndex(MainRoute, branch.Path[0]);
            var fallbackRejoin = FindNearestIndex(MainRoute, branch.Path[branch.Path.Count - 1]);
            branch.StartIndex = ValidateBranchIndex(branch.Name, "attach", branch.StartIndex, fallbackAttach, MainRoute.Count);
            branch.RejoinIndex = ValidateBranchIndex(branch.Name, "rejoin", branch.RejoinIndex, fallbackRejoin, MainRoute.Count);

            if (branch.StartIndex == branch.RejoinIndex)
            {
                AddValidationIssue(RouteValidationSeverity.Warning,
                    $"Branch {branch.Name} attach and rejoin at the same main index {branch.StartIndex}.");
            }
        }

        /// <summary>
        /// Ensure a branch attach/rejoin index is valid, falling back to a safe value if needed.
        /// </summary>
        private int ValidateBranchIndex(string branchName, string label, int index, int fallbackIndex, int mainCount)
        {
            if (mainCount <= 0) return 0;

            if (index < 0 || index >= mainCount)
            {
                AddValidationIssue(RouteValidationSeverity.Warning,
                    $"Branch {branchName} {label} index {index} is outside 0-{mainCount - 1}; using {fallbackIndex} instead.");
                return fallbackIndex;
            }

            return index;
        }

        /// <summary>
        /// Record a validation issue and log it for diagnostics.
        /// </summary>
        private void AddValidationIssue(RouteValidationSeverity severity, string message)
        {
            validationIssues.Add(new RouteValidationIssue(severity, message));
            if (severity == RouteValidationSeverity.Error) logger.LogError(message);
            else logger.LogWarning(message);
        }

        /// <summary>
        /// Calculate 2D distance between two waypoints in meters.
        /// </summary>
        private static double CalculateDistanceMeters(Util.Waypoint a, Util.Waypoint b)
        {
            var dx = (a.Position.X - b.Position.X) / 65536.0;
            var dy = (a.Position.Y - b.Position.Y) / 65536.0;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// Add a string to a list if it is non-empty and not already present (case-insensitive).
        /// </summary>
        private bool TryAddUnique(List<string> list, string value, string source)
        {
            if (string.IsNullOrWhiteSpace(value)) return false;
            if (list.Exists(v => v.Equals(value, StringComparison.OrdinalIgnoreCase)))
            {
                AddValidationIssue(RouteValidationSeverity.Warning,
                    $"Duplicate route name \"{value}\" from {source} ignored.");
                return false;
            }

            list.Add(value);
            return true;
        }
    }
}
