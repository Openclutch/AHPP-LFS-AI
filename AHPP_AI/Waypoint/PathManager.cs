using System;
using System.Collections.Generic;
using System.Linq;
using AHPP_AI.AI;
using AHPP_AI.Debug;
using AHPP_AI.Util;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    /// Describes a detour branch with metadata and start/end indices on the main route.
    /// </summary>
    public class BranchRouteInfo
    {
        public string Name { get; set; } = string.Empty;
        public List<Util.Waypoint> Path { get; set; } = new List<Util.Waypoint>();
        public int StartIndex { get; set; }
        public int RejoinIndex { get; set; }
        public RouteMetadata Metadata { get; set; } = new RouteMetadata();
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
        private readonly Dictionary<string, List<Util.Waypoint>> spawnRoutes =
            new Dictionary<string, List<Util.Waypoint>>(StringComparer.OrdinalIgnoreCase);
        private readonly Dictionary<string, bool> spawnRouteLoops =
            new Dictionary<string, bool>(StringComparer.OrdinalIgnoreCase);

        public List<Util.Waypoint> SpawnRoute { get; private set; } = new List<Util.Waypoint>();
        public List<Util.Waypoint> MainRoute { get; private set; } = new List<Util.Waypoint>();
        public BranchRouteInfo? MainAlternateRoute { get; private set; }
        public IReadOnlyList<RouteValidationIssue> ValidationIssues => validationIssues;

        private readonly Dictionary<string, BranchRouteInfo> branches =
            new Dictionary<string, BranchRouteInfo>(StringComparer.OrdinalIgnoreCase);

        public RouteMetadata SpawnRouteMetadata { get; private set; } = new RouteMetadata();
        public RouteMetadata MainRouteMetadata { get; private set; } = new RouteMetadata();
        public IReadOnlyDictionary<string, List<Util.Waypoint>> SpawnRoutes => spawnRoutes;

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
            MainAlternateRoute = null;
            branches.Clear();
            validationIssues.Clear();
            spawnRoutes.Clear();
            spawnRouteLoops.Clear();

            waypointManager.LoadTrafficRoute(config.SpawnRouteName);
            SpawnRoute = waypointManager.GetTrafficRoute(config.SpawnRouteName);
            var spawnMetaCandidate = waypointManager.GetRecordedRoute(config.SpawnRouteName)?.Metadata;
            if (spawnMetaCandidate != null && spawnMetaCandidate.Type == RouteType.Unknown) spawnMetaCandidate.Type = RouteType.PitEntry;
            if (spawnMetaCandidate != null && (!spawnMetaCandidate.AiWeight.HasValue || spawnMetaCandidate.AiWeight < 0))
                spawnMetaCandidate.AiWeight = 0;
            RouteMetadata spawnMeta = spawnMetaCandidate ?? new RouteMetadata
            {
                Name = config.SpawnRouteName,
                Type = RouteType.PitEntry,
                AiWeight = 0,
                AiEnabled = false
            };
            SpawnRouteMetadata = spawnMeta;
            logger.Log(
                $"Loaded spawn route {config.SpawnRouteName} ({spawnMeta.Type}) with {SpawnRoute.Count} points");
            spawnRoutes[config.SpawnRouteName] = SpawnRoute;
            spawnRouteLoops[config.SpawnRouteName] = spawnMeta?.IsLoop ?? false;

            DiscoverAdditionalSpawnRoutes(config);

            waypointManager.LoadTrafficRoute(config.MainRouteName);
            MainRoute = waypointManager.GetTrafficRoute(config.MainRouteName);
            var mainMetaCandidate = waypointManager.GetRecordedRoute(config.MainRouteName)?.Metadata;
            if (mainMetaCandidate != null && mainMetaCandidate.Type == RouteType.Unknown) mainMetaCandidate.Type = RouteType.MainLoop;
            if (mainMetaCandidate != null && (!mainMetaCandidate.AiWeight.HasValue || mainMetaCandidate.AiWeight <= 0))
                mainMetaCandidate.AiWeight = 1.0;
            RouteMetadata mainMeta = mainMetaCandidate ?? new RouteMetadata
            {
                Name = config.MainRouteName,
                Type = RouteType.MainLoop,
                AiWeight = 1.0,
                AiEnabled = true
            };
            MainRouteMetadata = mainMeta;
            logger.Log(
                $"Loaded main route {config.MainRouteName} ({mainMeta.Type}) with {MainRoute.Count} points");

            // Load an optional alternate main lane (e.g., inner highway lane)
            if (!string.IsNullOrWhiteSpace(config.MainAlternateRouteName))
            {
                waypointManager.LoadTrafficRoute(config.MainAlternateRouteName);
                var altPath = waypointManager.GetTrafficRoute(config.MainAlternateRouteName);
                var altMeta = waypointManager.GetRecordedRoute(config.MainAlternateRouteName)?.Metadata;
                if (altMeta != null && altMeta.Type == RouteType.Unknown) altMeta.Type = RouteType.AlternateMain;
                if (altMeta == null)
                {
                    altMeta = new RouteMetadata
                    {
                        Name = config.MainAlternateRouteName,
                        Type = RouteType.AlternateMain,
                        AiWeight = 1.0,
                        AiEnabled = true
                    };
                }
                else if (!altMeta.AiWeight.HasValue || altMeta.AiWeight <= 0)
                {
                    altMeta.AiWeight = 1.0;
                }

                if (altPath.Count == 0)
                {
                    AddValidationIssue(RouteValidationSeverity.Warning,
                        $"Alternate main route {config.MainAlternateRouteName} has no points and will be skipped.");
                }
                else
                {
                    var nearestStart = FindNearestIndex(MainRoute, altPath[0]);
                    var nearestRejoin = FindNearestIndex(MainRoute, altPath[altPath.Count - 1]);
                    var startIndex = altMeta?.AttachMainIndex ?? nearestStart;
                    var rejoinIndex = altMeta?.RejoinMainIndex ?? nearestRejoin;
                    startIndex = ValidateBranchIndex(config.MainAlternateRouteName, "attach", startIndex, nearestStart,
                        MainRoute.Count);
                    rejoinIndex = ValidateBranchIndex(config.MainAlternateRouteName, "rejoin", rejoinIndex,
                        nearestRejoin, MainRoute.Count);

                    MainAlternateRoute = new BranchRouteInfo
                    {
                        Name = config.MainAlternateRouteName,
                        Path = altPath,
                        StartIndex = startIndex,
                        RejoinIndex = rejoinIndex,
                        Metadata = altMeta ?? new RouteMetadata
                        {
                            Name = config.MainAlternateRouteName,
                            Type = RouteType.AlternateMain
                        }
                    };

                    branches[config.MainAlternateRouteName] = MainAlternateRoute;

                    logger.Log(
                        $"Loaded alternate main route {config.MainAlternateRouteName} ({MainAlternateRoute.Metadata?.Type ?? RouteType.AlternateMain}) with {altPath.Count} points starting near index {startIndex} and rejoining near {rejoinIndex}");
                }
            }

            var branchNames = BuildBranchList(config);
            config.BranchRouteNames = branchNames;

            foreach (var name in branchNames)
            {
                if (MainAlternateRoute != null &&
                    name.Equals(MainAlternateRoute.Name, StringComparison.OrdinalIgnoreCase))
                    continue;

                waypointManager.LoadTrafficRoute(name);
                var path = waypointManager.GetTrafficRoute(name);
                if (path.Count == 0)
                {
                    AddValidationIssue(RouteValidationSeverity.Warning,
                        $"Branch route {name} has no points and will be skipped.");
                    continue;
                }
                RouteMetadata metadata = waypointManager.GetRecordedRoute(name)?.Metadata ?? new RouteMetadata
                {
                    Name = name,
                    Type = RouteType.Detour,
                    AiWeight = 1.0,
                    AiEnabled = true
                };
                if (!metadata.AiWeight.HasValue || metadata.AiWeight <= 0)
                {
                    metadata.AiWeight = 1.0;
                }
                var nearestStart = FindNearestIndex(MainRoute, path[0]);
                var nearestRejoin = FindNearestIndex(MainRoute, path[path.Count - 1]);
                var startIndex = metadata.AttachMainIndex ?? nearestStart;
                var rejoinIndex = metadata.RejoinMainIndex ?? nearestRejoin;
                startIndex = ValidateBranchIndex(name, "attach", startIndex, nearestStart, MainRoute.Count);
                rejoinIndex = ValidateBranchIndex(name, "rejoin", rejoinIndex, nearestRejoin, MainRoute.Count);
                var branch = new BranchRouteInfo
                {
                    Name = name,
                    Path = path,
                    StartIndex = startIndex,
                    RejoinIndex = rejoinIndex,
                    Metadata = metadata
                };
                branches[name] = branch;
                if ((metadata.Type == RouteType.AlternateMain || name.Equals(config.MainAlternateRouteName, StringComparison.OrdinalIgnoreCase)) &&
                    MainAlternateRoute == null)
                    MainAlternateRoute = branch;
                logger.Log($"Loaded branch {name} ({metadata.Type}) with {path.Count} points " +
                           $"starting near index {startIndex} and rejoining near {rejoinIndex}");
            }

            ValidateRoutes(config, spawnMeta, mainMeta);
        }

        /// <summary>
        /// Add any recorded pit/spawn routes beyond the configured default so multi-lane spawns can be chosen dynamically.
        /// </summary>
        private void DiscoverAdditionalSpawnRoutes(AIConfig config)
        {
            try
            {
                var recorded = routeLibrary.ListRoutes(out var duplicates);
                foreach (var duplicate in duplicates)
                {
                    AddValidationIssue(RouteValidationSeverity.Warning,
                        $"Duplicate recorded route name \"{duplicate}\" detected. Only the first instance is used.");
                }

                foreach (var route in recorded)
                {
                    if (route?.Metadata == null) continue;
                    var name = route.Metadata.Name;
                    if (string.IsNullOrWhiteSpace(name)) continue;
                    if (spawnRoutes.ContainsKey(name)) continue;
                    var type = route.Metadata.Type == RouteType.Unknown
                        ? routeLibrary.GuessRouteType(name)
                        : route.Metadata.Type;
                    if (type != RouteType.PitEntry)
                    {
                        // Allow routes that look like spawn/pit by name even if metadata is missing.
                        if (!(name.Contains("pit", StringComparison.OrdinalIgnoreCase) ||
                              name.Contains("spawn", StringComparison.OrdinalIgnoreCase)))
                            continue;
                    }

                    waypointManager.LoadTrafficRoute(name);
                    var path = waypointManager.GetTrafficRoute(name);
                    if (path.Count == 0)
                    {
                        AddValidationIssue(RouteValidationSeverity.Warning,
                            $"Spawn route candidate {name} has no points and will be skipped.");
                        continue;
                    }

                    spawnRoutes[name] = path;
                    spawnRouteLoops[name] = route.Metadata?.IsLoop ?? false;
                    logger.Log($"Discovered additional spawn route {name} with {path.Count} points");
                }
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to discover additional spawn routes");
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
        public bool TryGetBranch(int mainIndex, out BranchRouteInfo? branchInfo)
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
        /// Get the configured alternate main route (e.g., right-lane highway path) if available.
        /// </summary>
        public BranchRouteInfo? GetAlternateMainRoute()
        {
            return MainAlternateRoute;
        }

        /// <summary>
        /// Find a branch by name for manual selection (case-insensitive).
        /// </summary>
        public bool TryGetBranchByName(string name, out BranchRouteInfo? branchInfo)
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

            if (config?.BranchRouteNames != null)
            {
                foreach (var configured in config.BranchRouteNames)
                {
                    TryAddUnique(names, configured, "config");
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

                    if (type == RouteType.Detour || type == RouteType.AlternateMain)
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
        /// Choose the best-fit spawn route for a given position/heading among all recorded pit-entry routes.
        /// Returns the path, start index and direction that should be used.
        /// </summary>
        public bool TrySelectSpawnRoute(Vec position, int heading, out string routeName, out List<Util.Waypoint> path,
            out int startIndex, out bool clockwise, out bool isLoop)
        {
            routeName = SpawnRouteMetadata?.Name ?? string.Empty;
            path = SpawnRoute;
            startIndex = 0;
            clockwise = true;
            isLoop = SpawnRouteMetadata?.IsLoop ?? false;

            if (spawnRoutes.Count == 0)
                return path != null && path.Count > 0;

            var bestScore = double.MaxValue;
            var found = false;

            foreach (var kvp in spawnRoutes)
            {
                var candidatePath = kvp.Value;
                if (candidatePath == null || candidatePath.Count < 2) continue;

                var loop = spawnRouteLoops.TryGetValue(kvp.Key, out var recordedLoop) && recordedLoop;
                var (score, index, forward) = waypointManager.ScoreRouteFit(position, heading, candidatePath, loop, false);
                if (score < bestScore)
                {
                    bestScore = score;
                    routeName = kvp.Key;
                    path = candidatePath;
                    startIndex = index;
                    clockwise = forward;
                    isLoop = loop;
                    found = true;
                }
            }

            return found;
        }

        /// <summary>
        /// Validate loaded routes so the user can be notified of problems.
        /// </summary>
        private void ValidateRoutes(AIConfig config, RouteMetadata spawnMeta, RouteMetadata mainMeta)
        {
            ValidateRouteBasics(config.SpawnRouteName, SpawnRoute, spawnMeta, "spawn");
            ValidateRouteBasics(config.MainRouteName, MainRoute, mainMeta, "main");
            ValidateMainLoop(config.MainRouteName, mainMeta);

            if (MainAlternateRoute != null)
            {
                ValidateRouteBasics(MainAlternateRoute.Name, MainAlternateRoute.Path, MainAlternateRoute.Metadata,
                    "alternate main");
                ValidateBranchIndices(MainAlternateRoute);
            }

            foreach (var branch in branches.Values)
            {
                if (MainAlternateRoute != null &&
                    branch.Name.Equals(MainAlternateRoute.Name, StringComparison.OrdinalIgnoreCase))
                    continue;

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
        /// Expose all loaded branch routes for consumers that need to inspect them.
        /// </summary>
        public IEnumerable<BranchRouteInfo> GetBranches()
        {
            return branches.Values;
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
        private void ValidateBranchIndices(BranchRouteInfo? branch)
        {
            if (branch == null || branch.Path == null || branch.Path.Count == 0) return;
            var branchName = branch.Name;
            if (MainRoute == null || MainRoute.Count == 0)
            {
                AddValidationIssue(RouteValidationSeverity.Error,
                    $"Branch {branchName} cannot attach because the main route is empty.");
                branch.StartIndex = 0;
                branch.RejoinIndex = 0;
                return;
            }

            var fallbackAttach = FindNearestIndex(MainRoute, branch.Path[0]);
            var fallbackRejoin = FindNearestIndex(MainRoute, branch.Path[branch.Path.Count - 1]);
            branch.StartIndex = ValidateBranchIndex(branch.Name, "attach", branch.StartIndex, fallbackAttach, MainRoute.Count);
            branch.RejoinIndex = ValidateBranchIndex(branch.Name, "rejoin", branch.RejoinIndex, fallbackRejoin, MainRoute.Count);

            // For looped/alternate lanes the attach and rejoin can legitimately coincide; only warn otherwise.
            var allowsSameIndex = (branch.Metadata?.IsLoop ?? false) ||
                                  branch.Metadata?.Type == RouteType.AlternateMain;

            if (branch.StartIndex == branch.RejoinIndex && !allowsSameIndex)
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
