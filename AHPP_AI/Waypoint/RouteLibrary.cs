using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;
using AHPP_AI.Debug;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    /// Handles loading and saving recorded routes with metadata on disk.
    /// </summary>
    public class RouteLibrary
    {
        private readonly Logger logger;
        private readonly JsonSerializerOptions serializerOptions;
        private readonly string routesRoot;
        private string trackCode = "UnknownTrack";
        private string layoutName = "DefaultLayout";

        public string TrackCode => trackCode;
        public string LayoutName => layoutName;

        public RouteLibrary(Logger logger)
        {
            this.logger = logger ?? throw new ArgumentNullException(nameof(logger));
            routesRoot = InitializeRoutesRoot();
            serializerOptions = new JsonSerializerOptions
            {
                WriteIndented = true,
                PropertyNameCaseInsensitive = true
            };
            serializerOptions.Converters.Add(new JsonStringEnumConverter());
        }

        /// <summary>
        /// Determine and create the root folder used to store route files.
        /// Prefers a Routes subdirectory beside the executable for easy sharing.
        /// </summary>
        private string InitializeRoutesRoot()
        {
            var routesPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Routes");
            try
            {
                Directory.CreateDirectory(routesPath);
                return routesPath;
            }
            catch (Exception ex)
            {
                logger.LogWarning($"Could not create Routes folder at {routesPath}: {ex.Message}");
                return AppDomain.CurrentDomain.BaseDirectory;
            }
        }

        /// <summary>
        /// Update the active track and layout context so routes are stored per combo.
        /// </summary>
        public void SetTrackLayout(string track, string layout)
        {
            var safeTrack = string.IsNullOrWhiteSpace(track) ? "UnknownTrack" : SanitizeName(track);
            var safeLayout = string.IsNullOrWhiteSpace(layout) ? "DefaultLayout" : SanitizeName(layout);

            if (safeTrack.Equals(trackCode, StringComparison.OrdinalIgnoreCase) &&
                safeLayout.Equals(layoutName, StringComparison.OrdinalIgnoreCase))
                return;

            trackCode = safeTrack;
            layoutName = safeLayout;

            var contextPath = GetContextPath();
            try
            {
                Directory.CreateDirectory(contextPath);
                logger.Log($"Route library context set to {trackCode}/{layoutName}");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Failed to create route directory {contextPath}");
            }
        }

        /// <summary>
        /// Load a recorded route from disk, creating a template file if none exists.
        /// </summary>
        public RecordedRoute Load(string name)
        {
            var contextPath = GetContextPath();
            EnsureContextPath(contextPath);
            var file = GetRoutePath(name);
            if (!File.Exists(file))
            {
                var legacy = GetLegacyRoutePath(name);
                if (!string.IsNullOrWhiteSpace(legacy) && File.Exists(legacy))
                {
                    try
                    {
                        var json = File.ReadAllText(legacy);
                        var migrated = NormalizeRoute(ParseRoute(json, name), name);
                        Save(migrated, file);
                        logger.Log($"Migrated legacy route {migrated.Metadata.Name} into {GetContextPath()}");
                        return migrated;
                    }
                    catch (Exception ex)
                    {
                        logger.LogException(ex, $"Failed to migrate legacy route {legacy}");
                    }
                }

                var template = CreateTemplate(name);
                Save(template, file);
                logger.Log($"Created new route template at {file}");
                return template;
            }

            try
            {
                var json = File.ReadAllText(file);
                var route = ParseRoute(json, name);
                return NormalizeRoute(route, name);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Failed to load route {name}");
                return CreateTemplate(name);
            }
        }

        /// <summary>
        /// Save a recorded route to disk using its metadata name.
        /// </summary>
        public void Save(RecordedRoute route)
        {
            var file = GetRoutePath(route?.Metadata?.Name ?? "route");
            Save(route, file);
        }

        /// <summary>
        /// Convert a recorded route into waypoints for AI or visualization.
        /// </summary>
        public List<Util.Waypoint> ConvertToWaypoints(RecordedRoute route)
        {
            return route == null ? new List<Util.Waypoint>() : route.ToWaypoints();
        }

        /// <summary>
        /// Guess a route type based on its name.
        /// </summary>
        public RouteType GuessRouteType(string name)
        {
            var lower = (name ?? string.Empty).ToLowerInvariant();
            if (lower.Contains("main")) return RouteType.MainLoop;
            if (lower.Contains("pit") || lower.Contains("spawn")) return RouteType.PitEntry;
            if (lower.Contains("detour") || lower.Contains("branch")) return RouteType.Detour;
            return RouteType.Unknown;
        }

        private void Save(RecordedRoute route, string file)
        {
            if (route == null) throw new ArgumentNullException(nameof(route));
            var normalized = NormalizeRoute(route, Path.GetFileNameWithoutExtension(file));
            var directory = Path.GetDirectoryName(file);
            if (!string.IsNullOrWhiteSpace(directory)) EnsureContextPath(directory);
            File.WriteAllText(file, JsonSerializer.Serialize(normalized, serializerOptions));
            logger.Log($"Saved route {normalized.Metadata.Name} with {normalized.Nodes.Count} points to {file}");
        }

        private RecordedRoute ParseRoute(string json, string name)
        {
            try
            {
                var route = JsonSerializer.Deserialize<RecordedRoute>(json, serializerOptions);
                if (route != null) return route;
            }
            catch
            {
                // Continue trying legacy formats
            }

            try
            {
                var saved = JsonSerializer.Deserialize<SavedRoute>(json, serializerOptions);
                if (saved != null && saved.Nodes != null) return ConvertLegacy(saved, name);
            }
            catch
            {
                // Continue trying legacy formats
            }

            try
            {
                var points = JsonSerializer.Deserialize<List<RoutePoint>>(json, serializerOptions);
                if (points != null) return new RecordedRoute
                {
                    Metadata = BuildMetadata(name, GuessRouteType(name)),
                    Nodes = points
                };
            }
            catch
            {
                // Continue to template fallback
            }

            return CreateTemplate(name);
        }

        private RecordedRoute ConvertLegacy(SavedRoute saved, string fallbackName)
        {
            var type = ParseTypeString(saved.Type);
            var metadata = BuildMetadata(saved.Name ?? fallbackName, type);
            return new RecordedRoute
            {
                Metadata = metadata,
                Nodes = saved.Nodes ?? new List<RoutePoint>()
            };
        }

        private RouteType ParseTypeString(string type)
        {
            if (string.IsNullOrWhiteSpace(type)) return RouteType.Unknown;
            switch (type.ToLowerInvariant())
            {
                case "main":
                case "loop":
                    return RouteType.MainLoop;
                case "pit":
                case "spawn":
                    return RouteType.PitEntry;
                case "branch":
                case "detour":
                    return RouteType.Detour;
                default:
                    return RouteType.Unknown;
            }
        }

        private RecordedRoute NormalizeRoute(RecordedRoute route, string fallbackName)
        {
            if (route == null) return CreateTemplate(fallbackName);

            if (route.Metadata == null) route.Metadata = BuildMetadata(fallbackName, GuessRouteType(fallbackName));
            if (string.IsNullOrWhiteSpace(route.Metadata.Name))
                route.Metadata.Name = string.IsNullOrWhiteSpace(fallbackName) ? "route" : fallbackName;

            route.Metadata.Name = NormalizeRouteName(route.Metadata.Name);

            route.Metadata.Track = trackCode;
            route.Metadata.Layout = layoutName;

            if (route.Metadata.Type == RouteType.Unknown)
                route.Metadata.Type = GuessRouteType(route.Metadata.Name);

            route.Metadata.IsLoop = route.Metadata.Type == RouteType.MainLoop || route.Metadata.IsLoop;

            if (route.Nodes == null) route.Nodes = new List<RoutePoint>();

            foreach (var node in route.Nodes)
            {
                if (!node.SpeedLimit.HasValue)
                {
                    // Prefer explicitly recorded speed if present, otherwise fall back to route default.
                    if (node.Speed > 0)
                        node.SpeedLimit = node.Speed;
                    else
                        node.SpeedLimit = route.Metadata.DefaultSpeedLimit ?? 0;
                }
            }

            return route;
        }

        private RouteMetadata BuildMetadata(string name, RouteType type)
        {
            return new RouteMetadata
            {
                Name = NormalizeRouteName(string.IsNullOrWhiteSpace(name) ? "route" : name),
                Type = type == RouteType.Unknown ? GuessRouteType(name) : type,
                IsLoop = type == RouteType.MainLoop,
                DefaultSpeedLimit = 60,
                Track = trackCode,
                Layout = layoutName
            };
        }

        private string GetRoutePath(string name)
        {
            var safeName = NormalizeRouteName(name);
            return Path.Combine(GetContextPath(), $"{safeName}.json");
        }

        private string GetLegacyRoutePath(string name)
        {
            var safeName = NormalizeRouteName(name);
            return Path.Combine(routesRoot, $"{safeName}.json");
        }

        private RecordedRoute CreateTemplate(string name)
        {
            var metadata = BuildMetadata(name, GuessRouteType(name));
            metadata.Description = "Template route. Replace nodes with recorded points.";

            return new RecordedRoute
            {
                Metadata = metadata,
                Nodes = new List<RoutePoint>()
            };
        }

        private string GetContextPath()
        {
            return Path.Combine(routesRoot, trackCode, layoutName);
        }

        private static string SanitizeName(string name)
        {
            foreach (var invalid in Path.GetInvalidFileNameChars())
            {
                name = name.Replace(invalid, '_');
            }
            return name.Trim();
        }

        private void EnsureContextPath(string contextPath)
        {
            if (Directory.Exists(contextPath)) return;
            Directory.CreateDirectory(contextPath);
        }

        /// <summary>
        /// Normalize route names for safe storage while keeping the original intent.
        /// </summary>
        public string NormalizeRouteName(string name)
        {
            var cleaned = SanitizeName(string.IsNullOrWhiteSpace(name) ? "route" : name);
            return string.IsNullOrWhiteSpace(cleaned) ? "route" : cleaned;
        }

        /// <summary>
        /// Load and normalize all routes within a directory, avoiding duplicates by name.
        /// </summary>
        private void LoadRoutesFromDirectory(string directory, List<RecordedRoute> routes, HashSet<string> seenNames)
        {
            try
            {
                if (!Directory.Exists(directory)) return;

                foreach (var file in Directory.GetFiles(directory, "*.json", SearchOption.TopDirectoryOnly))
                {
                    try
                    {
                        var json = File.ReadAllText(file);
                        var name = Path.GetFileNameWithoutExtension(file);
                        if (seenNames != null && seenNames.Contains(name)) continue;

                        var route = NormalizeRoute(ParseRoute(json, name), name);
                        routes.Add(route);
                        seenNames?.Add(route.Metadata?.Name ?? name);
                    }
                    catch (Exception ex)
                    {
                        logger.LogException(ex, $"Failed to read route {file}");
                    }
                }
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Failed to list routes in {directory}");
            }
        }

        /// <summary>
        /// Enumerate recorded routes for the current track/layout context.
        /// </summary>
        public List<RecordedRoute> ListRoutes()
        {
            var contextPath = GetContextPath();
            EnsureContextPath(contextPath);
            var routes = new List<RecordedRoute>();
            var seenNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            LoadRoutesFromDirectory(contextPath, routes, seenNames);

            var legacyPath = routesRoot;
            if (!contextPath.Equals(legacyPath, StringComparison.OrdinalIgnoreCase))
            {
                LoadRoutesFromDirectory(legacyPath, routes, seenNames);
            }

            return routes;
        }
    }
}
