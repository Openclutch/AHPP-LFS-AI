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
        private readonly string basePath;

        public RouteLibrary(Logger logger)
        {
            this.logger = logger ?? throw new ArgumentNullException(nameof(logger));
            basePath = InitializeBasePath();
            serializerOptions = new JsonSerializerOptions
            {
                WriteIndented = true,
                PropertyNameCaseInsensitive = true
            };
            serializerOptions.Converters.Add(new JsonStringEnumConverter());
        }

        /// <summary>
        /// Determine and create the folder used to store route files.
        /// Prefers a Routes subdirectory beside the executable for easy sharing.
        /// </summary>
        private string InitializeBasePath()
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
        /// Load a recorded route from disk, creating a template file if none exists.
        /// </summary>
        public RecordedRoute Load(string name)
        {
            var file = GetRoutePath(name);
            if (!File.Exists(file))
            {
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
                Name = string.IsNullOrWhiteSpace(name) ? "route" : name,
                Type = type == RouteType.Unknown ? GuessRouteType(name) : type,
                IsLoop = type == RouteType.MainLoop,
                DefaultSpeedLimit = 60
            };
        }

        private string GetRoutePath(string name)
        {
            var safeName = string.IsNullOrWhiteSpace(name) ? "route" : name;
            return Path.Combine(basePath, $"{safeName}.json");
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
    }
}
