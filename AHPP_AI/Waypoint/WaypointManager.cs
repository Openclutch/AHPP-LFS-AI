using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using AHPP_AI.AI;
using AHPP_AI.Debug;
using AHPP_AI.Util;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    /// Simplified waypoint manager that only supports recorded JSON routes.
    /// </summary>
    public class WaypointManager
    {
        private readonly Logger logger;
        private readonly Dictionary<string, List<Util.Waypoint>> trafficRoutes = new Dictionary<string, List<Util.Waypoint>>();
        private readonly Dictionary<string, string> routeTypes = new Dictionary<string, string>();

        public WaypointManager(Logger logger)
        {
            this.logger = logger ?? throw new ArgumentNullException(nameof(logger));
            logger.Log("WaypointManager initialized");
        }

        public List<Util.Waypoint> GetPathForCar(Vec spawnPos, AIConfig config)
        {
            var route = GetTrafficRoute(config.TrafficRouteName);
            if (route.Count > 0)
            {
                logger.Log($"Using recorded route {config.TrafficRouteName} with {route.Count} points");
                return route;
            }

            logger.LogWarning($"Recorded route {config.TrafficRouteName} not found, using fallback circle");
            var x = spawnPos.X / 65536.0;
            var y = spawnPos.Y / 65536.0;
            return CreateFallbackPath(x, y);
        }

        private List<Util.Waypoint> CreateFallbackPath(double spawnX, double spawnY)
        {
            logger.Log($"Creating fallback circular path around X={spawnX:F2}, Y={spawnY:F2}");

            var fallbackPath = new List<Util.Waypoint>();
            var radius = 20.0;
            var segments = 16;

            for (var i = 0; i < segments; i++)
            {
                var angle = 2 * Math.PI * i / segments;
                var x = spawnX + radius * Math.Cos(angle);
                var y = spawnY + radius * Math.Sin(angle);
                fallbackPath.Add(new Util.Waypoint(x, y, 30.0, (byte)i));
            }

            logger.Log($"Created fallback circular path with {fallbackPath.Count} waypoints");
            return fallbackPath;
        }

        public int FindNearestWaypoint(Vec position, List<Util.Waypoint> path)
        {
            if (path == null || path.Count == 0)
            {
                logger.LogError("Cannot find nearest waypoint: path is empty");
                return 0;
            }

            var posX = position.X / 65536.0;
            var posY = position.Y / 65536.0;

            var closestIndex = 0;
            var minDistance = double.MaxValue;

            for (var i = 0; i < path.Count; i++)
            {
                var wpX = path[i].Position.X / 65536.0;
                var wpY = path[i].Position.Y / 65536.0;
                var dx = wpX - posX;
                var dy = wpY - posY;
                var distance = Math.Sqrt(dx * dx + dy * dy);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestIndex = i;
                }
            }

            return closestIndex;
        }

        public (int waypointIndex, bool clockwise) FindBestWaypointForDirection(Vec position, int heading, List<Util.Waypoint> path)
        {
            if (path == null || path.Count < 2)
            {
                logger.LogError("Cannot find best waypoint: path too short");
                return (0, true);
            }

            var closestIndex = FindNearestWaypoint(position, path);
            return (closestIndex, true);
        }

        public void LoadTrafficRoute(string name)
        {
            var file = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, $"{name}.json");
            if (!File.Exists(file))
            {
                logger.LogWarning($"Route file not found: {file}. Creating empty file.");
                File.WriteAllText(file, "[]");
            }

            try
            {
                var json = File.ReadAllText(file);
                List<RoutePoint> points = null;
                try
                {
                    var saved = JsonSerializer.Deserialize<SavedRoute>(json);
                    if (saved != null && saved.Nodes != null)
                    {
                        points = saved.Nodes;
                        routeTypes[name] = saved.Type;
                    }
                }
                catch { }

                if (points == null)
                {
                    points = JsonSerializer.Deserialize<List<RoutePoint>>(json);
                    routeTypes[name] = "main";
                }

                if (points == null)
                {
                    logger.LogError($"Failed to parse route file {file}");
                    return;
                }

                var list = new List<Util.Waypoint>();
                byte idx = 0;
                foreach (var p in points)
                    list.Add(new Util.Waypoint(p.X, p.Y, p.Speed, idx++));

                trafficRoutes[name] = list;
                logger.Log($"Loaded traffic route {name} with {list.Count} points");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Error loading route {name}");
            }
        }

        public List<Util.Waypoint> GetTrafficRoute(string name)
        {
            if (trafficRoutes.TryGetValue(name, out var list))
                return list;
            return new List<Util.Waypoint>();
        }

        public string GetRouteType(string name)
        {
            return routeTypes.TryGetValue(name, out var t) ? t : "main";
        }
    }
}
