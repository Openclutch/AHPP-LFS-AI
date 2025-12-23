using System;
using System.Collections.Generic;
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
        private readonly RouteLibrary routeLibrary;
        private readonly Dictionary<string, List<Util.Waypoint>> trafficRoutes = new Dictionary<string, List<Util.Waypoint>>();
        private readonly Dictionary<string, RecordedRoute> recordedRoutes = new Dictionary<string, RecordedRoute>();

        public WaypointManager(Logger logger, RouteLibrary routeLibrary)
        {
            this.logger = logger ?? throw new ArgumentNullException(nameof(logger));
            this.routeLibrary = routeLibrary ?? throw new ArgumentNullException(nameof(routeLibrary));
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

        /// <summary>
        /// Find the waypoint that is both near the car and aligns with its current heading.
        /// Returns the index and whether the path should be used in its recorded (clockwise) order.
        /// </summary>
        public (int waypointIndex, bool clockwise) FindBestWaypointForDirection(Vec position, int heading, List<Util.Waypoint> path)
        {
            if (path == null || path.Count < 2)
            {
                logger.LogError("Cannot find best waypoint: path too short");
                return (0, true);
            }

            var posX = position.X / 65536.0;
            var posY = position.Y / 65536.0;
            var normalizedHeading = CoordinateUtils.NormalizeHeading(heading);

            var bestForwardScore = double.MaxValue;
            var bestForwardIndex = 0;
            var bestReverseScore = double.MaxValue;
            var bestReverseIndex = 0;

            for (var i = 0; i < path.Count; i++)
            {
                var wpX = path[i].Position.X / 65536.0;
                var wpY = path[i].Position.Y / 65536.0;
                var distance = Math.Sqrt(Math.Pow(wpX - posX, 2) + Math.Pow(wpY - posY, 2));

                // Forward direction uses the next waypoint; reverse uses the previous waypoint.
                var next = path[(i + 1) % path.Count];
                var prev = path[(i - 1 + path.Count) % path.Count];

                var forwardHeading = CoordinateUtils.CalculateHeadingToTarget(
                    next.Position.X / 65536.0 - wpX,
                    next.Position.Y / 65536.0 - wpY);
                var forwardError = Math.Abs(CoordinateUtils.CalculateHeadingError(normalizedHeading, forwardHeading));
                var forwardPenalty = forwardError * 360.0 / CoordinateUtils.FULL_CIRCLE / 45.0;
                var forwardScore = distance + forwardPenalty;

                if (forwardScore < bestForwardScore)
                {
                    bestForwardScore = forwardScore;
                    bestForwardIndex = i;
                }

                var reverseHeading = CoordinateUtils.CalculateHeadingToTarget(
                    prev.Position.X / 65536.0 - wpX,
                    prev.Position.Y / 65536.0 - wpY);
                var reverseError = Math.Abs(CoordinateUtils.CalculateHeadingError(normalizedHeading, reverseHeading));
                var reversePenalty = reverseError * 360.0 / CoordinateUtils.FULL_CIRCLE / 45.0;
                var reverseScore = distance + reversePenalty;

                if (reverseScore < bestReverseScore)
                {
                    bestReverseScore = reverseScore;
                    bestReverseIndex = i;
                }
            }

            var useForward = bestForwardScore <= bestReverseScore;

            if (!useForward)
            {
                logger.Log($"Path reversed to match heading at index {bestReverseIndex} (forward score {bestForwardScore:F2}, reverse score {bestReverseScore:F2})");
            }

            return (useForward ? bestForwardIndex : bestReverseIndex, useForward);
        }

        /// <summary>
        /// Load a traffic route from disk using the route library.
        /// </summary>
        public void LoadTrafficRoute(string name)
        {
            var route = routeLibrary.Load(name);
            recordedRoutes[name] = route;

            var list = routeLibrary.ConvertToWaypoints(route);
            trafficRoutes[name] = list;
            logger.Log($"Loaded traffic route {name} ({route.Metadata.Type}) with {list.Count} points");
        }

        public List<Util.Waypoint> GetTrafficRoute(string name)
        {
            if (trafficRoutes.TryGetValue(name, out var list))
                return list;
            return new List<Util.Waypoint>();
        }

        /// <summary>
        /// Get the recorded route including metadata for editing.
        /// </summary>
        public RecordedRoute GetRecordedRoute(string name)
        {
            return recordedRoutes.TryGetValue(name, out var route) ? route : null;
        }

        /// <summary>
        /// Clear cached route data so a new track/layout context can be loaded cleanly.
        /// </summary>
        public void ClearRoutes()
        {
            trafficRoutes.Clear();
            recordedRoutes.Clear();
        }

        /// <summary>
        /// Get the detected route type for a named route.
        /// </summary>
        public RouteType GetRouteType(string name)
        {
            if (recordedRoutes.TryGetValue(name, out var route)) return route.Metadata.Type;
            return routeLibrary.GuessRouteType(name);
        }
    }
}
