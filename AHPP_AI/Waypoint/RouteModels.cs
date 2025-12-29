using System.Collections.Generic;
using AHPP_AI.Util;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    /// Logical type of a recorded driving route.
    /// </summary>
    public enum RouteType
    {
        Unknown = 0,
        MainLoop,
        AlternateMain,
        PitEntry,
        Detour
    }

    /// <summary>
    /// Metadata describing how a recorded route should be used.
    /// </summary>
    public class RouteMetadata
    {
        public string Name { get; set; } = string.Empty;
        public RouteType Type { get; set; } = RouteType.Unknown;
        public string Description { get; set; } = string.Empty;
        public string Track { get; set; } = string.Empty;
        public string Layout { get; set; } = string.Empty;
        public bool IsLoop { get; set; }
        public int? AttachMainIndex { get; set; }
        public int? RejoinMainIndex { get; set; }
        public double? DefaultSpeedLimit { get; set; }
        public int? AiTargetCount { get; set; }
        public double? AiTargetPercent { get; set; }
        public double? AiWeight { get; set; }
        public bool AiEnabled { get; set; } = true;
    }

    /// <summary>
    /// Single recorded position for a route.
    /// </summary>
    public class RoutePoint
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Speed { get; set; }
        public double? SpeedLimit { get; set; }
        public float Throttle { get; set; }
        public float Brake { get; set; }
        public float Steering { get; set; }
        public int Heading { get; set; }
        public string Note { get; set; }
    }

    /// <summary>
    /// Wrapper combining metadata and recorded nodes.
    /// </summary>
    public class RecordedRoute
    {
        public RouteMetadata Metadata { get; set; } = new RouteMetadata();
        public List<RoutePoint> Nodes { get; set; } = new List<RoutePoint>();

        /// <summary>
        /// Convert recorded nodes into waypoint list for AI or visualization.
        /// </summary>
        public List<Util.Waypoint> ToWaypoints()
        {
            var waypoints = new List<Util.Waypoint>();
            if (Nodes == null) return waypoints;

            byte idx = 0;
            foreach (var node in Nodes)
            {
                var speed = node.SpeedLimit ?? node.Speed;
                waypoints.Add(new Util.Waypoint(node.X, node.Y, speed, idx++));
            }

            return waypoints;
        }
    }

    /// <summary>
    /// Legacy wrapper for saved routes, kept for compatibility with older JSON files.
    /// </summary>
    public class SavedRoute
    {
        public string Name { get; set; }
        public string Type { get; set; }
        public List<RoutePoint> Nodes { get; set; }
    }
}
