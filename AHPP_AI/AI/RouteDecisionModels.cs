using System.Collections.Generic;
using AHPP_AI.Util;
using AHPP_AI.Waypoint;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Describes a fully scored route candidate for initial spawn classification.
    /// </summary>
    internal sealed class RouteCandidate
    {
        public string RouteLabel { get; set; } = "spawn";
        public string RouteName { get; set; } = string.Empty;
        public List<Util.Waypoint> Path { get; set; } = new List<Util.Waypoint>();
        public int StartIndex { get; set; }
        public bool IsLoop { get; set; }
        public AIConfig.DrivingMode DrivingMode { get; set; } = AIConfig.DrivingMode.Cruise;
        public BranchRouteInfo? BranchInfo { get; set; }
        public double Score { get; set; } = double.MaxValue;
        public double NearestDistance { get; set; } = double.MaxValue;
        public int HeadingError { get; set; }
    }

    /// <summary>
    ///     Captures the final spawn-time route decision between spawn and on-track candidates.
    /// </summary>
    internal sealed class InitialRouteDecision
    {
        public RouteCandidate? SelectedCandidate { get; set; }
        public RouteCandidate? SpawnCandidate { get; set; }
        public RouteCandidate? TrackCandidate { get; set; }
        public string ReasonCode { get; set; } = string.Empty;
    }

    /// <summary>
    ///     Enumerates the possible outcomes when evaluating whether a spawn route should hand off.
    /// </summary>
    internal enum SpawnTransitionAction
    {
        StayOnSpawnRoute,
        TransitionToDrivingRoute,
        Fail
    }

    /// <summary>
    ///     Describes the deterministic outcome of a spawn-route transition evaluation.
    /// </summary>
    internal sealed class SpawnTransitionDecision
    {
        public SpawnTransitionAction Action { get; set; } = SpawnTransitionAction.Fail;
        public string ReasonCode { get; set; } = string.Empty;
        public string SpawnRouteName { get; set; } = string.Empty;
        public string TargetRouteName { get; set; } = string.Empty;
        public double DistanceToEndMeters { get; set; } = double.MaxValue;
    }
}
