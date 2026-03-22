using System;
using System.Collections.Generic;
using AHPP_AI.Util;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Contains pure policy logic for deciding when a spawn route should hand off to its driving route.
    /// </summary>
    internal static class SpawnRouteTransitionPlanner
    {
        /// <summary>
        ///     Evaluate whether the active spawn route should remain active, transition to its paired driving route, or
        ///     fail because no valid target route can be resolved.
        /// </summary>
        public static SpawnTransitionDecision Evaluate(
            List<Util.Waypoint> spawnPath,
            double xMeters,
            double yMeters,
            double thresholdMeters,
            double loopClosureMeters,
            string spawnRouteName,
            Func<string, string> resolveDrivingRoute)
        {
            var decision = new SpawnTransitionDecision
            {
                SpawnRouteName = spawnRouteName
            };

            var targetRoute = resolveDrivingRoute(spawnRouteName);

            if (spawnPath == null || spawnPath.Count < 2)
                return BuildTransitionDecision("spawn-path-missing");

            var geometry = PathProjection.GetGeometry(spawnPath, loopClosureMeters);
            if (geometry == null || geometry.TotalLength <= 0)
                return BuildTransitionDecision("spawn-geometry-missing");

            if (geometry.IsLoop)
                return BuildTransitionDecision("spawn-loop-route");

            if (!PathProjection.TryProjectToPath(spawnPath, geometry, xMeters, yMeters, out var projection))
                return BuildTransitionDecision("spawn-projection-missing");

            var distanceToEnd = geometry.TotalLength - projection.DistanceAlongPathMeters;
            decision.DistanceToEndMeters = distanceToEnd;

            if (distanceToEnd > Math.Max(1.0, thresholdMeters))
            {
                decision.Action = SpawnTransitionAction.StayOnSpawnRoute;
                decision.ReasonCode = "spawn-continue";
                return decision;
            }

            if (string.IsNullOrWhiteSpace(targetRoute))
                return BuildTransitionDecision("spawn-target-missing", distanceToEnd);

            decision.Action = SpawnTransitionAction.TransitionToDrivingRoute;
            decision.ReasonCode = "spawn-transition-ready";
            decision.TargetRouteName = targetRoute;
            return decision;

            SpawnTransitionDecision BuildTransitionDecision(string reasonCode, double distanceToEndMeters = double.MaxValue)
            {
                if (string.IsNullOrWhiteSpace(targetRoute))
                {
                    return new SpawnTransitionDecision
                    {
                        Action = SpawnTransitionAction.Fail,
                        ReasonCode = reasonCode,
                        SpawnRouteName = spawnRouteName,
                        DistanceToEndMeters = distanceToEndMeters
                    };
                }

                return new SpawnTransitionDecision
                {
                    Action = SpawnTransitionAction.TransitionToDrivingRoute,
                    ReasonCode = reasonCode,
                    SpawnRouteName = spawnRouteName,
                    TargetRouteName = targetRoute,
                    DistanceToEndMeters = distanceToEndMeters
                };
            }
        }
    }
}
