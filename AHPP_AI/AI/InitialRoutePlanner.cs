using System;
using System.Collections.Generic;
using AHPP_AI.Util;
using AHPP_AI.Waypoint;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Contains pure policy logic for creating and selecting initial route candidates.
    /// </summary>
    internal static class InitialRoutePlanner
    {
        /// <summary>
        ///     Build a scored route candidate, reversing the path when the spawn heading best fits the opposite travel
        ///     direction.
        /// </summary>
        public static RouteCandidate? CreateCandidate(
            string routeLabel,
            string routeName,
            List<Util.Waypoint> path,
            int? preferredStartIndex,
            bool assumeClockwise,
            bool isLoop,
            AIConfig.DrivingMode drivingMode,
            BranchRouteInfo? branchInfo,
            Vec position,
            int heading,
            Func<Vec, int, List<Util.Waypoint>, bool, bool, (double score, int startIndex, bool clockwise)> scoreRouteFit,
            Func<Vec, Util.Waypoint, double> calculateWaypointDistance)
        {
            if (path == null || path.Count < 2)
                return null;

            var (score, scoredIndex, clockwise) = scoreRouteFit(position, heading, path, isLoop, false);
            var startIndex = preferredStartIndex ?? scoredIndex;
            var appliedPath = path;
            var appliedIndex = Math.Max(0, Math.Min(path.Count - 1, startIndex));
            var useClockwise = preferredStartIndex.HasValue ? assumeClockwise : clockwise;

            if (!useClockwise)
            {
                appliedPath = new List<Util.Waypoint>(path);
                appliedPath.Reverse();
                appliedIndex = appliedPath.Count - 1 - appliedIndex;
            }

            return new RouteCandidate
            {
                RouteLabel = routeLabel,
                RouteName = routeName,
                Path = appliedPath,
                StartIndex = appliedIndex,
                IsLoop = isLoop,
                DrivingMode = drivingMode,
                BranchInfo = branchInfo,
                Score = score,
                NearestDistance = calculateWaypointDistance(position, appliedPath[appliedIndex]),
                HeadingError = CalculateHeadingError(appliedPath, appliedIndex, heading, isLoop)
            };
        }

        /// <summary>
        ///     Reject on-track candidates that are too far from the spawn position or require a large heading flip.
        /// </summary>
        public static bool IsViableTrackCandidate(
            RouteCandidate? candidate,
            double maxTrackInitialRouteDistanceMeters,
            double maxTrackInitialHeadingErrorDegrees)
        {
            if (candidate == null)
                return false;

            var headingErrorDegrees = Math.Abs(candidate.HeadingError) * 360.0 / CoordinateUtils.FULL_CIRCLE;
            return candidate.NearestDistance <= maxTrackInitialRouteDistanceMeters &&
                   headingErrorDegrees <= maxTrackInitialHeadingErrorDegrees;
        }

        /// <summary>
        ///     Reject spawn-route candidates that are too far from the actual spawn position to be believable.
        /// </summary>
        public static bool IsViableSpawnCandidate(
            RouteCandidate? candidate,
            double maxSpawnInitialRouteDistanceMeters)
        {
            if (candidate == null)
                return false;

            return candidate.NearestDistance <= maxSpawnInitialRouteDistanceMeters;
        }

        /// <summary>
        ///     Choose between a spawn-route and on-track candidate using the existing score and plausibility thresholds.
        /// </summary>
        public static InitialRouteDecision SelectInitialRoute(
            RouteCandidate? spawnCandidate,
            RouteCandidate? trackCandidate,
            double spawnRouteFarDistanceThresholdMeters,
            double onTrackSpawnScoreAdvantage,
            double trackSpawnScoreAdvantage)
        {
            var result = new InitialRouteDecision
            {
                SpawnCandidate = spawnCandidate,
                TrackCandidate = trackCandidate
            };

            if (trackCandidate == null)
            {
                result.SelectedCandidate = spawnCandidate;
                result.ReasonCode = spawnCandidate == null ? "initial-no-candidate" : "initial-spawn-only";
                return result;
            }

            if (spawnCandidate == null)
            {
                result.SelectedCandidate = trackCandidate;
                result.ReasonCode = "initial-track-only";
                return result;
            }

            if (spawnCandidate.NearestDistance > spawnRouteFarDistanceThresholdMeters &&
                trackCandidate.Score + onTrackSpawnScoreAdvantage < spawnCandidate.Score)
            {
                result.SelectedCandidate = trackCandidate;
                result.ReasonCode = "initial-track-overrides-far-spawn";
                return result;
            }

            if (trackCandidate.Score + trackSpawnScoreAdvantage < spawnCandidate.Score)
            {
                result.SelectedCandidate = trackCandidate;
                result.ReasonCode = "initial-track-better-score";
                return result;
            }

            result.SelectedCandidate = spawnCandidate;
            result.ReasonCode = "initial-spawn-kept";
            return result;
        }

        /// <summary>
        ///     Measure the heading error from the selected start waypoint toward the next waypoint in travel order.
        /// </summary>
        public static int CalculateHeadingError(
            List<Util.Waypoint> path,
            int startIndex,
            int heading,
            bool isLoop)
        {
            if (path == null || path.Count < 2)
                return 0;

            var nextIndex = startIndex + 1;
            if (nextIndex >= path.Count)
            {
                nextIndex = isLoop ? 0 : path.Count - 1;
                if (nextIndex == startIndex)
                    nextIndex = Math.Max(0, startIndex - 1);
            }

            var current = path[startIndex];
            var next = path[nextIndex];
            var desiredHeading = CoordinateUtils.CalculateHeadingToTarget(
                next.Position.X / 65536.0 - current.Position.X / 65536.0,
                next.Position.Y / 65536.0 - current.Position.Y / 65536.0);

            return CoordinateUtils.CalculateHeadingError(
                CoordinateUtils.NormalizeHeading(heading),
                desiredHeading);
        }
    }
}
