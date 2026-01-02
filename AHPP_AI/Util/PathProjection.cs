using System;
using System.Collections.Generic;

namespace AHPP_AI.Util
{
    /// <summary>
    ///     Stores derived geometry information for a path of waypoints.
    /// </summary>
    public class PathGeometry
    {
        public int PointCount { get; set; }
        public int SegmentCount { get; set; }
        public double[] CumulativeDistances { get; set; } = Array.Empty<double>();
        public double[] SegmentLengths { get; set; } = Array.Empty<double>();
        public double TotalLength { get; set; }
        public bool IsLoop { get; set; }
        public double LoopClosureMeters { get; set; }
    }

    /// <summary>
    ///     Projection details for a position along a path.
    /// </summary>
    public struct PathProjectionResult
    {
        public int SegmentIndex;
        public double DistanceAlongPathMeters;
        public double LateralOffsetMeters;
        public double DistanceToPathMeters;
        public double DirectionX;
        public double DirectionY;
    }

    /// <summary>
    ///     Helpers for projecting world positions onto waypoint paths.
    /// </summary>
    public static class PathProjection
    {
        private static readonly object CacheLock = new object();
        private static readonly Dictionary<List<Waypoint>, PathGeometry> GeometryCache =
            new Dictionary<List<Waypoint>, PathGeometry>();

        /// <summary>
        ///     Build or fetch cached geometry for a waypoint path.
        /// </summary>
        public static PathGeometry GetGeometry(List<Waypoint> path, double loopClosureMeters)
        {
            if (path == null || path.Count < 2) return null;

            lock (CacheLock)
            {
                if (GeometryCache.TryGetValue(path, out var cached) &&
                    cached.PointCount == path.Count &&
                    Math.Abs(cached.LoopClosureMeters - loopClosureMeters) < 0.01)
                    return cached;
            }

            var geometry = BuildGeometry(path, loopClosureMeters);

            lock (CacheLock)
            {
                GeometryCache[path] = geometry;
            }

            return geometry;
        }

        /// <summary>
        ///     Project a world position onto the closest segment of the path.
        /// </summary>
        public static bool TryProjectToPath(
            List<Waypoint> path,
            PathGeometry geometry,
            double xMeters,
            double yMeters,
            out PathProjectionResult result)
        {
            result = default;
            if (path == null || geometry == null || path.Count < 2 || geometry.SegmentCount <= 0)
                return false;

            var bestDistanceSq = double.MaxValue;
            var bestSegment = -1;
            var bestT = 0.0;
            var bestProjX = 0.0;
            var bestProjY = 0.0;
            var bestDirX = 1.0;
            var bestDirY = 0.0;
            var bestSegmentLength = 0.0;

            for (var i = 0; i < geometry.SegmentCount; i++)
            {
                var start = path[i].Position;
                var end = path[(i + 1) % path.Count].Position;
                var ax = start.X / 65536.0;
                var ay = start.Y / 65536.0;
                var bx = end.X / 65536.0;
                var by = end.Y / 65536.0;

                var sx = bx - ax;
                var sy = by - ay;
                var segLenSq = sx * sx + sy * sy;
                if (segLenSq <= 0.0001) continue;

                var t = ((xMeters - ax) * sx + (yMeters - ay) * sy) / segLenSq;
                if (t < 0) t = 0;
                else if (t > 1) t = 1;

                var projX = ax + t * sx;
                var projY = ay + t * sy;
                var dx = xMeters - projX;
                var dy = yMeters - projY;
                var distSq = dx * dx + dy * dy;

                if (distSq < bestDistanceSq)
                {
                    bestDistanceSq = distSq;
                    bestSegment = i;
                    bestT = t;
                    bestProjX = projX;
                    bestProjY = projY;

                    var segLen = Math.Sqrt(segLenSq);
                    bestSegmentLength = segLen;
                    bestDirX = sx / segLen;
                    bestDirY = sy / segLen;
                }
            }

            if (bestSegment < 0) return false;

            var distanceAlong = geometry.CumulativeDistances[bestSegment] + bestT * bestSegmentLength;
            var offsetX = xMeters - bestProjX;
            var offsetY = yMeters - bestProjY;
            var lateral = bestDirX * offsetY - bestDirY * offsetX;

            result = new PathProjectionResult
            {
                SegmentIndex = bestSegment,
                DistanceAlongPathMeters = distanceAlong,
                LateralOffsetMeters = lateral,
                DistanceToPathMeters = Math.Sqrt(bestDistanceSq),
                DirectionX = bestDirX,
                DirectionY = bestDirY
            };
            return true;
        }

        /// <summary>
        ///     Calculate forward distance from one path coordinate to another.
        /// </summary>
        public static double GetForwardDistance(PathGeometry geometry, double fromDistance, double toDistance)
        {
            var delta = toDistance - fromDistance;
            if (!geometry.IsLoop) return delta;
            if (delta < 0) delta += geometry.TotalLength;
            return delta;
        }

        /// <summary>
        ///     Calculate backward distance from one path coordinate to another.
        /// </summary>
        public static double GetBackwardDistance(PathGeometry geometry, double fromDistance, double toDistance)
        {
            if (!geometry.IsLoop) return fromDistance - toDistance;
            var forward = GetForwardDistance(geometry, fromDistance, toDistance);
            return geometry.TotalLength - forward;
        }

        /// <summary>
        ///     Rebuild geometry cache entry for a path.
        /// </summary>
        private static PathGeometry BuildGeometry(List<Waypoint> path, double loopClosureMeters)
        {
            var geometry = new PathGeometry
            {
                PointCount = path.Count,
                LoopClosureMeters = loopClosureMeters,
                IsLoop = IsLoopPath(path, loopClosureMeters)
            };

            geometry.SegmentCount = geometry.IsLoop ? path.Count : Math.Max(0, path.Count - 1);
            geometry.CumulativeDistances = new double[path.Count];
            geometry.SegmentLengths = new double[path.Count];

            var total = 0.0;
            for (var i = 0; i < geometry.SegmentCount; i++)
            {
                var start = path[i];
                var end = path[(i + 1) % path.Count];
                var length = DistanceMeters(start, end);
                geometry.SegmentLengths[i] = length;

                if (i + 1 < path.Count)
                    geometry.CumulativeDistances[i + 1] = geometry.CumulativeDistances[i] + length;

                total += length;
            }

            geometry.TotalLength = total;
            return geometry;
        }

        /// <summary>
        ///     Determine if a path should be treated as a closed loop.
        /// </summary>
        private static bool IsLoopPath(List<Waypoint> path, double loopClosureMeters)
        {
            if (path == null || path.Count < 2) return false;
            var first = path[0];
            var last = path[path.Count - 1];
            return DistanceMeters(first, last) <= loopClosureMeters;
        }

        /// <summary>
        ///     Calculate distance between two waypoints in meters.
        /// </summary>
        private static double DistanceMeters(Waypoint a, Waypoint b)
        {
            var dx = (a.Position.X - b.Position.X) / 65536.0;
            var dy = (a.Position.Y - b.Position.Y) / 65536.0;
            return Math.Sqrt(dx * dx + dy * dy);
        }
    }
}
