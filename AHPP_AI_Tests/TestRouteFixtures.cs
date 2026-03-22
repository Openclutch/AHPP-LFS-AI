using System.Collections.Generic;
using AHPP_AI.Util;

namespace AHPP_AI_Tests
{
    /// <summary>
    ///     Builds small synthetic waypoint paths for deterministic route tests.
    /// </summary>
    internal static class TestRouteFixtures
    {
        /// <summary>
        ///     Create a straight path on the X axis in metres.
        /// </summary>
        public static List<Waypoint> StraightPath(params double[] xPositions)
        {
            var path = new List<Waypoint>();
            for (var i = 0; i < xPositions.Length; i++)
                path.Add(new Waypoint(xPositions[i], 0.0, 60.0, (byte)i));

            return path;
        }

        /// <summary>
        ///     Create a simple square loop.
        /// </summary>
        public static List<Waypoint> SquareLoop(double sizeMeters = 10.0)
        {
            return new List<Waypoint>
            {
                new Waypoint(0.0, 0.0, 50.0, 0),
                new Waypoint(sizeMeters, 0.0, 50.0, 1),
                new Waypoint(sizeMeters, sizeMeters, 50.0, 2),
                new Waypoint(0.0, sizeMeters, 50.0, 3),
                new Waypoint(0.0, 0.0, 50.0, 4)
            };
        }

        /// <summary>
        ///     Calculate simple Euclidean distance to a waypoint in metres.
        /// </summary>
        public static double DistanceToWaypoint(Vec position, Waypoint waypoint)
        {
            var posX = position.X / 65536.0;
            var posY = position.Y / 65536.0;
            var wpX = waypoint.Position.X / 65536.0;
            var wpY = waypoint.Position.Y / 65536.0;
            var dx = wpX - posX;
            var dy = wpY - posY;
            return System.Math.Sqrt(dx * dx + dy * dy);
        }
    }
}
