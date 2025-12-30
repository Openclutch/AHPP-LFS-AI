using System;
using SixLabors.ImageSharp;

namespace AHPP_AI.Util
{
    /// <summary>
    ///     Represents a 3D vector with integer coordinates, used for positions
    ///     in LFS coordinates (65536 units = 1 meter)
    /// </summary>
    public struct Vec
    {
        public int X;
        public int Y;
        public int Z;

        /// <summary>
        ///     Creates a new Vec with the specified coordinates
        /// </summary>
        public Vec(int x, int y, int z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        /// <summary>
        ///     Creates a new Vec with only X and Y set
        /// </summary>
        public Vec(int x, int y)
        {
            X = x;
            Y = y;
            Z = 0;
        }

        /// <summary>
        ///     Convert double values (meters) to LFS integer coordinates
        /// </summary>
        public static Vec FromMeters(double xMeters, double yMeters, double zMeters = 0)
        {
            return new Vec(
                (int)(xMeters * 65536.0),
                (int)(yMeters * 65536.0),
                (int)(zMeters * 65536.0)
            );
        }

        /// <summary>
        ///     Calculate distance between two vectors
        /// </summary>
        public static double Distance(Vec v1, Vec v2)
        {
            var dx = (v1.X - v2.X) / 65536.0;
            var dy = (v1.Y - v2.Y) / 65536.0;
            var dz = (v1.Z - v2.Z) / 65536.0;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>
        ///     Get vector as string representation
        /// </summary>
        public override string ToString()
        {
            return $"({X / 65536.0:F2}, {Y / 65536.0:F2}, {Z / 65536.0:F2})";
        }
    }

    /// <summary>
    ///     Represents a waypoint in the game world
    /// </summary>
    public struct Waypoint
    {
        /// <summary>
        ///     Elevation in meters for the waypoint position.
        /// </summary>
        public double ZMeters { get; }

        /// <summary>
        ///     Position in LFS coordinates (65536 units = 1 meter)
        /// </summary>
        public Point Position;

        /// <summary>
        ///     Maximum recommended speed in km/h
        /// </summary>
        public double SpeedLimit;

        /// <summary>
        ///     Route index for identification
        /// </summary>
        public byte RouteIndex;

        /// <summary>
        ///     Creates a waypoint from integer pixel coordinates
        /// </summary>
        public Waypoint(int x, int y, double speedLimit, byte routeIndex = 0, double zMeters = 0)
        {
            Position = new Point(x, y);
            SpeedLimit = speedLimit;
            RouteIndex = routeIndex;
            ZMeters = zMeters;
        }

        /// <summary>
        ///     Creates a waypoint from real-world coordinates in meters
        /// </summary>
        public Waypoint(double xMeters, double yMeters, double speedLimit, byte routeIndex = 0, double zMeters = 0)
        {
            var x = (int)(xMeters * 65536);
            var y = (int)(yMeters * 65536);
            Position = new Point(x, y);
            SpeedLimit = speedLimit;
            RouteIndex = routeIndex;
            ZMeters = zMeters;
        }

        /// <summary>
        ///     Get waypoint as string representation
        /// </summary>
        public override string ToString()
        {
            return
                $"Waypoint {RouteIndex}: ({Position.X / 65536.0:F2}, {Position.Y / 65536.0:F2}, {ZMeters:F1}m) - {SpeedLimit:F1} km/h";
        }
    }

    /// <summary>
    ///     Represents a 3D vector with floating point coordinates
    /// </summary>
    public struct Vector
    {
        public float X;
        public float Y;
        public float Z;

        /// <summary>
        ///     Creates a new Vector with the specified coordinates
        /// </summary>
        public Vector(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        /// <summary>
        ///     Calculate magnitude (length) of the vector
        /// </summary>
        public float Magnitude()
        {
            return (float)Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        /// <summary>
        ///     Get vector as string representation
        /// </summary>
        public override string ToString()
        {
            return $"({X:F2}, {Y:F2}, {Z:F2})";
        }
    }
}
