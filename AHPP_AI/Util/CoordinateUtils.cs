using System;

namespace AHPP_AI.Util
{
    /// <summary>
    ///     Utility class for handling coordinate system conversions and heading calculations for LFS
    /// </summary>
    public static class CoordinateUtils
    {
        // Constants for LFS heading system
        public const int FULL_CIRCLE = 65536;
        public const int HALF_CIRCLE = 32768;
        public const int QUARTER_CIRCLE = 16384;

        /// <summary>
        ///     Calculate the desired heading in LFS format based on a target position relative to current position
        /// </summary>
        /// <param name="targetX">X coordinate of target relative to current position</param>
        /// <param name="targetY">Y coordinate of target relative to current position</param>
        /// <returns>Heading in LFS format (0-65535, where 0=North, 16384=East, 32768=South, 49152=West)</returns>
        public static int CalculateHeadingToTarget(double targetX, double targetY)
        {
            // Calculate angle to target in radians
            var angleToTarget = Math.Atan2(targetY, targetX);

            // Convert to positive angle (0 to 2π)
            if (angleToTarget < 0) angleToTarget += 2 * Math.PI;

            // Convert to LFS heading format
            var desiredHeading = (int)(angleToTarget * FULL_CIRCLE / (2 * Math.PI));

            // Rotate by 90 degrees counter-clockwise
            desiredHeading -= QUARTER_CIRCLE;

            // Normalize
            if (desiredHeading >= FULL_CIRCLE) desiredHeading -= FULL_CIRCLE;

            return desiredHeading;
        }

        public static double CalculateDistance(Vec p1, Vec p2)
        {
            var dx = (p1.X - p2.X) / 65536.0;
            var dy = (p1.Y - p2.Y) / 65536.0;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        ///     Calculate the heading error between current heading and desired heading
        ///     with improved handling for the 180-degree case
        /// </summary>
        /// <param name="currentHeading">Current heading in LFS format</param>
        /// <param name="desiredHeading">Desired heading in LFS format</param>
        /// <returns>Heading error in LFS units (-32768 to 32767)</returns>
        public static int CalculateHeadingError(int currentHeading, int desiredHeading)
        {
            // Normalize both headings to ensure they're in the 0-65535 range
            currentHeading = NormalizeHeading(currentHeading);
            desiredHeading = NormalizeHeading(desiredHeading);

            // Calculate raw difference
            var headingError = desiredHeading - currentHeading;

            // Ensure the error is in the range -32768 to +32767 (shortest path)
            if (headingError > HALF_CIRCLE)
                headingError -= FULL_CIRCLE;
            else if (headingError < -HALF_CIRCLE)
                headingError += FULL_CIRCLE;

            // Special case for when error is exactly half circle (180 degrees)
            // In this case, arbitrarily choose to turn right (positive)
            if (Math.Abs(headingError) == HALF_CIRCLE)
                headingError = HALF_CIRCLE - 1; // Make it slightly less than 180 degrees

            return headingError;
        }

        /// <summary>
        ///     Convert heading from LFS format to degrees (0-359.99)
        /// </summary>
        /// <param name="lfsHeading">Heading in LFS format (0-65535)</param>
        /// <returns>Heading in degrees (0-359.99)</returns>
        public static double HeadingToDegrees(int lfsHeading)
        {
            return lfsHeading * 360.0 / FULL_CIRCLE % 360.0;
        }

        /// <summary>
        ///     Convert heading from degrees to LFS format
        /// </summary>
        /// <param name="degrees">Heading in degrees (0-359.99)</param>
        /// <returns>Heading in LFS format (0-65535)</returns>
        public static int DegreesToHeading(double degrees)
        {
            var heading = (int)(degrees * FULL_CIRCLE / 360.0);

            // Ensure it's within the valid range
            if (heading < 0) heading += FULL_CIRCLE;
            if (heading >= FULL_CIRCLE) heading -= FULL_CIRCLE;

            return heading;
        }

        /// <summary>
        ///     Converts direction to normalized LFS heading value
        /// </summary>
        /// <param name="direction">Direction value to normalize</param>
        /// <returns>Normalized heading in LFS format (0-65535)</returns>
        public static int NormalizeHeading(int direction)
        {
            if (direction < 0) direction += FULL_CIRCLE;
            if (direction >= FULL_CIRCLE) direction -= FULL_CIRCLE;
            return direction;
        }
    }
}