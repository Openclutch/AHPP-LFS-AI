using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    ///     Extracts waypoints from an image based on colored routes
    /// </summary>
    public class RouteExtractor
    {
        private static readonly Logger logger = new Logger("route_extractor.log", false);

        /// <summary>
        ///     Extracts waypoints from an image with adaptive spacing (more in corners, fewer on straights)
        /// </summary>
        /// <param name="imagePath">Path to the image file</param>
        /// <param name="targetColor">Color of the line to extract (e.g., Color.Red)</param>
        /// <param name="tolerance">Color tolerance for matching (e.g., 50)</param>
        /// <param name="sampleInterval">Base interval for sampling waypoints</param>
        /// <returns>List of waypoints in clockwise order with adaptive spacing</returns>
        public static List<Point> ExtractWaypoints(string imagePath, Rgba32 targetColor, int tolerance,
            int sampleInterval)
        {
            // Validate image file
            if (!File.Exists(imagePath))
            {
                logger.LogError($"Image file not found at: {imagePath}");
                return new List<Point>();
            }

            Image<Rgba32> image;
            try
            {
                image = Image.Load<Rgba32>(imagePath);
                logger.Log($"Loaded image: {image.Width}x{image.Height} pixels");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Failed to load image");
                return new List<Point>();
            }

            logger.Log(
                $"Processing image looking for color: R:{targetColor.R} G:{targetColor.G} B:{targetColor.B}, tolerance: {tolerance}");

            try
            {
                // Create cache of all target color pixels for better performance
                var targetPixels = new HashSet<Point>();

                logger.Log("Scanning image for target color pixels...");
                for (var y = 0; y < image.Height; y += 2)
                for (var x = 0; x < image.Width; x += 2)
                    if (IsTargetColor(image[x, y], targetColor, tolerance))
                    {
                        targetPixels.Add(new Point(x, y));

                        // Add adjacent pixels to improve connectivity
                        for (var dy = -1; dy <= 1; dy++)
                        for (var dx = -1; dx <= 1; dx++)
                        {
                            if (dx == 0 && dy == 0) continue;

                            var nx = x + dx;
                            var ny = y + dy;

                            if (nx >= 0 && nx < image.Width && ny >= 0 && ny < image.Height &&
                                IsTargetColor(image[nx, ny], targetColor, tolerance))
                                targetPixels.Add(new Point(nx, ny));
                        }
                    }

                logger.Log($"Found {targetPixels.Count} pixels matching target color");

                if (targetPixels.Count == 0)
                {
                    logger.LogError($"No pixels found for color {targetColor}");
                    return new List<Point>();
                }

                // Find the starting pixel - use pixel closest to the edge of the track
                var start = FindStartingPixel(image, targetPixels);
                logger.Log($"Starting point found at ({start.X}, {start.Y})");

                // Trace the path from the starting point with improved contour following
                var path = TracePath(image, start, targetPixels);
                if (path.Count < 3)
                {
                    logger.LogError("Path is too short to form a valid route");
                    return path;
                }

                logger.Log($"Traced path with {path.Count} points");

                // Ensure path is in clockwise order for LFS
                EnsureClockwiseOrder(ref path);

                // Apply bezier smoothing for smoother path
                var smoothedPath = SmoothPathWithBezier(path, 4);
                logger.Log($"Smoothed path to {smoothedPath.Count} points using Bezier curves");

                // Calculate curvature along the path to determine waypoint spacing
                var curvature = new List<double>();

                for (var i = 0; i < smoothedPath.Count; i++)
                {
                    // Get previous, current, and next points (handling wrap-around)
                    var prevIdx = (i - 1 + smoothedPath.Count) % smoothedPath.Count;
                    var nextIdx = (i + 1) % smoothedPath.Count;

                    var prev = smoothedPath[prevIdx];
                    var current = smoothedPath[i];
                    var next = smoothedPath[nextIdx];

                    // Calculate vectors
                    double dx1 = current.X - prev.X;
                    double dy1 = current.Y - prev.Y;
                    double dx2 = next.X - current.X;
                    double dy2 = next.Y - current.Y;

                    // Calculate angles
                    var angle1 = Math.Atan2(dy1, dx1);
                    var angle2 = Math.Atan2(dy2, dx2);

                    // Calculate change in angle (curvature)
                    var angleDiff = angle2 - angle1;

                    // Normalize to range [-π, π]
                    if (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
                    if (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

                    // The absolute value of this angle difference is our curvature measure
                    curvature.Add(Math.Abs(angleDiff));
                }

                // Smooth the curvature values
                for (var i = 0; i < curvature.Count; i++)
                {
                    double sum = 0;
                    var count = 0;
                    var windowSize = 5;

                    for (var j = i - windowSize / 2; j <= i + windowSize / 2; j++)
                    {
                        var idx = (j + curvature.Count) % curvature.Count; // Handle wrap-around
                        sum += curvature[idx];
                        count++;
                    }

                    curvature[i] = sum / count;
                }

                // Normalize curvature to [0,1] range
                var maxCurvature = curvature.Max();
                if (maxCurvature > 0)
                    for (var i = 0; i < curvature.Count; i++)
                        curvature[i] = curvature[i] / maxCurvature;

                // Sample waypoints adaptively based on curvature - more in corners, fewer on straights
                var waypoints = new List<Point>();
                var minSpacing = Math.Max(1, sampleInterval / 2);
                var maxSpacing = sampleInterval * 2;

                // Always include the first point
                waypoints.Add(smoothedPath[0]);

                // Calculate total path length
                double totalLength = 0;
                for (var i = 0; i < smoothedPath.Count - 1; i++)
                    totalLength += DistanceBetween(smoothedPath[i], smoothedPath[i + 1]);

                // Add closed loop segment if needed
                if (smoothedPath[0] != smoothedPath[smoothedPath.Count - 1])
                    totalLength += DistanceBetween(smoothedPath[smoothedPath.Count - 1], smoothedPath[0]);

                // Add remaining points based on adaptive spacing
                double distanceCovered = 0;
                var currentSegment = 0;
                double currentSegmentStart = 0;

                logger.Log(
                    $"Sampling adaptive waypoints with min spacing {minSpacing}px and max spacing {maxSpacing}px");

                while (distanceCovered < totalLength)
                {
                    // Get curvature at current position
                    var positionRatio = distanceCovered / totalLength;
                    var curveIndex = (int)(positionRatio * smoothedPath.Count) % smoothedPath.Count;
                    var curveValue = curvature[curveIndex];

                    // Calculate spacing based on curvature - square for more pronounced effect
                    var spacingFactor = 1.0 - Math.Pow(curveValue, 2);
                    var desiredSpacing = minSpacing + spacingFactor * (maxSpacing - minSpacing);

                    // Advance by the calculated spacing
                    distanceCovered += desiredSpacing;
                    if (distanceCovered >= totalLength) break;

                    // Find point at this distance
                    var targetDistance = distanceCovered;

                    // Find which segment contains the target distance
                    while (true)
                    {
                        var nextSegmentIndex = (currentSegment + 1) % smoothedPath.Count;
                        var segmentLength =
                            DistanceBetween(smoothedPath[currentSegment], smoothedPath[nextSegmentIndex]);
                        var segmentEnd = currentSegmentStart + segmentLength;

                        if (segmentEnd >= targetDistance || currentSegment == smoothedPath.Count - 1)
                        {
                            // This segment contains our target point
                            var segmentPos = (targetDistance - currentSegmentStart) / segmentLength;
                            var x = (int)Math.Round(smoothedPath[currentSegment].X +
                                                    segmentPos * (smoothedPath[nextSegmentIndex].X -
                                                                  smoothedPath[currentSegment].X));
                            var y = (int)Math.Round(smoothedPath[currentSegment].Y +
                                                    segmentPos * (smoothedPath[nextSegmentIndex].Y -
                                                                  smoothedPath[currentSegment].Y));

                            waypoints.Add(new Point(x, y));
                            break;
                        }

                        // Move to next segment
                        currentSegmentStart = segmentEnd;
                        currentSegment = (currentSegment + 1) % smoothedPath.Count;

                        // Prevent infinite loop
                        if (currentSegment == 0) break;
                    }
                }

                logger.Log($"Generated {waypoints.Count} adaptive waypoints");


                // After adaptive waypoint generation, before returning:
                var smoothedWaypoints = SmoothStraightSegments(waypoints);
                logger.Log($"Smoothed straight segments, result: {smoothedWaypoints.Count} points");

                return smoothedWaypoints;
            }
            finally
            {
                // Dispose the bitmap to free resources
                image.Dispose();
            }
        }

        /// <summary>
        ///     Smooths and reduces waypoints on straight segments while preserving some intermediate points
        /// </summary>
        private static List<Point> SmoothStraightSegments(List<Point> waypoints, double angleThreshold = 0.05,
            int minSegmentLength = 3, int maxSpacingOnStraight = 20)
        {
            if (waypoints.Count < minSegmentLength)
                return waypoints;

            var result = new List<Point>();
            var segmentStart = 0;
            var inStraightSegment = false;

            // Add first point
            result.Add(waypoints[0]);

            // Detect straight segments
            for (var i = 1; i < waypoints.Count - 1; i++)
            {
                // Calculate angle between vectors
                var v1 = new { x = waypoints[i].X - waypoints[i - 1].X, y = waypoints[i].Y - waypoints[i - 1].Y };
                var v2 = new { x = waypoints[i + 1].X - waypoints[i].X, y = waypoints[i + 1].Y - waypoints[i].Y };

                // Calculate magnitude of vectors
                var mag1 = Math.Sqrt(v1.x * v1.x + v1.y * v1.y);
                var mag2 = Math.Sqrt(v2.x * v2.x + v2.y * v2.y);

                if (mag1 < 0.001 || mag2 < 0.001)
                {
                    result.Add(waypoints[i]);
                    continue;
                }

                // Calculate cosine of angle between vectors
                var cosAngle = (v1.x * v2.x + v1.y * v2.y) / (mag1 * mag2);
                cosAngle = Math.Min(1.0, Math.Max(-1.0, cosAngle)); // Clamp to [-1, 1]
                var angle = Math.Acos(cosAngle);

                // Check if we're in a straight segment
                if (angle < angleThreshold)
                {
                    if (!inStraightSegment)
                    {
                        segmentStart = i - 1;
                        inStraightSegment = true;
                    }
                }
                else
                {
                    // If we were in a straight segment, smooth it
                    if (inStraightSegment && i - segmentStart >= minSegmentLength)
                    {
                        // Remove any points we added from this segment
                        while (result.Count > segmentStart + 1)
                            result.RemoveAt(result.Count - 1);

                        // Create a perfect straight line between start and end of segment
                        var start = waypoints[segmentStart];
                        var end = waypoints[i];

                        // Calculate total segment length
                        var segmentLength = DistanceBetween(start, end);

                        // Calculate number of intermediate points based on segment length
                        var numPoints = Math.Max(1, (int)(segmentLength / maxSpacingOnStraight));

                        // Add intermediate points along the perfect line
                        for (var j = 1; j <= numPoints; j++)
                        {
                            var ratio = j / (double)(numPoints + 1);
                            var x = (int)Math.Round(start.X + ratio * (end.X - start.X));
                            var y = (int)Math.Round(start.Y + ratio * (end.Y - start.Y));
                            result.Add(new Point(x, y));
                        }

                        result.Add(end);
                        logger.Log(
                            $"Smoothed straight segment from ({start.X},{start.Y}) to ({end.X},{end.Y}) with {numPoints + 2} points");
                    }
                    else if (inStraightSegment)
                    {
                        // Not long enough to be considered a proper straight
                        for (var j = segmentStart + 1; j <= i; j++)
                            if (!result.Contains(waypoints[j]))
                                result.Add(waypoints[j]);
                    }
                    else
                    {
                        result.Add(waypoints[i]);
                    }

                    inStraightSegment = false;
                }
            }

            // Handle final segment
            if (inStraightSegment && waypoints.Count - 1 - segmentStart >= minSegmentLength)
            {
                // Remove any points we added from this segment
                while (result.Count > segmentStart + 1)
                    result.RemoveAt(result.Count - 1);

                var start = waypoints[segmentStart];
                var end = waypoints[waypoints.Count - 1];

                // Calculate total segment length
                var segmentLength = DistanceBetween(start, end);

                // Calculate number of intermediate points based on segment length
                var numPoints = Math.Max(1, (int)(segmentLength / maxSpacingOnStraight));

                // Add intermediate points along the perfect line
                for (var j = 1; j <= numPoints; j++)
                {
                    var ratio = j / (double)(numPoints + 1);
                    var x = (int)Math.Round(start.X + ratio * (end.X - start.X));
                    var y = (int)Math.Round(start.Y + ratio * (end.Y - start.Y));
                    result.Add(new Point(x, y));
                }

                result.Add(end);
            }
            else if (!result.Contains(waypoints[waypoints.Count - 1]))
            {
                result.Add(waypoints[waypoints.Count - 1]);
            }

            return result;
        }

        /// <summary>
        ///     Improved algorithm to find the best starting pixel, preferring outer edges
        /// </summary>
        private static Point FindStartingPixel(Image<Rgba32> image, HashSet<Point> targetPixels)
        {
            if (targetPixels.Count == 0)
                return new Point(-1, -1);

            // Start in the middle and spiral outward to find the first colored pixel
            var centerX = image.Width / 2;
            var centerY = image.Height / 2;

            logger.Log($"Searching for start pixel from center ({centerX}, {centerY})");

            // Spiral search - start at center and work outward
            var x = centerX;
            var y = centerY;
            var dx = 0;
            var dy = -1;
            var maxLength = Math.Max(image.Width, image.Height);

            for (var i = 0; i < maxLength * maxLength; i++)
            {
                if (x >= 0 && x < image.Width && y >= 0 && y < image.Height)
                    if (targetPixels.Contains(new Point(x, y)))
                    {
                        logger.Log($"Found starting point at ({x}, {y})");
                        return new Point(x, y);
                    }

                // Spiral algorithm
                if (x == y || (x < 0 && x == -y) || (x > 0 && x == 1 - y))
                {
                    var temp = dx;
                    dx = -dy;
                    dy = temp;
                }

                x += dx;
                y += dy;

                // Break if we've gone too far
                if (Math.Abs(x - centerX) > image.Width / 2 || Math.Abs(y - centerY) > image.Height / 2)
                    break;
            }

            // If spiral search fails, fallback to first pixel in set
            logger.LogWarning("Spiral search did not find a start point, using first available pixel");
            return targetPixels.First();
        }

        /// <summary>
        ///     Enhanced path tracing algorithm with better gap handling and more consistent contour following
        /// </summary>
        private static List<Point> TracePath(Image<Rgba32> image, Point start, HashSet<Point> targetPixels)
        {
            var path = new List<Point> { start };
            var visited = new HashSet<Point> { start };
            var current = start;
            var previous = new Point(-1, -1);

            // Maximum iterations to prevent infinite loops
            var maxIterations = targetPixels.Count * 2;
            var iteration = 0;
            var closedLoop = false;

            while (iteration < maxIterations)
            {
                // Get neighbors of the current point
                var neighbors = GetNeighbors(current, targetPixels, visited);

                // Check if we've returned to near the start (closed loop)
                if (path.Count > 20 && IsNearPoint(current, start, 5))
                {
                    logger.Log("Closed loop detected, path complete");
                    closedLoop = true;
                    break;
                }

                // If no neighbors, handle gaps
                if (neighbors.Count == 0)
                {
                    var nearest = FindNearestUnvisited(current, targetPixels, visited, 30);
                    if (nearest.X != -1)
                    {
                        logger.Log(
                            $"Jumped gap to pixel at ({nearest.X}, {nearest.Y}), distance: {DistanceBetween(current, nearest):F1}px");
                        previous = current;
                        current = nearest;
                        path.Add(current);
                        visited.Add(current);
                    }
                    else
                    {
                        logger.LogWarning("No more unvisited pixels found, ending path");
                        break;
                    }
                }
                else
                {
                    // Choose the best neighbor to continue the path
                    var next = ChooseBestNeighbor(current, previous, neighbors);
                    previous = current;
                    current = next;
                    path.Add(current);
                    visited.Add(current);
                }

                iteration++;

                // Log progress occasionally
                if (iteration % 1000 == 0) logger.Log($"Traced {path.Count} points so far...");
            }

            if (iteration >= maxIterations)
                logger.LogWarning($"Path tracing reached maximum iterations ({maxIterations})");

            // Close the loop if needed
            if (!closedLoop && path.Count > 10)
                if (DistanceBetween(path[0], path[path.Count - 1]) > 10)
                {
                    logger.Log("Attempting to close incomplete loop");
                    path.Add(start); // Add the start point to form a loop
                }

            // Smooth and simplify the path
            if (path.Count > 5)
            {
                path = SmoothPath(path, 3);
                logger.Log($"Smoothed path to {path.Count} points");
            }

            return path;
        }

        /// <summary>
        ///     Gets neighboring points from the target pixels set that haven't been visited
        /// </summary>
        private static List<Point> GetNeighbors(Point current, HashSet<Point> targetPixels, HashSet<Point> visited)
        {
            var neighbors = new List<Point>();
            var searchRadius = 5; // Increased from 2

            // Search in a circle rather than a grid
            for (var dy = -searchRadius; dy <= searchRadius; dy++)
            for (var dx = -searchRadius; dx <= searchRadius; dx++)
            {
                if (dx == 0 && dy == 0) continue;

                // Only consider points within the circular radius
                if (Math.Sqrt(dx * dx + dy * dy) > searchRadius) continue;

                var nx = current.X + dx;
                var ny = current.Y + dy;
                var neighbor = new Point(nx, ny);

                if (targetPixels.Contains(neighbor) && !visited.Contains(neighbor))
                    neighbors.Add(neighbor);
            }

            return neighbors;
        }

        /// <summary>
        ///     Chooses the best neighbor to continue the path, prioritizing direction continuity
        /// </summary>
        private static Point ChooseBestNeighbor(Point current, Point previous, List<Point> neighbors)
        {
            if (neighbors.Count == 0) return current;
            if (neighbors.Count == 1) return neighbors[0];

            // Calculate the general direction of travel
            var dxPrev = previous.X == -1 ? 0 : current.X - previous.X;
            var dyPrev = previous.Y == -1 ? 0 : current.Y - previous.Y;

            var bestNeighbor = neighbors[0];
            var bestScore = double.MinValue;

            foreach (var neighbor in neighbors)
            {
                var dx = neighbor.X - current.X;
                var dy = neighbor.Y - current.Y;

                // Base score on alignment with previous direction
                double directionScore = 0;
                if (dxPrev != 0 || dyPrev != 0)
                {
                    // Dot product for direction alignment
                    directionScore = (dxPrev * dx + dyPrev * dy) /
                                     (Math.Sqrt(dxPrev * dxPrev + dyPrev * dyPrev) *
                                      Math.Sqrt(dx * dx + dy * dy));
                    directionScore *= 10; // Weight direction heavily
                }

                // Prefer closer neighbors
                var distanceScore = 3.0 / (Math.Sqrt(dx * dx + dy * dy) + 0.1);

                // Combined score
                var totalScore = directionScore + distanceScore;

                if (totalScore > bestScore)
                {
                    bestScore = totalScore;
                    bestNeighbor = neighbor;
                }
            }

            return bestNeighbor;
        }

        /// <summary>
        ///     Find the nearest unvisited route pixel within maxDistance
        /// </summary>
        private static Point FindNearestUnvisited(Point current, HashSet<Point> routePixels, HashSet<Point> visited,
            int maxDistance)
        {
            var nearest = new Point(-1, -1);
            var minDistance = double.MaxValue;

            // Search in expanding squares until we find something or reach max distance
            for (var distance = 1; distance <= maxDistance; distance++)
            {
                var foundSomething = false;

                for (var dx = -distance; dx <= distance; dx++)
                for (var dy = -distance; dy <= distance; dy++)
                {
                    // Only check the perimeter of the square
                    if (Math.Abs(dx) != distance && Math.Abs(dy) != distance) continue;

                    var pixel = new Point(current.X + dx, current.Y + dy);

                    if (routePixels.Contains(pixel) && !visited.Contains(pixel))
                    {
                        var dist = Math.Sqrt(dx * dx + dy * dy);
                        if (dist < minDistance)
                        {
                            minDistance = dist;
                            nearest = pixel;
                            foundSomething = true;
                        }
                    }
                }

                // If we found something at this distance, no need to search further
                if (foundSomething) break;
            }

            return nearest;
        }

        /// <summary>
        ///     Check if two points are within distance of each other
        /// </summary>
        private static bool IsNearPoint(Point p1, Point p2, int distance)
        {
            double dx = p1.X - p2.X;
            double dy = p1.Y - p2.Y;
            return Math.Sqrt(dx * dx + dy * dy) <= distance;
        }

        /// <summary>
        ///     Helper method to calculate distance between points
        /// </summary>
        private static double DistanceBetween(Point p1, Point p2)
        {
            double dx = p1.X - p2.X;
            double dy = p1.Y - p2.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        ///     Smooth the path by averaging nearby points
        /// </summary>
        private static List<Point> SmoothPath(List<Point> path, int windowSize)
        {
            if (path.Count <= windowSize * 2 + 1) return path;

            var smoothed = new List<Point>();

            // Keep first point unchanged
            smoothed.Add(path[0]);

            // Smooth middle points
            for (var i = 1; i < path.Count - 1; i++)
            {
                int sumX = 0, sumY = 0;
                var count = 0;

                for (var j = Math.Max(0, i - windowSize); j <= Math.Min(path.Count - 1, i + windowSize); j++)
                {
                    sumX += path[j].X;
                    sumY += path[j].Y;
                    count++;
                }

                smoothed.Add(new Point(sumX / count, sumY / count));
            }

            // Keep last point unchanged
            smoothed.Add(path[path.Count - 1]);

            return smoothed;
        }

        /// <summary>
        ///     Smooth the path by using a Bezier curve
        /// </summary>
        private static List<Point> SmoothPathWithBezier(List<Point> path, int controlPointCount)
        {
            if (path.Count < 4) return path;

            var smoothed = new List<Point>();
            var segments = path.Count / 3;

            // Generate more points for smoother curve
            for (var i = 0; i < segments; i++)
            {
                var idx0 = i * 3 % path.Count;
                var idx1 = (idx0 + 1) % path.Count;
                var idx2 = (idx0 + 2) % path.Count;
                var idx3 = (idx0 + 3) % path.Count;

                var p0 = path[idx0];
                var p1 = path[idx1];
                var p2 = path[idx2];
                var p3 = path[idx3];

                // Generate points along the cubic Bezier curve
                for (float t = 0; t < 1.0f; t += 0.05f)
                {
                    var t2 = t * t;
                    var t3 = t2 * t;
                    var mt = 1 - t;
                    var mt2 = mt * mt;
                    var mt3 = mt2 * mt;

                    // Cubic Bezier formula
                    var x = (int)(mt3 * p0.X + 3 * mt2 * t * p1.X + 3 * mt * t2 * p2.X + t3 * p3.X);
                    var y = (int)(mt3 * p0.Y + 3 * mt2 * t * p1.Y + 3 * mt * t2 * p2.Y + t3 * p3.Y);

                    smoothed.Add(new Point(x, y));
                }
            }

            return smoothed;
        }

        /// <summary>
        ///     Detects and straightens line segments in a list of waypoints
        /// </summary>
        private static List<Point> StraightenLineSegments(List<Point> waypoints, double angleThreshold = 0.05,
            int minSegmentLength = 3)
        {
            if (waypoints.Count < minSegmentLength)
                return waypoints;

            var result = new List<Point>();
            var segmentStart = 0;
            var inStraightSegment = false;

            // Add first point
            result.Add(waypoints[0]);

            // Detect straight segments
            for (var i = 1; i < waypoints.Count - 1; i++)
            {
                // Calculate angle between vectors
                var v1 = new
                {
                    x = waypoints[i].X - waypoints[i - 1].X,
                    y = waypoints[i].Y - waypoints[i - 1].Y
                };
                var v2 = new
                {
                    x = waypoints[i + 1].X - waypoints[i].X,
                    y = waypoints[i + 1].Y - waypoints[i].Y
                };

                // Calculate magnitude of vectors
                var mag1 = Math.Sqrt(v1.x * v1.x + v1.y * v1.y);
                var mag2 = Math.Sqrt(v2.x * v2.x + v2.y * v2.y);

                // Avoid division by zero
                if (mag1 < 0.001 || mag2 < 0.001)
                {
                    result.Add(waypoints[i]);
                    continue;
                }

                // Calculate cosine of angle between vectors
                var cosAngle = (v1.x * v2.x + v1.y * v2.y) / (mag1 * mag2);
                cosAngle = Math.Min(1.0, Math.Max(-1.0, cosAngle)); // Clamp to [-1, 1]
                var angle = Math.Acos(cosAngle);

                // Check if we're in a straight segment
                if (angle < angleThreshold)
                {
                    if (!inStraightSegment)
                    {
                        segmentStart = i - 1;
                        inStraightSegment = true;
                    }
                }
                else
                {
                    // If we were in a straight segment, straighten it
                    if (inStraightSegment && i - segmentStart >= minSegmentLength)
                    {
                        // Remove any points we added from this segment
                        while (result.Count > segmentStart + 1)
                            result.RemoveAt(result.Count - 1);

                        // Create a perfect straight line between start and end of segment
                        var start = waypoints[segmentStart];
                        var end = waypoints[i];
                        result.Add(end);

                        logger.Log(
                            $"Straightened segment from ({start.X},{start.Y}) to ({end.X},{end.Y}) with {i - segmentStart} points");
                    }
                    else if (inStraightSegment)
                    {
                        // Not long enough to be considered a proper straight
                        for (var j = segmentStart + 1; j <= i; j++)
                            if (!result.Contains(waypoints[j]))
                                result.Add(waypoints[j]);
                    }
                    else
                    {
                        result.Add(waypoints[i]);
                    }

                    inStraightSegment = false;
                }
            }

            // Handle final segment
            if (inStraightSegment && waypoints.Count - 1 - segmentStart >= minSegmentLength)
            {
                // Remove any points we added from this segment
                while (result.Count > segmentStart + 1)
                    result.RemoveAt(result.Count - 1);

                // Add final point
                result.Add(waypoints[waypoints.Count - 1]);
            }
            else if (!result.Contains(waypoints[waypoints.Count - 1]))
            {
                // Add final point if not already added
                result.Add(waypoints[waypoints.Count - 1]);
            }

            return result;
        }

        /// <summary>
        ///     Checks if a pixel's color is within tolerance of the target color using RGB Euclidean distance
        /// </summary>
        private static bool IsTargetColor(Rgba32 pixel, Rgba32 target, int tolerance)
        {
            // Special case for red route - more lenient with red channel
            if (target.R > 200 && target.G < 50 && target.B < 50)
                return pixel.R > pixel.G + 50 && pixel.R > pixel.B + 50 && pixel.R > 150;

            // Special case for blue route
            if (target.B > 200 && target.R < 50 && target.G < 50)
                return pixel.B > pixel.R + 50 && pixel.B > pixel.G + 50 && pixel.B > 150;

            // Default RGB Euclidean distance check
            var dr = pixel.R - target.R;
            var dg = pixel.G - target.G;
            var db = pixel.B - target.B;
            return dr * dr + dg * dg + db * db < tolerance * tolerance;
        }

        /// <summary>
        ///     Calculates the signed area of the polygon formed by the path
        ///     Positive area = counter-clockwise order, Negative area = clockwise order
        /// </summary>
        private static double CalculateSignedArea(List<Point> path)
        {
            double area = 0;
            var n = path.Count;
            for (var i = 0; i < n; i++)
            {
                var j = (i + 1) % n;
                area += (double)path[i].X * path[j].Y - (double)path[j].X * path[i].Y;
            }

            return area / 2.0;
        }

        /// <summary>
        ///     Ensures the path is in clockwise order, reverses it if not
        /// </summary>
        private static void EnsureClockwiseOrder(ref List<Point> path)
        {
            // Calculate the signed area - positive means counter-clockwise
            var area = CalculateSignedArea(path);

            // In image coordinates, Y increases downward, so the sign is reversed
            // Positive area actually means clockwise in screen coordinates
            if (area < 0)
            {
                logger.Log("Path is counter-clockwise, reversing to clockwise");
                path.Reverse();
            }
            else
            {
                logger.Log("Path is already in clockwise order");
            }
        }

        /// <summary>
        ///     Improved waypoint sampling that produces more evenly spaced points
        /// </summary>
        private static List<Point> SampleWaypoints(List<Point> path, int sampleInterval)
        {
            var waypoints = new List<Point>();
            if (path.Count < 2) return path;

            // Always include the first point
            waypoints.Add(path[0]);

            // Calculate total path length for more even distribution
            double totalLength = 0;
            for (var i = 0; i < path.Count - 1; i++) totalLength += DistanceBetween(path[i], path[i + 1]);

            // Add the closed loop segment if needed
            if (path[0] != path[path.Count - 1]) totalLength += DistanceBetween(path[path.Count - 1], path[0]);

            // Determine number of points based on total length and sample interval
            var numPoints = Math.Max(10, (int)(totalLength / sampleInterval));
            var pointDistance = totalLength / numPoints;

            // Add evenly spaced points
            double distanceCovered = 0;
            var currentSegment = 0;
            double currentSegmentStart = 0;

            logger.Log(
                $"Sampling {numPoints} points at {pointDistance:F1}px intervals (total path length: {totalLength:F1}px)");

            for (var i = 1; i < numPoints; i++)
            {
                var targetDistance = i * pointDistance;

                // Find which segment contains the target distance
                while (true)
                {
                    var nextSegmentIndex = (currentSegment + 1) % path.Count;
                    var segmentLength = DistanceBetween(path[currentSegment], path[nextSegmentIndex]);
                    var segmentEnd = currentSegmentStart + segmentLength;

                    if (segmentEnd >= targetDistance || currentSegment == path.Count - 1)
                    {
                        // This segment contains our target point
                        var segmentPos = (targetDistance - currentSegmentStart) / segmentLength;
                        var x = (int)Math.Round(path[currentSegment].X +
                                                segmentPos * (path[nextSegmentIndex].X - path[currentSegment].X));
                        var y = (int)Math.Round(path[currentSegment].Y +
                                                segmentPos * (path[nextSegmentIndex].Y - path[currentSegment].Y));

                        waypoints.Add(new Point(x, y));
                        break;
                    }

                    // Move to next segment
                    currentSegmentStart = segmentEnd;
                    currentSegment = (currentSegment + 1) % path.Count;

                    // Prevent infinite loop
                    if (currentSegment == 0) break;
                }
            }

            return waypoints;
        }


        /// <summary>
        ///     Simple logger class for RouteExtractor
        /// </summary>
        private class Logger
        {
            private readonly bool consoleOutput;
            private readonly string filePath;

            public Logger(string filePath, bool consoleOutput)
            {
                this.filePath = filePath;
                this.consoleOutput = consoleOutput;

                try
                {
                    File.WriteAllText(filePath, $"=== RouteExtractor Log started at {DateTime.Now} ===\r\n");
                }
                catch
                {
                    /* Ignore if we can't write to file */
                }
            }

            public void Log(string message)
            {
                var logLine = $"{DateTime.Now:HH:mm:ss.fff} - {message}";

                if (consoleOutput) Console.WriteLine(logLine);

                try
                {
                    File.AppendAllText(filePath, logLine + Environment.NewLine);
                }
                catch
                {
                    /* Ignore if we can't write to file */
                }
            }

            public void LogWarning(string message)
            {
                Log($"WARNING: {message}");
            }

            public void LogError(string message)
            {
                Log($"ERROR: {message}");
            }

            public void LogException(Exception ex, string context)
            {
                LogError($"{context}: {ex.Message}");
            }
        }
    }
}
