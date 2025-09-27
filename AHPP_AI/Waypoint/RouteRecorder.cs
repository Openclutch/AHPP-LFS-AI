using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using AHPP_AI.Debug;
using AHPP_AI.Util;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    /// Helper class to record player driven routes and save them as JSON files.
    /// Each point stores position in meters and the speed at that moment.
    /// </summary>
    public class RouteRecorder
    {
        private readonly Logger logger;
        private readonly LFSLayout layout;
        private readonly DebugUI debugUI;
        private List<RoutePoint> currentRoute = new List<RoutePoint>();
        private string currentName = string.Empty;
        private string currentType = "main";
        private double captureInterval = 1.0;
        private Vec? lastPos = null;
        private int pointCount = 0;
        public bool IsRecording { get; private set; }
        public int PointCount => pointCount;

        public void SetInterval(double meters)
        {
            captureInterval = Math.Max(0.1, meters);
        }

        public RouteRecorder(Logger logger, LFSLayout layout, DebugUI debugUI)
        {
            this.logger = logger;
            this.layout = layout;
            this.debugUI = debugUI;
        }

        /// <summary>
        /// Start recording a new route with the given name and type.
        /// </summary>
        public void Start(string name, string type = "main")
        {
            if (IsRecording) Stop();
            currentRoute = new List<RoutePoint>();
            currentName = name;
            currentType = type;
            lastPos = null;
            pointCount = 0;
            IsRecording = true;
            logger.Log($"Started route recording: {name}");
            debugUI?.UpdateRecordingButton(name, pointCount, true);
        }

        /// <summary>
        /// Stop recording and save the route to JSON.
        /// </summary>
        public void Stop()
        {
            if (!IsRecording) return;
            IsRecording = false;
            var file = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, $"{currentName}.json");
            try
            {
                var options = new JsonSerializerOptions { WriteIndented = true };
                var saved = new SavedRoute
                {
                    Name = currentName,
                    Type = currentType,
                    Nodes = currentRoute
                };
                File.WriteAllText(file, JsonSerializer.Serialize(saved, options));
                logger.Log($"Saved route {currentName} with {currentRoute.Count} points to {file}");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Failed to save route {currentName}");
            }
            debugUI?.UpdateRecordingButton(currentName, pointCount, false);
        }

        /// <summary>
        /// Add a point if recording is active.
        /// </summary>
        public void AddPoint(Vec pos, double speedKmh, float throttle, float brake, float steering, int heading, byte plid)
        {
            if (!IsRecording) return;

            if (lastPos != null && Vec.Distance(lastPos.Value, pos) < captureInterval)
                return;

            currentRoute.Add(new RoutePoint
            {
                X = pos.X / 65536.0,
                Y = pos.Y / 65536.0,
                Z = pos.Z / 65536.0,
                Speed = speedKmh,
                Throttle = throttle,
                Brake = brake,
                Steering = steering,
                Heading = heading
            });

            pointCount++;
            lastPos = pos;

            layout?.PlaceArrowMarker(plid, (float)(pos.X / 65536.0), (float)(pos.Y / 65536.0), (byte)(heading / 4096));
            debugUI?.UpdateRecordingButton(currentName, pointCount, true);
        }
    }

    /// <summary>
    /// Single recorded position for a route.
    /// </summary>
    public struct RoutePoint
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Speed { get; set; }
        public float Throttle { get; set; }
        public float Brake { get; set; }
        public float Steering { get; set; }
        public int Heading { get; set; }
    }

    /// <summary>
    /// Metadata wrapper for saved routes.
    /// </summary>
    public class SavedRoute
    {
        public string Name { get; set; }
        public string Type { get; set; }
        public List<RoutePoint> Nodes { get; set; }
    }
}
