using System;
using System.Collections.Generic;
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
        private readonly RouteLibrary routeLibrary;
        private readonly UI.MainUI mainUI;
        private RecordedRoute currentRoute = new RecordedRoute();
        private double captureInterval = 1.0;
        private Vec? lastPos = null;
        private int pointCount = 0;
        public bool IsRecording { get; private set; }
        public int PointCount => pointCount;

        public void SetInterval(double meters)
        {
            captureInterval = Math.Max(0.1, meters);
        }

        public RouteRecorder(Logger logger, LFSLayout layout, DebugUI debugUI, RouteLibrary routeLibrary, UI.MainUI mainUI)
        {
            this.logger = logger;
            this.layout = layout;
            this.debugUI = debugUI;
            this.routeLibrary = routeLibrary;
            this.mainUI = mainUI;
        }

        /// <summary>
        /// Start recording a new route with the supplied metadata.
        /// </summary>
        public void Start(RouteMetadata metadata)
        {
            if (metadata == null) throw new ArgumentNullException(nameof(metadata));
            if (IsRecording) Stop();

            currentRoute = new RecordedRoute
            {
                Metadata = CloneMetadata(metadata),
                Nodes = new List<RoutePoint>()
            };

            if (currentRoute.Metadata.Type == RouteType.Unknown)
                currentRoute.Metadata.Type = routeLibrary.GuessRouteType(currentRoute.Metadata.Name);
            if (currentRoute.Metadata.Type == RouteType.MainLoop) currentRoute.Metadata.IsLoop = true;
            if (!currentRoute.Metadata.DefaultSpeedLimit.HasValue) currentRoute.Metadata.DefaultSpeedLimit = 60;

            lastPos = null;
            pointCount = 0;
            IsRecording = true;
            logger.Log($"Started route recording: {currentRoute.Metadata.Name} ({currentRoute.Metadata.Type})");
            debugUI?.UpdateRecordingButton(currentRoute.Metadata.Name, pointCount, true);
            mainUI?.UpdateRecordStatus(currentRoute.Metadata.Name, pointCount, true);
        }

        /// <summary>
        /// Start recording with just a name and optional type.
        /// </summary>
        public void Start(string name, RouteType type = RouteType.Unknown)
        {
            var metadata = new RouteMetadata
            {
                Name = name,
                Type = type
            };
            Start(metadata);
        }

        /// <summary>
        /// Stop recording and save the route to JSON.
        /// </summary>
        public void Stop()
        {
            if (!IsRecording) return;
            IsRecording = false;
            try
            {
                routeLibrary.Save(currentRoute);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Failed to save route {currentRoute.Metadata.Name}");
            }
            debugUI?.UpdateRecordingButton(currentRoute.Metadata.Name, pointCount, false);
            mainUI?.UpdateRecordStatus(currentRoute.Metadata.Name, pointCount, false);
        }

        /// <summary>
        /// Add a point if recording is active.
        /// </summary>
        public void AddPoint(Vec pos, double speedKmh, float throttle, float brake, float steering, int heading, byte plid)
        {
            if (!IsRecording) return;

            if (lastPos != null && Vec.Distance(lastPos.Value, pos) < captureInterval)
                return;

            currentRoute.Nodes.Add(new RoutePoint
            {
                X = pos.X / 65536.0,
                Y = pos.Y / 65536.0,
                Z = pos.Z / 65536.0,
                Speed = speedKmh,
                SpeedLimit = speedKmh,
                Throttle = throttle,
                Brake = brake,
                Steering = steering,
                Heading = heading
            });

            pointCount++;
            lastPos = pos;

            // Convert 0-65535 car heading to 0-255 object heading so the Chalk Ahead arrow matches the car
            // and flip 180 degrees because the chalk asset points opposite the car heading by default.
            var chalkHeading = (byte)((heading / 256 + 128) % 256);
            var xMeters = (float)(pos.X / 65536.0);
            var yMeters = (float)(pos.Y / 65536.0);
            var zMeters = (float)(pos.Z / 65536.0);
            layout?.PlaceArrowMarker(plid, xMeters, yMeters, zMeters, chalkHeading);
            debugUI?.UpdateRecordingButton(currentRoute.Metadata.Name, pointCount, true);
            mainUI?.UpdateRecordStatus(currentRoute.Metadata.Name, pointCount, true);
        }

        private static RouteMetadata CloneMetadata(RouteMetadata metadata)
        {
            return new RouteMetadata
            {
                Name = metadata.Name,
                Type = metadata.Type,
                Description = metadata.Description,
                Track = metadata.Track,
                Layout = metadata.Layout,
                IsLoop = metadata.IsLoop,
                AttachMainIndex = metadata.AttachMainIndex,
                RejoinMainIndex = metadata.RejoinMainIndex,
                DefaultSpeedLimit = metadata.DefaultSpeedLimit
            };
        }
    }
}
