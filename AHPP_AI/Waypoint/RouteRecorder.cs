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
        private readonly DebugUI? debugUI;
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

        public RouteRecorder(Logger logger, LFSLayout layout, DebugUI? debugUI, RouteLibrary routeLibrary, UI.MainUI mainUI)
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
                Metadata = PrepareMetadata(metadata),
                Nodes = new List<RoutePoint>()
            };

            lastPos = null;
            pointCount = 0;
            IsRecording = true;
            logger.Log($"Started route recording: {currentRoute.Metadata.Name} ({currentRoute.Metadata.Type})");
            debugUI?.UpdateRecordingButton(currentRoute.Metadata.Name, pointCount, true);
            mainUI?.UpdateRecordStatus(currentRoute.Metadata.Name, pointCount, true);
        }

        /// <summary>
        /// Start recording from an existing route by trimming after a selected node so new points extend the path.
        /// </summary>
        /// <param name="baseRoute">Existing route to extend.</param>
        /// <param name="startIndex">Index of the node to keep as the last preserved point.</param>
        public void StartFromExisting(RecordedRoute baseRoute, int startIndex)
        {
            if (baseRoute == null) throw new ArgumentNullException(nameof(baseRoute));
            if (IsRecording) Stop();

            var preparedMetadata = PrepareMetadata(baseRoute.Metadata ?? new RouteMetadata());
            var trimmedNodes = TrimNodes(baseRoute.Nodes, startIndex);

            baseRoute.Metadata = preparedMetadata;
            baseRoute.Nodes = trimmedNodes;

            currentRoute = baseRoute;
            pointCount = trimmedNodes.Count;
            lastPos = pointCount > 0
                ? Vec.FromMeters(trimmedNodes[pointCount - 1].X, trimmedNodes[pointCount - 1].Y,
                    trimmedNodes[pointCount - 1].Z)
                : (Vec?)null;
            IsRecording = true;

            logger.Log(
                $"Extending route {preparedMetadata.Name} from node {Math.Min(startIndex, Math.Max(0, pointCount - 1))} ({pointCount} nodes kept)");
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

        /// <summary>
        /// Normalize metadata for recording, filling defaults and current track/layout.
        /// </summary>
        private RouteMetadata PrepareMetadata(RouteMetadata? metadata)
        {
            var prepared = CloneMetadata(metadata ?? new RouteMetadata());
            if (string.IsNullOrWhiteSpace(prepared.Name)) prepared.Name = "route";

            if (prepared.Type == RouteType.Unknown)
                prepared.Type = routeLibrary.GuessRouteType(prepared.Name);
            if (prepared.Type == RouteType.MainLoop || prepared.Type == RouteType.AlternateMain)
                prepared.IsLoop = true;
            if (!prepared.DefaultSpeedLimit.HasValue) prepared.DefaultSpeedLimit = 60;

            prepared.Track = routeLibrary.TrackCode;
            prepared.Layout = routeLibrary.LayoutName;
            return prepared;
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
                DefaultSpeedLimit = metadata.DefaultSpeedLimit,
                AiTargetCount = metadata.AiTargetCount,
                AiTargetPercent = metadata.AiTargetPercent,
                AiWeight = metadata.AiWeight,
                AiEnabled = metadata.AiEnabled
            };
        }

        private List<RoutePoint> TrimNodes(List<RoutePoint>? nodes, int startIndex)
        {
            var trimmed = new List<RoutePoint>();
            if (nodes == null || nodes.Count == 0) return trimmed;

            var endIndex = Math.Max(0, Math.Min(startIndex, nodes.Count - 1));
            for (var i = 0; i <= endIndex; i++)
            {
                trimmed.Add(CloneRoutePoint(nodes[i]));
            }

            return trimmed;
        }

        private static RoutePoint CloneRoutePoint(RoutePoint source)
        {
            return new RoutePoint
            {
                X = source.X,
                Y = source.Y,
                Z = source.Z,
                Speed = source.Speed,
                SpeedLimit = source.SpeedLimit,
                Throttle = source.Throttle,
                Brake = source.Brake,
                Steering = source.Steering,
                Heading = source.Heading,
                Note = source.Note
            };
        }
    }
}
