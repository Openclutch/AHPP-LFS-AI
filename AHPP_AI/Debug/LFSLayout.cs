using System;
using System.Collections.Generic;
using System.Linq;
using AHPP_AI.Util;
using AHPP_AI.Waypoint;
using InSimDotNet.Helpers;
using InSimDotNet.Packets;
using InSimClient = InSimDotNet.InSimClient;

namespace AHPP_AI.Debug
{
    /// <summary>
    ///     Handles visualization of objects (waypoints, etc.) in the game
    /// </summary>
    public class LFSLayout
    {
        // Object IDs from ObjectHelper
        private const byte CHALK_AHEAD = 6; // Chalk arrow that points in the heading direction
        private const byte CONE_RED = 21; // Cone Red
        private const byte CHALK_COLOR_WHITE = 0; // Flags: white chalk
        private const byte CHALK_COLOR_RED = 1; // Flags: red chalk
        private const byte CONE_BLUE = 24; // Cone Blue
        private const byte CONE_GREEN = 26; // Cone Green

        private const float WAYPOINT_RADIUS = 2.5f; // Radius of waypoint visualization in meters
        private const int CONES_PER_CIRCLE = 0; // Number of cones to place around each waypoint
        private const int LFS_MIN_COORD = -16000; // Safe bounds for AXM coordinates (1/16th meters)
        private const int LFS_MAX_COORD = 16000;

        public const int MaxVisibleWaypoints = 3600; // Maximum number of waypoints to visualize

        // Track active waypoint marker per PLID to remove old ones
        private readonly Dictionary<byte, ObjectInfo> activeWaypointMarkerIds = new Dictionary<byte, ObjectInfo>();
        private readonly Dictionary<byte, (int X, int Y)> lastActiveWaypointPos = new Dictionary<byte, (int X, int Y)>();

        private readonly InSimClient insim;
        public readonly List<ObjectInfo> layoutObjects = new List<ObjectInfo>();
        private readonly Logger logger;
        private readonly Dictionary<byte, List<ObjectInfo>> placedObjectsByPlid = new Dictionary<byte, List<ObjectInfo>>();
        private readonly HashSet<Tuple<float, float>> waypointPositions = new HashSet<Tuple<float, float>>();

        /// <summary>
        /// Fired when the layout editor selection changes (Shift+U selection updates).
        /// </summary>
        public event Action<ObjectInfo> LayoutSelectionChanged;

        /// <summary>
        /// Fired when a layout object is moved in the editor (PMO_MOVE_MODIFY).
        /// </summary>
        public event Action<ObjectInfo> LayoutObjectMoved;

        /// <summary>
        ///     Initializes a new instance of the LFSLayout
        /// </summary>
        /// <param name="logger">Logger for debug information</param>
        /// <param name="insim">InSim connection</param>
        public LFSLayout(Logger logger, InSimClient insim)
        {
            this.logger = logger;
            this.insim = insim;
        }

        public bool LayoutObjectsRequested { get; set; }
        public bool WaypointsVisualized { get; set; }

        /// <summary>
        ///     Handles AXM packets (layout objects)
        /// </summary>
        public void OnAXM(IS_AXM axm)
        {
            logger.Log($"AXM received: PMOAction={axm.PMOAction}, NumO={axm.NumO}");

            if (axm.PMOAction == ActionFlags.PMO_SELECTION || axm.PMOAction == ActionFlags.PMO_TTC_SEL)
            {
                NotifyLayoutSelection(axm);
                return;
            }

            if (axm.PMOFlags.HasFlag(PMOFlags.PMO_MOVE_MODIFY) && axm.PMOAction == ActionFlags.PMO_ADD_OBJECTS)
            {
                NotifyLayoutMove(axm);
            }

            if (axm.PMOAction == ActionFlags.PMO_LOADING_FILE ||
                axm.PMOAction == ActionFlags.PMO_ADD_OBJECTS ||
                axm.PMOAction == ActionFlags.PMO_TINY_AXM)
                foreach (var obj in axm.Info)
                {
                    // Convert to meters for easier understanding
                    var xMeters = obj.X / 16.0f;
                    var yMeters = obj.Y / 16.0f;
                    var zMeters = obj.Zbyte / 4.0f;
                    var objName = ObjectHelper.GetObjName(obj.Index) ?? $"Unknown ({obj.Index})";

                    //logger.Log(
                    //$"Object: {objName}, Index={obj.Index}, X={xMeters:F2}m, Y={yMeters:F2}m, Z={zMeters:F2}m, Heading={obj.Heading}, Flags={obj.Flags}");
                    layoutObjects.Add(obj);
                }
        }

        /// <summary>
        /// Dispatch layout selection changes to listeners.
        /// </summary>
        private void NotifyLayoutSelection(IS_AXM axm)
        {
            if (LayoutSelectionChanged == null) return;

            if (axm.Info == null || axm.Info.Count == 0)
            {
                LayoutSelectionChanged.Invoke(null);
                return;
            }

            LayoutSelectionChanged.Invoke(axm.Info[0]);
        }

        /// <summary>
        /// Dispatch layout move events to listeners when a layout object is repositioned.
        /// </summary>
        private void NotifyLayoutMove(IS_AXM axm)
        {
            if (LayoutObjectMoved == null) return;
            if (axm.Info == null || axm.Info.Count == 0) return;

            LayoutObjectMoved.Invoke(axm.Info[0]);
        }

        /// <summary>
        ///     Visualize waypoints for a specific player
        /// </summary>
        /// <param name="plid">Player ID</param>
        /// <param name="aiPaths">Dictionary of paths by player ID</param>
        public void VisualizeWaypoints(byte plid, Dictionary<byte, List<Util.Waypoint>> aiPaths)
        {
            // First, check if the dictionary contains the key
            if (!aiPaths.ContainsKey(plid))
            {
                logger.LogWarning($"Cannot visualize waypoints for PLID {plid}: no path available in dictionary");
                return;
            }

            // Then check if the path exists and has waypoints
            if (aiPaths[plid] == null || aiPaths[plid].Count == 0)
            {
                logger.LogWarning($"Cannot visualize waypoints for PLID {plid}: path is empty");
                return;
            }

            // Clear previous visualizations for this player
            ClearPlayerVisualizations(plid);
            waypointPositions.Clear();

            logger.Log($"Visualizing waypoints for PLID {plid} with {ObjectHelper.GetObjName(CHALK_AHEAD)}");

            // First visualize coordinate system for reference
            VisualizeCoordinateSystem(plid);

            var path = aiPaths[plid];
            var waypointCount = path.Count;

            // If too many waypoints, sample them to avoid clutter
            var step = Math.Max(1, waypointCount / MaxVisibleWaypoints);

            // Start with current waypoint and visualize forward
            var startIdx = 0;
            var placedCount = 0;

            for (var i = 0; i < waypointCount && placedCount < MaxVisibleWaypoints; i += step)
            {
                var idx = (startIdx + i) % waypointCount;
                var waypoint = path[idx];
                var nextWaypoint = path[(idx + 1) % waypointCount];

                // Estimate heading to the next waypoint so arrows point along the route direction.
                var dx = (nextWaypoint.Position.X - waypoint.Position.X) / 65536.0;
                var dy = (nextWaypoint.Position.Y - waypoint.Position.Y) / 65536.0;
                var desiredHeading = dx == 0 && dy == 0
                    ? 0
                    : CoordinateUtils.CalculateHeadingToTarget(dx, dy);
                var arrowHeading = (byte)((desiredHeading / 256 + 128) % 256);

                // Convert LFS coordinates (65536 units/meter) to meters
                var centerX = waypoint.Position.X / 65536.0f;
                var centerY = waypoint.Position.Y / 65536.0f;

                // Check if we already have cones at this position (using tolerance)
                var positionKey = Tuple.Create(
                    (float)Math.Round(centerX, 1),
                    (float)Math.Round(centerY, 1)
                );

                if (waypointPositions.Contains(positionKey)) continue;

                // Place cones in a circle around the waypoint
                PlaceWaypointCircle(plid, centerX, centerY, waypoint.RouteIndex, arrowHeading);
                waypointPositions.Add(positionKey);
                placedCount++;

                // Log every few placed waypoints
                if (placedCount % 5 == 0 || placedCount == 1 || placedCount == MaxVisibleWaypoints)
                    logger.Log(
                        $"Placed waypoint visualization {placedCount} at X={centerX:F2}, Y={centerY:F2}, Index={idx}");
            }

            WaypointsVisualized = true;
            insim.Send(new IS_MST { Msg = $"Visualized {placedCount} waypoints with cones" });
        }

        /// <summary>
        /// Visualize a recorded route with configurable sampling to keep layout objects within limits.
        /// </summary>
        public void VisualizeRecordedRoute(byte plid, RecordedRoute route, int detailStep, bool clearExisting = true)
        {
            if (route == null || route.Nodes == null || route.Nodes.Count == 0)
            {
                logger.LogWarning("No route data available to visualize");
                return;
            }

            if (clearExisting)
            {
                ClearPlayerVisualizations(plid);
            }

            // Reset per-route duplicate tracking so multiple routes can overlay while each still avoids self-duplicates.
            waypointPositions.Clear();

            var nodes = route.Nodes;
            logger.Log(
                $"Visualizing recorded route {route.Metadata.Name} ({route.Metadata.Type}) for editing with {nodes.Count} nodes");

            var count = 0;
            var step = Math.Max(1, detailStep);
            var requiredStep = Math.Max(1, (int)Math.Ceiling(nodes.Count / (double)MaxVisibleWaypoints));
            var effectiveStep = Math.Max(step, requiredStep);
            // Default marker uses Chalk Ahead; use red chalk for pit entry routes via flags.
            var markerType = CHALK_AHEAD;
            var markerFlags = route.Metadata.Type == RouteType.PitEntry ? CHALK_COLOR_RED : CHALK_COLOR_WHITE;
            for (var i = 0; i < nodes.Count && count < MaxVisibleWaypoints; i += effectiveStep)
            {
                var heading = CalculateHeadingForNode(nodes, i, route.Metadata.IsLoop);
                PlaceEditableWaypoint(plid, nodes[i], heading, markerType, markerFlags);
                count++;
            }

            WaypointsVisualized = true;
            insim.Send(new IS_MST { Msg = $"Visualized {count} nodes for {route.Metadata.Name}" });
        }

        /// <summary>
        ///     Place objects to visualize a waypoint with an optional heading for arrows
        /// </summary>
        private void PlaceWaypointCircle(byte plid, float centerX, float centerY, int routeIndex, byte heading = 0)
        {
            // Ensure we have storage for this player's objects
            if (!placedObjectsByPlid.ContainsKey(plid)) placedObjectsByPlid[plid] = new List<ObjectInfo>();

            // Create a key for this position to avoid duplicates
            var positionKey = Tuple.Create(
                (float)Math.Round(centerX, 1),
                (float)Math.Round(centerY, 1)
            );

            // Skip if we already have cones at this position
            if (waypointPositions.Contains(positionKey)) return;

            logger.Log($"Placing waypoint circle at X={centerX:F2}, Y={centerY:F2}, Index={routeIndex}");

            try
            {
                // Place an arrow at the exact waypoint position for precise visualization
                var chalkObj = PlaceObject(plid, CHALK_AHEAD, centerX, centerY, 0.5f, heading);
                if (chalkObj != null) placedObjectsByPlid[plid].Add(chalkObj);

                // Mark this position as having cones
                waypointPositions.Add(positionKey);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"Error placing waypoint circle at X={centerX}, Y={centerY}");
            }
        }

        /// <summary>
        /// Place a chalk arrow marker for a recorded route node to support manual editing.
        /// </summary>
        private void PlaceEditableWaypoint(byte plid, RoutePoint node, byte heading, byte objectType,
            byte flags = 0)
        {
            if (!placedObjectsByPlid.ContainsKey(plid)) placedObjectsByPlid[plid] = new List<ObjectInfo>();

            var xMeters = (float)node.X;
            var yMeters = (float)node.Y;
            var zMeters = (float)node.Z;

            var marker = PlaceObject(plid, objectType, xMeters, yMeters, zMeters, heading, flags);
            if (marker != null) placedObjectsByPlid[plid].Add(marker);
        }

        /// <summary>
        /// Calculate an arrow heading toward the next recorded node so arrows show flow direction.
        /// </summary>
        private byte CalculateHeadingForNode(IReadOnlyList<RoutePoint> nodes, int index, bool isLoop)
        {
            if (nodes == null || nodes.Count == 0) return 0;

            var current = nodes[index];
            var nextIndex = index + 1;
            if (nextIndex >= nodes.Count)
            {
                if (!isLoop) return 0;
                nextIndex = 0;
            }

            var next = nodes[nextIndex];
            var dx = next.X - current.X;
            var dy = next.Y - current.Y;
            if (Math.Abs(dx) < 0.0001 && Math.Abs(dy) < 0.0001) return 0;

            var desiredHeading = CoordinateUtils.CalculateHeadingToTarget(dx, dy);
            return (byte)((desiredHeading / 256 + 128) % 256);
        }

        /// <summary>
        ///     Visualize the coordinate system to help with debugging
        /// </summary>
        public void VisualizeCoordinateSystem(byte plid)
        {
            try
            {
                // Place markers for X and Y axes to help visualize the coordinate system
                // X axis (red cones)
                for (var i = 0; i < 5; i++)
                {
                    var xPos = i * 5.0f; // 5-meter intervals
                    var yPos = 0.0f;
                    var coneObj = PlaceObject(plid, CONE_RED, xPos, yPos, 0.5f);
                    if (coneObj != null && placedObjectsByPlid.ContainsKey(plid)) placedObjectsByPlid[plid].Add(coneObj);
                }

                // Y axis (blue cones)
                for (var i = 0; i < 5; i++)
                {
                    var xPos = 0.0f;
                    var yPos = i * 5.0f; // 5-meter intervals
                    var coneObj = PlaceObject(plid, CONE_BLUE, xPos, yPos, 0.5f);
                    if (coneObj != null && placedObjectsByPlid.ContainsKey(plid)) placedObjectsByPlid[plid].Add(coneObj);
                }

                // Origin (green cone)
                var originObj = PlaceObject(plid, CONE_GREEN, 0.0f, 0.0f, 0.5f);
                if (originObj != null && placedObjectsByPlid.ContainsKey(plid)) placedObjectsByPlid[plid].Add(originObj);

                logger.Log("Coordinate system visualization placed");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Error placing coordinate system visualization");
            }
        }

        /// <summary>
        ///     Place an object in the game world
        /// </summary>
        private ObjectInfo PlaceObject(byte plid, byte objectType, float xMeters, float yMeters, float zMeters,
            byte heading = 0, byte flags = 0)
        {
            try
            {
                var roundedX = (int)Math.Round(xMeters * 16f);
                var roundedY = (int)Math.Round(yMeters * 16f);
                var roundedZ = (int)Math.Round(zMeters * 4f);
                // Clamp to safe layout bounds to avoid invalid position errors.
                var clampedX = Math.Max(LFS_MIN_COORD, Math.Min(LFS_MAX_COORD, roundedX));
                var clampedY = Math.Max(LFS_MIN_COORD, Math.Min(LFS_MAX_COORD, roundedY));
                var objX = (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, clampedX));
                var objY = (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, clampedY));
                var objZ = (byte)Math.Max(byte.MinValue, Math.Min(byte.MaxValue, roundedZ));

                logger.Log($"Placing object: {ObjectHelper.GetObjName(objectType)} (Type={objectType}), " +
                           $"Meters=({xMeters:F2},{yMeters:F2},{zMeters:F2}), Heading={heading}, Flags={flags}, LYT=({objX},{objY},{objZ})");

                var objInfo = new ObjectInfo
                {
                    Index = objectType, // ✅ Use proper object type
                    X = objX,
                    Y = objY,
                    Zbyte = objZ,
                    Flags = flags,
                    Heading = heading
                };

                layoutObjects.Add(objInfo);

                var axmPacket = new IS_AXM
                {
                    PMOAction = ActionFlags.PMO_ADD_OBJECTS
                };

                axmPacket.Info.Add(objInfo);
                insim.Send(axmPacket);

                return objInfo;
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Error placing object");
                return null;
            }
        }

        /// <summary>
        /// Place a Chalk Ahead arrow marker aligned to the supplied heading.
        /// </summary>
        public void PlaceArrowMarker(byte plid, float xMeters, float yMeters, float zMeters, byte heading)
        {
            PlaceObject(plid, CHALK_AHEAD, xMeters, yMeters, zMeters, heading);
        }


        /// <summary>
        ///     Show the active waypoint for each AI
        /// </summary>
        public void VisualizeActiveWaypoint(byte plid, Util.Waypoint waypoint)
        {
            try
            {
                // Skip if the active waypoint hasn't changed to avoid spamming identical objects.
                var currentPos = (waypoint.Position.X, waypoint.Position.Y);
                if (lastActiveWaypointPos.TryGetValue(plid, out var lastPos) &&
                    lastPos.X == currentPos.X && lastPos.Y == currentPos.Y)
                    return;

                // Remove the previous active waypoint marker if it exists
                if (activeWaypointMarkerIds.ContainsKey(plid))
                {
                    var previousMarker = activeWaypointMarkerIds[plid];

                    var objectToDelete = previousMarker;

                    var deletePacket = new IS_AXM
                    {
                        PMOAction = ActionFlags.PMO_DEL_OBJECTS
                    };
                    deletePacket.Info.Add(objectToDelete);

                    insim.Send(deletePacket);

                    logger.Log($"Removed previous active waypoint marker at X={previousMarker.X/16f:F2}, Y={previousMarker.Y/16f:F2} for PLID={plid}");
                }

                // Convert waypoint position to meters
                var xMeters = waypoint.Position.X / 65536.0f;
                var yMeters = waypoint.Position.Y / 65536.0f;

                logger.Log($"Placing active waypoint marker at X={xMeters:F2}, Y={yMeters:F2}");

                // Place a red cone at the new waypoint position
                var newMarker = PlaceObject(plid, 21 /* CONE_RED */, xMeters, yMeters, 0.5f);

                if (newMarker != null)
                {
                    activeWaypointMarkerIds[plid] = newMarker;
                    if (placedObjectsByPlid.ContainsKey(plid))
                        placedObjectsByPlid[plid].Add(newMarker);
                }

                lastActiveWaypointPos[plid] = currentPos;
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Error placing active waypoint marker");
            }
        }


        /// <summary>
        ///     Clear visualizations for a specific player
        /// </summary>
        private void ClearPlayerVisualizations(byte plid)
        {
            if (!placedObjectsByPlid.ContainsKey(plid) || placedObjectsByPlid[plid].Count == 0) return;

            var objectsToClear = placedObjectsByPlid[plid];
            logger.Log($"Clearing {objectsToClear.Count} objects for PLID {plid}");

            try
            {
                // Create a list of objects to delete
                var objectsToDelete = new List<ObjectInfo>();

                foreach (var obj in objectsToClear)
                {
                    objectsToDelete.Add(obj);

                    // Remove from layout objects list
                    layoutObjects.RemoveAll(o =>
                        o.X == obj.X && o.Y == obj.Y && o.Zbyte == obj.Zbyte && o.Index == obj.Index &&
                        o.Heading == obj.Heading);
                }

                // Delete objects in batches to avoid packet size issues (AXM supports up to 60 per packet)
                const int batchSize = 60;
                for (var i = 0; i < objectsToDelete.Count; i += batchSize)
                {
                    // Create a new AXM packet for deletion
                    var axmPacket = new IS_AXM
                    {
                        PMOAction = ActionFlags.PMO_DEL_OBJECTS
                    };

                    // Get the batch of objects to delete
                    var count = Math.Min(batchSize, objectsToDelete.Count - i);
                    for (var j = 0; j < count; j++) axmPacket.Info.Add(objectsToDelete[i + j]);

                    // Send the deletion packet
                    insim.Send(axmPacket);
                }

                // Clear the list
                placedObjectsByPlid[plid].Clear();
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Error clearing player visualizations");
            }
        }

        /// <summary>
        ///     Clear all visualizations for all players
        /// </summary>
        public void ClearAllVisualizations()
        {
            try
            {
                // Get all players with visualizations
                var plids = placedObjectsByPlid.Keys.ToList();

                foreach (var plid in plids) ClearPlayerVisualizations(plid);

                waypointPositions.Clear();
                activeWaypointMarkerIds.Clear();
                lastActiveWaypointPos.Clear();
                logger.Log("All waypoint visualizations cleared");
            }
            catch (Exception ex)
            {
                logger.LogException(ex, "Error clearing all visualizations");
            }
        }
    }
}
