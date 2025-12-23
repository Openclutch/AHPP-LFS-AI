using System;
using System.Collections.Generic;
using AHPP_AI.Debug;
using AHPP_AI.Util;
using InSimDotNet.Packets;
using InSimClient = InSimDotNet.InSimClient;

namespace AHPP_AI.UI
{
    /// <summary>
    /// High level UI for controlling recording and AI management.
    /// Provides main panel buttons and AI list.
    /// </summary>
    public class MainUI
    {
        private readonly InSimClient insim;
        private readonly Logger logger;

        private const byte LEFT_COL = 5;
        private const byte RIGHT_COL = 50;
        private const byte AI_LIST_COL = 95; // Separate column for spawned AI buttons to avoid overlap
        private const byte ROW_HEIGHT = 5;
        private const byte BTN_W = 20;
        private const byte SELECT_BTN_W = 20;
        private const byte SELECT_BTN_SPACING = 2;
        private const byte VISUAL_ROUTE_START_ROW = 100;
        private const byte VISUAL_ROUTE_COLUMNS = 2;
        private const byte VISUAL_DETAIL_ROW = 160;
        private const byte DETAIL_BTN_W = 6;
        private const byte DETAIL_LABEL_W = 12;
        private const byte DETAIL_BTN_SPACING = 1;
        private const byte SELECT_ROW = 170;
        private const byte RECORD_ROW = 180;
        private const byte ROUTE_NAME_ROW = 160;
        private const byte REMOVE_BTN_W = 6;

        public const byte AddAiDialogId = 150;
        public const byte SpeedInputId = 151;
        public const byte RecordingIntervalId = 152;
        public const byte RouteNameInputId = 153;
        public const byte VisualizationDetailMinusId = 90;
        public const byte VisualizationDetailPlusId = 91;
        public const byte VisualizationDetailLabelId = 92;

        private readonly Dictionary<byte, byte> aiListButtons = new Dictionary<byte, byte>(); // AI ID -> button ID
        private readonly Dictionary<byte, byte> aiRemoveButtons = new Dictionary<byte, byte>(); // remove button ID -> AI ID
        private readonly List<(byte id, string name)> routeButtons = new List<(byte id, string name)>();
        private readonly List<(byte id, string name)> visualizationRouteButtons = new List<(byte id, string name)>();
        private const byte ROUTE_BUTTON_START_ID = 11;
        private const byte ROUTE_BUTTON_MAX_COUNT = 8;
        private const byte VISUAL_ROUTE_BUTTON_START_ID = 30;

        private string selectedRoute = "main_loop";
        private string selectedVisualizationRoute = "main_loop";
        private int visualizationDetailStep = 2;
        private bool isRecording;
        private int recordedPoints;
        private bool uiInitialized;
        
        public MainUI(InSimClient insim, Logger logger)
        {
            this.insim = insim;
            this.logger = logger;
        }

        /// <summary>
        /// Draw the static panels.
        /// </summary>
        public void Show()
        {
            uiInitialized = true;

            byte row = 70;
            CreateButton(2, "Reload Routes", LEFT_COL, row); row += ROW_HEIGHT;
            CreateButton(5, "Toggle Layout", LEFT_COL, row); row += ROW_HEIGHT;
            CreateButton(6, "Reset Layout/AI", LEFT_COL, row);

            row = 70;
            CreateInputButton(AddAiDialogId, RIGHT_COL, row, "AI Count"); row += ROW_HEIGHT;
            CreateInputButton(SpeedInputId, RIGHT_COL, row, "AI Speed"); row += ROW_HEIGHT;
            CreateInputButton(RecordingIntervalId, RIGHT_COL, row, "Rec Meters"); row += ROW_HEIGHT;
            CreateInputButton(RouteNameInputId, RIGHT_COL, ROUTE_NAME_ROW, "Route Name", 24);
            CreateButton(105, "Start All AI", RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(103, "Stop All AI", RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(104, "Pit All AI", RIGHT_COL, row);

            RenderRecordingSelectors();
            RenderRecordButton();
            RenderVisualizationRouteSelectors();
            RenderVisualizationDetailControls();
        }

        /// <summary>
        /// Update the AI list on the right panel.
        /// </summary>
        public void UpdateAIList(IEnumerable<(byte id, string name)> ai)
        {
            byte row = (byte)(5 + ROW_HEIGHT * 4);
            foreach (var b in aiListButtons.Values) DeleteButton(b);
            foreach (var b in aiRemoveButtons.Keys) DeleteButton(b);
            aiListButtons.Clear();
            aiRemoveButtons.Clear();

            byte index = 0;
            foreach (var (id, name) in ai)
            {
                var labelId = (byte)(120 + index);
                var removeId = (byte)(180 + index);
                var top = (byte)(row + ROW_HEIGHT * index);
                var removeLeft = (byte)Math.Max(0, AI_LIST_COL - REMOVE_BTN_W - 2);

                CreateButton(removeId, "X", removeLeft, top);
                CreateButton(labelId, $"{name}", AI_LIST_COL, top);

                aiListButtons[id] = labelId;
                aiRemoveButtons[removeId] = id;
                index++;
            }
        }

        /// <summary>
        /// Map a remove button ClickID to the AI it controls.
        /// </summary>
        public bool TryGetAiForRemoveButton(byte clickId, out byte aiId)
        {
            return aiRemoveButtons.TryGetValue(clickId, out aiId);
        }

        /// <summary>
        /// Create a clickable button in the UI.
        /// </summary>
        private void CreateButton(byte id, string text, byte left, byte top, byte width = BTN_W, byte height = ROW_HEIGHT,
            bool clickable = true)
        {
            var btn = new IS_BTN
            {
                ReqI = 1,
                UCID = 0,
                ClickID = id,
                Inst = 0,
                BStyle = clickable ? ButtonStyles.ISB_DARK | ButtonStyles.ISB_CLICK : ButtonStyles.ISB_DARK,
                L = left,
                T = top,
                W = width,
                H = height,
                Text = text
            };
            insim.Send(btn);
        }

        /// <summary>
        /// Delete a button by its ID.
        /// </summary>
        private void DeleteButton(byte id)
        {
            var del = new IS_BFN
            {
                ReqI = 1,
                SubT = ButtonFunction.BFN_DEL_BTN,
                UCID = 0,
                ClickID = id,
                ClickMax = id
            };
            insim.Send(del);
        }

        /// <summary>
        /// Create a type-in text field style button for numeric input.
        /// </summary>
        private void CreateInputButton(byte id, byte left, byte top, string caption, byte typeIn = 3)
        {
            var btn = new IS_BTN
            {
                ReqI = 1,
                UCID = 0,
                ClickID = id,
                Inst = 0,
                BStyle = ButtonStyles.ISB_DARK | ButtonStyles.ISB_CLICK,
                TypeIn = typeIn,
                L = left,
                T = top,
                W = BTN_W,
                H = ROW_HEIGHT,
                // Show the caption on the button and as the dialog title so users can see what to type.
                Text = caption,
                Caption = caption
            };
            insim.Send(btn);
        }

        /// <summary>
        /// Render route selection buttons to choose which route to record.
        /// </summary>
        private void RenderRecordingSelectors()
        {
            if (!uiInitialized) return;

            if (routeButtons.Count == 0)
            {
                SetRouteOptions(new[] { "main_loop", "pit_entry" }, selectedRoute);
                return;
            }

            byte totalWidth = (byte)(routeButtons.Count * SELECT_BTN_W + (routeButtons.Count - 1) * SELECT_BTN_SPACING);
            var startLeft = (byte)Math.Max(0, (200 - totalWidth) / 2);

            for (var i = 0; i < routeButtons.Count; i++)
            {
                var option = routeButtons[i];
                var left = (byte)(startLeft + i * (SELECT_BTN_W + SELECT_BTN_SPACING));
                DeleteButton(option.id);
                var text = option.name.Equals(selectedRoute, StringComparison.OrdinalIgnoreCase)
                    ? $"[{option.name}]"
                    : option.name;
                CreateButton(option.id, text, left, SELECT_ROW);
            }
        }

        /// <summary>
        /// Render route buttons for selecting a recorded route to visualize.
        /// </summary>
        private void RenderVisualizationRouteSelectors()
        {
            if (!uiInitialized) return;

            var maxButtons = GetVisualizationRouteButtonCapacity();
            var count = Math.Min(visualizationRouteButtons.Count, maxButtons);

            for (var i = 0; i < count; i++)
            {
                var option = visualizationRouteButtons[i];
                var column = i % VISUAL_ROUTE_COLUMNS;
                var row = i / VISUAL_ROUTE_COLUMNS;
                var left = column == 0 ? LEFT_COL : RIGHT_COL;
                var top = (byte)(VISUAL_ROUTE_START_ROW + row * ROW_HEIGHT);
                DeleteButton(option.id);
                var text = option.name.Equals(selectedVisualizationRoute, StringComparison.OrdinalIgnoreCase)
                    ? $"[{option.name}]"
                    : option.name;
                CreateButton(option.id, text, left, top);
            }
        }

        /// <summary>
        /// Render the detail selector used for visualization sampling.
        /// </summary>
        private void RenderVisualizationDetailControls()
        {
            if (!uiInitialized) return;

            var left = LEFT_COL;
            DeleteButton(VisualizationDetailMinusId);
            DeleteButton(VisualizationDetailPlusId);
            DeleteButton(VisualizationDetailLabelId);

            CreateButton(VisualizationDetailMinusId, "-", left, VISUAL_DETAIL_ROW, DETAIL_BTN_W);
            left = (byte)(left + DETAIL_BTN_W + DETAIL_BTN_SPACING);
            CreateButton(VisualizationDetailLabelId, $"Detail x{visualizationDetailStep}", left, VISUAL_DETAIL_ROW,
                DETAIL_LABEL_W, ROW_HEIGHT, false);
            left = (byte)(left + DETAIL_LABEL_W + DETAIL_BTN_SPACING);
            CreateButton(VisualizationDetailPlusId, "+", left, VISUAL_DETAIL_ROW, DETAIL_BTN_W);
        }

        /// <summary>
        /// Render the central Record toggle button near the bottom of the screen.
        /// </summary>
        private void RenderRecordButton()
        {
            if (!uiInitialized) return;

            var left = (byte)Math.Max(0, (200 - BTN_W) / 2);
            DeleteButton(1);
            var label = isRecording
                ? $"^2Rec {selectedRoute} ({recordedPoints})"
                : $"Record {selectedRoute}";
            CreateButton(1, label, left, RECORD_ROW);
        }

        /// <summary>
        /// Update visual state when the recording route changes.
        /// </summary>
        public void UpdateRecordingRouteSelection(string routeName)
        {
            var normalized = string.IsNullOrWhiteSpace(routeName) ? "main_loop" : routeName;
            selectedRoute = normalized;
            EnsureRouteButton(normalized);
        }

        /// <summary>
        /// Update the record button to reflect recording state and point count.
        /// </summary>
        public void UpdateRecordStatus(string routeName, int pointCount, bool recording)
        {
            if (!string.IsNullOrWhiteSpace(routeName))
            {
                selectedRoute = routeName;
            }

            isRecording = recording;
            recordedPoints = Math.Max(0, pointCount);
            RenderRecordButton();
        }

        /// <summary>
        /// Update the available route buttons to reflect current track/layout.
        /// </summary>
        public void SetRouteOptions(IEnumerable<string> routeNames, string preferredSelection)
        {
            selectedRoute = string.IsNullOrWhiteSpace(preferredSelection) ? selectedRoute : preferredSelection;
            var unique = new List<string>();

            if (routeNames != null)
            {
                foreach (var name in routeNames)
                {
                    if (string.IsNullOrWhiteSpace(name)) continue;
                    if (unique.Exists(r => r.Equals(name, StringComparison.OrdinalIgnoreCase))) continue;
                    unique.Add(name);
                }
            }

            if (!unique.Exists(r => r.Equals(selectedRoute, StringComparison.OrdinalIgnoreCase)))
                unique.Insert(0, selectedRoute);

            routeButtons.Clear();
            byte id = ROUTE_BUTTON_START_ID;
            foreach (var name in unique)
            {
                if (routeButtons.Count >= ROUTE_BUTTON_MAX_COUNT) break;
                routeButtons.Add((id, name));
                id++;
            }

            if (uiInitialized)
            {
                RenderRecordingSelectors();
                RenderRecordButton();
            }
        }

        /// <summary>
        /// Update the available route buttons to reflect current track/layout for visualization.
        /// </summary>
        public void SetVisualizationRouteOptions(IEnumerable<string> routeNames, string preferredSelection)
        {
            selectedVisualizationRoute = string.IsNullOrWhiteSpace(preferredSelection)
                ? selectedVisualizationRoute
                : preferredSelection;
            var unique = new List<string>();

            if (routeNames != null)
            {
                foreach (var name in routeNames)
                {
                    if (string.IsNullOrWhiteSpace(name)) continue;
                    if (unique.Exists(r => r.Equals(name, StringComparison.OrdinalIgnoreCase))) continue;
                    unique.Add(name);
                }
            }

            if (string.IsNullOrWhiteSpace(selectedVisualizationRoute) && unique.Count > 0)
                selectedVisualizationRoute = unique[0];

            if (!string.IsNullOrWhiteSpace(selectedVisualizationRoute) &&
                !unique.Exists(r => r.Equals(selectedVisualizationRoute, StringComparison.OrdinalIgnoreCase)))
                unique.Insert(0, selectedVisualizationRoute);

            foreach (var button in visualizationRouteButtons) DeleteButton(button.id);
            visualizationRouteButtons.Clear();
            byte id = VISUAL_ROUTE_BUTTON_START_ID;
            var maxButtons = GetVisualizationRouteButtonCapacity();
            foreach (var name in unique)
            {
                if (visualizationRouteButtons.Count >= maxButtons) break;
                visualizationRouteButtons.Add((id, name));
                id++;
            }

            if (uiInitialized)
            {
                RenderVisualizationRouteSelectors();
                RenderVisualizationDetailControls();
            }
        }

        /// <summary>
        /// Map a route selection button ClickID back to the route name it represents.
        /// </summary>
        public bool TryGetRouteNameForButton(byte clickId, out string routeName)
        {
            foreach (var button in routeButtons)
            {
                if (button.id == clickId)
                {
                    routeName = button.name;
                    return true;
                }
            }

            routeName = string.Empty;
            return false;
        }

        /// <summary>
        /// Map a visualization route button ClickID back to the route name it represents.
        /// </summary>
        public bool TryGetVisualizationRouteNameForButton(byte clickId, out string routeName)
        {
            foreach (var button in visualizationRouteButtons)
            {
                if (button.id == clickId)
                {
                    routeName = button.name;
                    return true;
                }
            }

            routeName = string.Empty;
            return false;
        }

        /// <summary>
        /// Get the currently selected recording route name.
        /// </summary>
        public string GetSelectedRoute()
        {
            return selectedRoute;
        }

        /// <summary>
        /// Update the selected visualization route name and refresh UI.
        /// </summary>
        public void UpdateVisualizationRouteSelection(string routeName)
        {
            var normalized = string.IsNullOrWhiteSpace(routeName) ? selectedVisualizationRoute : routeName;
            selectedVisualizationRoute = normalized;
            EnsureVisualizationRouteButton(normalized);
        }

        /// <summary>
        /// Update the visualization detail step and refresh UI.
        /// </summary>
        public void UpdateVisualizationDetail(int detailStep)
        {
            visualizationDetailStep = Math.Max(1, detailStep);
            RenderVisualizationDetailControls();
        }

        /// <summary>
        /// Make sure the selected route has a button so it can be toggled.
        /// </summary>
        private void EnsureRouteButton(string routeName)
        {
            if (routeButtons.Exists(b => b.name.Equals(routeName, StringComparison.OrdinalIgnoreCase)))
            {
                RenderRecordingSelectors();
                RenderRecordButton();
                return;
            }

            if (routeButtons.Count >= ROUTE_BUTTON_MAX_COUNT)
            {
                routeButtons.RemoveAt(routeButtons.Count - 1);
            }

            routeButtons.Insert(0, (ROUTE_BUTTON_START_ID, routeName));
            for (var i = 0; i < routeButtons.Count; i++)
            {
                routeButtons[i] = ((byte)(ROUTE_BUTTON_START_ID + i), routeButtons[i].name);
            }

            RenderRecordingSelectors();
            RenderRecordButton();
        }

        /// <summary>
        /// Make sure the selected visualization route has a button so it can be toggled.
        /// </summary>
        private void EnsureVisualizationRouteButton(string routeName)
        {
            if (visualizationRouteButtons.Exists(b => b.name.Equals(routeName, StringComparison.OrdinalIgnoreCase)))
            {
                RenderVisualizationRouteSelectors();
                RenderVisualizationDetailControls();
                return;
            }

            var maxButtons = GetVisualizationRouteButtonCapacity();
            if (visualizationRouteButtons.Count >= maxButtons)
            {
                var removed = visualizationRouteButtons[visualizationRouteButtons.Count - 1];
                DeleteButton(removed.id);
                visualizationRouteButtons.RemoveAt(visualizationRouteButtons.Count - 1);
            }

            visualizationRouteButtons.Insert(0, (VISUAL_ROUTE_BUTTON_START_ID, routeName));
            for (var i = 0; i < visualizationRouteButtons.Count; i++)
            {
                visualizationRouteButtons[i] =
                    ((byte)(VISUAL_ROUTE_BUTTON_START_ID + i), visualizationRouteButtons[i].name);
            }

            RenderVisualizationRouteSelectors();
            RenderVisualizationDetailControls();
        }

        /// <summary>
        /// Determine how many visualization route buttons fit in the available grid.
        /// </summary>
        private int GetVisualizationRouteButtonCapacity()
        {
            var availableRows = (SELECT_ROW - VISUAL_ROUTE_START_ROW) / ROW_HEIGHT;
            if (availableRows <= 0) availableRows = 1;
            return availableRows * VISUAL_ROUTE_COLUMNS;
        }
    }
}
