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
        private const byte SCREEN_WIDTH = 200;
        private const byte SHOW_BUTTON_LEFT = 0;
        private const byte SHOW_BUTTON_TOP = 0;
        private const byte SHOW_BUTTON_W = 12;
        private const byte VISUAL_ROUTE_START_ROW = 100;
        private const byte VISUAL_COLUMN_LEFT = (byte)(SCREEN_WIDTH > BTN_W ? SCREEN_WIDTH - BTN_W : 0);
        private const byte VISUAL_DETAIL_ROW = 160;
        private const byte DETAIL_BTN_W = 6;
        private const byte DETAIL_LABEL_W = 12;
        private const byte DETAIL_BTN_SPACING = 1;
        private const byte SELECT_ROW = 170;
        private const byte RECORD_ROW = 180;
        private const byte ROUTE_NAME_ROW = 160;
        private const byte HIDE_BUTTON_ROW = 60;
        private const byte INSIM_STATUS_ROW = 85;
        private const byte REMOVE_BTN_W = 6;
        private const byte RECORD_LABEL_ROW = (byte)(SELECT_ROW - ROW_HEIGHT - 1);
        private const byte RECORD_LABEL_W = 42;
        private const byte VISUALIZER_LABEL_ROW = (byte)(VISUAL_ROUTE_START_ROW - ROW_HEIGHT - 1);
        private const byte LAYOUT_EDIT_ROW = 190;
        private const byte LAYOUT_STATUS_ROW = 195;
        private const byte LAYOUT_STATUS_W = 30;

        public const byte RecordToggleId = (byte)(ButtonIds.MainStart + 0);
        public const byte ReloadRoutesId = (byte)(ButtonIds.MainStart + 1);
        public const byte ToggleLayoutId = (byte)(ButtonIds.MainStart + 2);
        public const byte ResetLayoutId = (byte)(ButtonIds.MainStart + 3);
        public const byte InSimStatusLabelId = (byte)(ButtonIds.MainStart + 4);
        public const byte InSimHostLabelId = (byte)(ButtonIds.MainStart + 5);
        public const byte RefreshSelectionFeedId = (byte)(ButtonIds.MainStart + 6);
        public const byte StartAllAisId = (byte)(ButtonIds.MainStart + 7);
        public const byte StopAllAisId = (byte)(ButtonIds.MainStart + 8);
        public const byte PitAllAisId = (byte)(ButtonIds.MainStart + 9);
        public const byte AddAiDialogId = (byte)(ButtonIds.MainStart + 10);
        public const byte SpeedInputId = (byte)(ButtonIds.MainStart + 11);
        public const byte RecordingIntervalId = (byte)(ButtonIds.MainStart + 12);
        public const byte RouteNameInputId = (byte)(ButtonIds.MainStart + 13);
        public const byte NodeSpeedInputId = (byte)(ButtonIds.MainStart + 14);
        public const byte LayoutAttachIndexId = (byte)(ButtonIds.MainStart + 15);
        public const byte LayoutRejoinIndexId = (byte)(ButtonIds.MainStart + 16);
        public const byte LayoutSelectionLabelId = (byte)(ButtonIds.MainStart + 17);
        private const byte RECORD_LABEL_ID = (byte)(ButtonIds.MainStart + 18);
        private const byte VISUALIZER_LABEL_ID = (byte)(ButtonIds.MainStart + 19);
        public const byte HideUiButtonId = (byte)(ButtonIds.MainStart + 63);
        public const byte ShowUiButtonId = (byte)(ButtonIds.MainStart + 64);
        public const byte VisualizationDetailMinusId = (byte)(ButtonIds.MainStart + 60);
        public const byte VisualizationDetailPlusId = (byte)(ButtonIds.MainStart + 61);
        public const byte VisualizationDetailLabelId = (byte)(ButtonIds.MainStart + 62);

        private readonly Dictionary<byte, byte> aiListButtons = new Dictionary<byte, byte>(); // AI ID -> button ID
        private readonly Dictionary<byte, byte> aiRemoveButtons = new Dictionary<byte, byte>(); // remove button ID -> AI ID
        private readonly List<(byte id, string name)> routeButtons = new List<(byte id, string name)>();
        private readonly List<(byte id, string name)> visualizationRouteButtons = new List<(byte id, string name)>();
        private readonly List<(byte id, string name)> lastAiSnapshot = new List<(byte id, string name)>();
        private const byte ROUTE_BUTTON_START_ID = (byte)(ButtonIds.MainStart + 20);
        private const byte ROUTE_BUTTON_MAX_COUNT = 8;
        private const byte VISUAL_ROUTE_BUTTON_START_ID = (byte)(ButtonIds.MainStart + 40);

        private string selectedRoute = "main_loop";
        private string selectedVisualizationRoute = "main_loop";
        private int visualizationDetailStep = 2;
        private bool isRecording;
        private int recordedPoints;
        private bool uiInitialized;
        private bool uiHidden;
        private string layoutSelectionStatus = "No node selected";
        private string insimStatus = "InSim: not connected";
        private string insimHost = "Host: pending";
        
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
            uiHidden = false;

            byte row = 70;
            CreateButton(HideUiButtonId, "Hide UI", LEFT_COL, HIDE_BUTTON_ROW);
            CreateButton(ReloadRoutesId, "Reload Routes", LEFT_COL, row); row += ROW_HEIGHT;
            CreateButton(ToggleLayoutId, "Toggle Layout", LEFT_COL, row); row += ROW_HEIGHT;
            CreateButton(ResetLayoutId, "Reset Layout", LEFT_COL, row);
            RenderInSimStatus();
            CreateButton(RefreshSelectionFeedId, "Refresh AXM", LEFT_COL, (byte)(row + ROW_HEIGHT * 3));

            row = 70;
            CreateInputButton(AddAiDialogId, RIGHT_COL, row, "AI Count"); row += ROW_HEIGHT;
            CreateInputButton(SpeedInputId, RIGHT_COL, row, "AI Speed"); row += ROW_HEIGHT;
            CreateInputButton(RecordingIntervalId, RIGHT_COL, row, "Rec Meters"); row += ROW_HEIGHT;
            CreateInputButton(RouteNameInputId, RIGHT_COL, ROUTE_NAME_ROW, "Route Name", 24);
            CreateButton(StartAllAisId, "Start All AI", RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(StopAllAisId, "Stop All AI", RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(PitAllAisId, "Pit All AI", RIGHT_COL, row);
            var layoutButtonGroupWidth = (byte)(BTN_W * 3 + SELECT_BTN_SPACING * 2);
            var layoutButtonLeft = (byte)Math.Max(0, (SCREEN_WIDTH - layoutButtonGroupWidth) / 2);
            var nodeSpeedLeft = (byte)(layoutButtonLeft + BTN_W + SELECT_BTN_SPACING);
            var rejoinLeft = (byte)(nodeSpeedLeft + BTN_W + SELECT_BTN_SPACING);
            CreateButton(LayoutAttachIndexId, "Set Attach", layoutButtonLeft, LAYOUT_EDIT_ROW);
            CreateInputButton(NodeSpeedInputId, nodeSpeedLeft, LAYOUT_EDIT_ROW, "Node Speed");
            CreateButton(LayoutRejoinIndexId, "Set Rejoin", rejoinLeft, LAYOUT_EDIT_ROW);

            RenderRecordingSelectors();
            RenderRecordButton();
            RenderVisualizationRouteSelectors();
            RenderVisualizationDetailControls();
            RenderLayoutSelectionStatus();
            RenderAIList(lastAiSnapshot);
        }

        /// <summary>
        /// Update the AI list on the right panel.
        /// </summary>
        public void UpdateAIList(IEnumerable<(byte id, string name)> ai)
        {
            lastAiSnapshot.Clear();
            if (ai != null)
            {
                lastAiSnapshot.AddRange(ai);
            }

            if (!uiInitialized || uiHidden) return;

            RenderAIList(lastAiSnapshot);
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
            if (!uiInitialized || uiHidden) return;

            RenderRecordingLabel();

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
            if (!uiInitialized || uiHidden) return;

            RenderVisualizationLabel();

            var maxButtons = GetVisualizationRouteButtonCapacity();
            var count = Math.Min(visualizationRouteButtons.Count, maxButtons);

            for (var i = 0; i < count; i++)
            {
                var option = visualizationRouteButtons[i];
                var left = VISUAL_COLUMN_LEFT;
                var top = (byte)(VISUAL_ROUTE_START_ROW + i * ROW_HEIGHT);
                DeleteButton(option.id);
                var text = option.name.Equals(selectedVisualizationRoute, StringComparison.OrdinalIgnoreCase)
                    ? $"[{option.name}]"
                    : option.name;
                CreateButton(option.id, text, left, top);
            }
        }

        /// <summary>
        /// Draw the label that identifies the visualization column.
        /// </summary>
        private void RenderVisualizationLabel()
        {
            if (!uiInitialized || uiHidden) return;

            DeleteButton(VISUALIZER_LABEL_ID);
            CreateButton(VISUALIZER_LABEL_ID, "Visualizer", VISUAL_COLUMN_LEFT, VISUALIZER_LABEL_ROW, BTN_W, ROW_HEIGHT, false);
        }

        /// <summary>
        /// Render the detail selector used for visualization sampling.
        /// </summary>
        private void RenderVisualizationDetailControls()
        {
            if (!uiInitialized || uiHidden) return;

            var detailWidth = DETAIL_BTN_W + DETAIL_BTN_SPACING + DETAIL_LABEL_W + DETAIL_BTN_SPACING + DETAIL_BTN_W;
            var left = (byte)Math.Max(0, SCREEN_WIDTH - detailWidth);
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
            if (!uiInitialized || uiHidden) return;

            var left = (byte)Math.Max(0, (200 - BTN_W) / 2);
            DeleteButton(RecordToggleId);
            var label = isRecording
                ? $"^2Rec {selectedRoute} ({recordedPoints})"
                : $"Record {selectedRoute}";
            CreateButton(RecordToggleId, label, left, RECORD_ROW);
        }

        /// <summary>
        /// Draw the label describing the recording controls below the route selectors.
        /// </summary>
        private void RenderRecordingLabel()
        {
            if (!uiInitialized || uiHidden) return;

            DeleteButton(RECORD_LABEL_ID);
            var left = (byte)Math.Max(0, (200 - RECORD_LABEL_W) / 2);
            CreateButton(RECORD_LABEL_ID, "Recording", left, RECORD_LABEL_ROW, RECORD_LABEL_W, ROW_HEIGHT, false);
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

            if (uiInitialized && !uiHidden)
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

            if (uiInitialized && !uiHidden)
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
        /// Update the layout editor selection status label.
        /// </summary>
        public void UpdateLayoutSelectionStatus(string status)
        {
            layoutSelectionStatus = string.IsNullOrWhiteSpace(status) ? "No node selected" : status;
            RenderLayoutSelectionStatus();
        }

        /// <summary>
        /// Render the layout selection status label for editing.
        /// </summary>
        private void RenderLayoutSelectionStatus()
        {
            if (!uiInitialized || uiHidden) return;

            DeleteButton(LayoutSelectionLabelId);
            CreateButton(LayoutSelectionLabelId, layoutSelectionStatus, RIGHT_COL, LAYOUT_STATUS_ROW, LAYOUT_STATUS_W,
                ROW_HEIGHT, false);
        }

        /// <summary>
        /// Update the InSim connection status and host labels.
        /// </summary>
        public void UpdateInSimStatus(string status, string host)
        {
            insimStatus = string.IsNullOrWhiteSpace(status) ? "InSim: unknown" : status;
            insimHost = string.IsNullOrWhiteSpace(host) ? "Host: unknown" : host;
            RenderInSimStatus();
        }

        /// <summary>
        /// Render the InSim connection status and host labels.
        /// </summary>
        private void RenderInSimStatus()
        {
            if (!uiInitialized || uiHidden) return;

            DeleteButton(InSimStatusLabelId);
            DeleteButton(InSimHostLabelId);

            CreateButton(InSimStatusLabelId, insimStatus, LEFT_COL, INSIM_STATUS_ROW, LAYOUT_STATUS_W, ROW_HEIGHT, false);
            CreateButton(InSimHostLabelId, insimHost, LEFT_COL, (byte)(INSIM_STATUS_ROW + ROW_HEIGHT), LAYOUT_STATUS_W,
                ROW_HEIGHT, false);
        }

        /// <summary>
        /// Hide all UI controls and leave a restore button in the top-left corner.
        /// </summary>
        public void HideUI()
        {
            uiHidden = true;
            ClearAllButtons();
            CreateButton(ShowUiButtonId, "Show UI", SHOW_BUTTON_LEFT, SHOW_BUTTON_TOP, SHOW_BUTTON_W);
        }

        /// <summary>
        /// Restore all UI controls after they have been hidden.
        /// </summary>
        public void ShowUI()
        {
            uiHidden = false;
            ClearAllButtons();
            Show();
        }

        /// <summary>
        /// Clear all buttons created by this InSim instance.
        /// </summary>
        private void ClearAllButtons()
        {
            var clear = new IS_BFN
            {
                ReqI = 1,
                SubT = ButtonFunction.BFN_CLEAR,
                UCID = 0,
                ClickID = 0,
                ClickMax = 0
            };
            insim.Send(clear);
        }

        /// <summary>
        /// Render the AI list using the latest snapshot of AI ids and names.
        /// </summary>
        private void RenderAIList(IEnumerable<(byte id, string name)> ai)
        {
            if (!uiInitialized || uiHidden) return;

            byte row = (byte)(5 + ROW_HEIGHT * 4);
            foreach (var b in aiListButtons.Values) DeleteButton(b);
            foreach (var b in aiRemoveButtons.Keys) DeleteButton(b);
            aiListButtons.Clear();
            aiRemoveButtons.Clear();

            byte index = 0;
            foreach (var (id, name) in ai)
            {
                var labelId = ButtonIds.Dynamic(index);
                var removeId = ButtonIds.Remove(index);
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
        /// Determine how many visualization route buttons fit vertically between the label and recording selectors.
        /// </summary>
        private int GetVisualizationRouteButtonCapacity()
        {
            var bottomRow = Math.Min((int)VISUAL_DETAIL_ROW, (int)SELECT_ROW);
            var availableRows = (bottomRow - VISUAL_ROUTE_START_ROW) / ROW_HEIGHT;
            if (availableRows <= 0) availableRows = 1;
            return availableRows;
        }
    }
}
