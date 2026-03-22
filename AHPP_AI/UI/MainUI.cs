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
        private const byte VISUAL_ROUTE_START_ROW = 60;
        private const byte VISUAL_COLUMN_LEFT = (byte)(SCREEN_WIDTH > BTN_W ? SCREEN_WIDTH - BTN_W : 0);
        private const byte VISUAL_DETAIL_ROW = 160;
        private const byte DETAIL_BTN_W = 6;
        private const byte DETAIL_LABEL_W = 12;
        private const byte DETAIL_BTN_SPACING = 1;
        private const byte SELECT_ROW = 170;
        private const byte RECORD_ROW = 180;
        private const byte DEBUG_BUTTON_ROW = 55;
        private const byte HIDE_BUTTON_ROW = 60;
        private const byte INSIM_STATUS_ROW = 85;
        private const byte PERFORMANCE_STATUS_ROW = 100;
        private const byte TRACK_LAYOUT_STATUS_ROW = (byte)(PERFORMANCE_STATUS_ROW + ROW_HEIGHT);
        private const byte REMOVE_BTN_W = 5;
        private const byte AI_LABEL_W = 14;
        private const byte AI_MODE_W = 8;
        private const byte AI_MODE_SPACING = 1;
        private const byte AI_LIST_START_ROW = (byte)(5 + ROW_HEIGHT * 4);
        private const byte RECORD_LABEL_ROW = (byte)(VISUAL_ROUTE_START_ROW - ROW_HEIGHT - 1);
        private const byte RECORD_LABEL_W = BTN_W;
        private const byte VISUALIZER_LABEL_ROW = (byte)(VISUAL_ROUTE_START_ROW - ROW_HEIGHT - 1);
        private const byte LAYOUT_EDIT_ROW = 190;
        private const byte LAYOUT_STATUS_ROW = 195;
        private const byte LAYOUT_STATUS_W = 30;
        private const byte TRACK_LAYOUT_OPTION_START_ID = (byte)(ButtonIds.MainStart + 73);
        private const byte TRACK_LAYOUT_OPTION_MAX_COUNT = 7;

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
        public const byte StartAutoAisId = (byte)(ButtonIds.MainStart + 65);
        public const byte SpawnDelayInputId = (byte)(ButtonIds.MainStart + 72);
        public const byte MaxAIsInputId = (byte)(ButtonIds.MainStart + 59);
        public const byte AddAiDialogId = (byte)(ButtonIds.MainStart + 10);
        public const byte SpeedInputId = (byte)(ButtonIds.MainStart + 11);
        public const byte RecordingIntervalId = (byte)(ButtonIds.MainStart + 12);
        public const byte RouteNameInputId = (byte)(ButtonIds.MainStart + 13);
        public const byte AddRouteButtonId = (byte)(ButtonIds.MainStart + 14);
        public const byte NodeSpeedInputId = (byte)(ButtonIds.MainStart + 15);
        public const byte LayoutAttachIndexId = (byte)(ButtonIds.MainStart + 16);
        public const byte LayoutRejoinIndexId = (byte)(ButtonIds.MainStart + 17);
        public const byte LayoutSelectionLabelId = (byte)(ButtonIds.MainStart + 18);
        public const byte LayoutDeleteNodeId = (byte)(ButtonIds.MainStart + 66);
        public const byte LayoutExtendRouteId = (byte)(ButtonIds.MainStart + 67);
        public const byte LayoutSpawnHereId = (byte)(ButtonIds.MainStart + 68);
        public const byte ToggleDebugButtonsId = (byte)(ButtonIds.MainStart + 69);
        public const byte PerformanceStatusId = (byte)(ButtonIds.MainStart + 70);
        public const byte TrackLayoutStatusId = (byte)(ButtonIds.MainStart + 71);
        public const byte RoutePagePreviousId = (byte)(ButtonIds.MainStart + 56);
        public const byte RoutePageLabelId = (byte)(ButtonIds.MainStart + 57);
        public const byte RoutePageNextId = (byte)(ButtonIds.MainStart + 58);
        private const byte RECORD_LABEL_ID = (byte)(ButtonIds.MainStart + 19);
        private const byte VISUALIZER_LABEL_ID = (byte)(ButtonIds.MainStart + 20);
        public const byte HideUiButtonId = (byte)(ButtonIds.MainStart + 63);
        public const byte ShowUiButtonId = (byte)(ButtonIds.MainStart + 64);
        public const byte VisualizationDetailMinusId = (byte)(ButtonIds.MainStart + 60);
        public const byte VisualizationDetailPlusId = (byte)(ButtonIds.MainStart + 61);
        public const byte VisualizationDetailLabelId = (byte)(ButtonIds.MainStart + 62);

        private class RouteButtonInfo
        {
            public string Name { get; set; } = string.Empty;
            public bool HasRecording { get; set; }
        }

        /// <summary>
        /// Track layout option shown beneath the active track button.
        /// </summary>
        private class TrackLayoutOptionInfo
        {
            public byte Id { get; set; }
            public string Name { get; set; } = string.Empty;
        }

        /// <summary>
        /// Single AI row shown in the right-hand list.
        /// </summary>
        public struct AiListEntry
        {
            public byte Id { get; set; }
            public string Name { get; set; }
            public string ModeLabel { get; set; }
        }

        private readonly Dictionary<byte, byte> aiLabelButtons = new Dictionary<byte, byte>(); // label button ID -> AI ID
        private readonly Dictionary<byte, byte> aiModeButtons = new Dictionary<byte, byte>(); // mode button ID -> AI ID
        private readonly Dictionary<byte, byte> aiRemoveButtons = new Dictionary<byte, byte>(); // remove button ID -> AI ID
        private readonly List<RouteButtonInfo> routeButtons = new List<RouteButtonInfo>();
        private readonly List<TrackLayoutOptionInfo> trackLayoutButtons = new List<TrackLayoutOptionInfo>();
        private readonly List<AiListEntry> lastAiSnapshot = new List<AiListEntry>();
        private const byte ROUTE_BUTTON_START_ID = (byte)(ButtonIds.MainStart + 21);
        private const byte VISUAL_ROUTE_BUTTON_START_ID = (byte)(ButtonIds.MainStart + 40);
        private readonly Dictionary<string, bool> routeRecordingStatus =
            new Dictionary<string, bool>(StringComparer.OrdinalIgnoreCase);

        private string selectedRoute = string.Empty;
        private string selectedVisualizationRoute = string.Empty;
        private int currentAiCount;
        private int visualizationDetailStep = 2;
        private bool isRecording;
        private int recordedPoints;
        private bool uiInitialized;
        private bool uiHidden;
        private string layoutSelectionStatus = "No node selected";
        private string insimStatus = "InSim: not connected";
        private string insimHost = "Host: pending";
        private bool autoPopulationEnabled = true;
        private byte autoAiButtonRow;
        private byte startAllAisRow;
        private bool layoutVisualizationEnabled;
        private byte toggleLayoutRow;
        private bool debugButtonsVisible = true;
        private string performanceStatusLabel = "AI Perf: waiting";
        private string trackLayoutStatusLabel = "Track: pending";
        private string selectedTrackLayoutName = "DefaultLayout";
        private bool trackLayoutDropdownExpanded;
        private int routePageIndex;
        
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
            CreateButton(ToggleDebugButtonsId, GetDebugButtonsToggleLabel(), LEFT_COL, DEBUG_BUTTON_ROW);
            CreateButton(HideUiButtonId, "Hide UI", LEFT_COL, HIDE_BUTTON_ROW);
            CreateButton(ReloadRoutesId, "Reload Routes", LEFT_COL, row); row += ROW_HEIGHT;
            toggleLayoutRow = row;
            CreateButton(ToggleLayoutId, GetLayoutToggleLabel(), LEFT_COL, row); row += ROW_HEIGHT;
            CreateButton(ResetLayoutId, "Reset Layout", LEFT_COL, row);
            RenderInSimStatus();
            CreateButton(RefreshSelectionFeedId, "Refresh AXM", LEFT_COL, (byte)(row + ROW_HEIGHT * 3));
            RenderPerformanceStatus();
            RenderTrackLayoutStatus();

            row = 70;
            CreateInputButton(SpawnDelayInputId, RIGHT_COL, row, "Spawn Delay (s)"); row += ROW_HEIGHT;
            CreateInputButton(MaxAIsInputId, RIGHT_COL, row, "Max Auto AI"); row += ROW_HEIGHT;
            CreateInputButton(AddAiDialogId, RIGHT_COL, row, "AI Count"); row += ROW_HEIGHT;
            CreateInputButton(SpeedInputId, RIGHT_COL, row, "AI Speed"); row += ROW_HEIGHT;
            CreateInputButton(RecordingIntervalId, RIGHT_COL, row, "Rec Meters"); row += ROW_HEIGHT;
            autoAiButtonRow = row;
            CreateButton(StartAutoAisId, GetAutoAiButtonText(), RIGHT_COL, row); row += ROW_HEIGHT;
            startAllAisRow = row;
            CreateButton(StartAllAisId, GetStartAllAiLabel(), RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(StopAllAisId, "Stop All AI", RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(PitAllAisId, "Pit All AI", RIGHT_COL, row);
            var layoutButtonGroupWidth = (byte)(BTN_W * 6 + SELECT_BTN_SPACING * 5);
            var layoutButtonLeft = (byte)Math.Max(0, (SCREEN_WIDTH - layoutButtonGroupWidth) / 2);
            var nodeSpeedLeft = (byte)(layoutButtonLeft + BTN_W + SELECT_BTN_SPACING);
            var rejoinLeft = (byte)(nodeSpeedLeft + BTN_W + SELECT_BTN_SPACING);
            var deleteLeft = (byte)(rejoinLeft + BTN_W + SELECT_BTN_SPACING);
            var extendLeft = (byte)(deleteLeft + BTN_W + SELECT_BTN_SPACING);
            var spawnLeft = (byte)(extendLeft + BTN_W + SELECT_BTN_SPACING);
            CreateButton(LayoutAttachIndexId, "Set Attach", layoutButtonLeft, LAYOUT_EDIT_ROW);
            CreateInputButton(NodeSpeedInputId, nodeSpeedLeft, LAYOUT_EDIT_ROW, "Node Speed");
            CreateButton(LayoutRejoinIndexId, "Set Rejoin", rejoinLeft, LAYOUT_EDIT_ROW);
            CreateButton(LayoutDeleteNodeId, "Delete Node", deleteLeft, LAYOUT_EDIT_ROW);
            CreateButton(LayoutExtendRouteId, "Extend Route", extendLeft, LAYOUT_EDIT_ROW);
            CreateButton(LayoutSpawnHereId, "Spawn Here", spawnLeft, LAYOUT_EDIT_ROW);

            RenderRouteSelectors();
            RenderRecordButton();
            RenderVisualizationDetailControls();
            RenderLayoutSelectionStatus();
            RenderAIList(lastAiSnapshot);
        }

        /// <summary>
        /// Update the AI list on the right panel.
        /// </summary>
        public void UpdateAIList(IEnumerable<AiListEntry> ai)
        {
            lastAiSnapshot.Clear();
            if (ai != null)
            {
                lastAiSnapshot.AddRange(ai);
            }
            currentAiCount = lastAiSnapshot.Count;

            if (!uiInitialized || uiHidden) return;

            RenderAIList(lastAiSnapshot);
            RenderAiCountLabel();
        }

        /// <summary>
        /// Reflect auto-population state in the UI button.
        /// </summary>
        public void SetAutoPopulationState(bool enabled)
        {
            autoPopulationEnabled = enabled;
            if (!uiInitialized || uiHidden || autoAiButtonRow == 0) return;
            CreateButton(StartAutoAisId, GetAutoAiButtonText(), RIGHT_COL, autoAiButtonRow);
        }

        /// <summary>
        /// Reflect debug button visibility state in the toggle control.
        /// </summary>
        /// <param name="visible">True when debug buttons are shown.</param>
        public void SetDebugButtonsVisible(bool visible)
        {
            debugButtonsVisible = visible;
            if (!uiInitialized || uiHidden) return;
            CreateButton(ToggleDebugButtonsId, GetDebugButtonsToggleLabel(), LEFT_COL, DEBUG_BUTTON_ROW);
        }

        /// <summary>
        /// Build the auto-population button label based on current state.
        /// </summary>
        private string GetAutoAiButtonText()
        {
            return autoPopulationEnabled ? "Auto AI: On" : "Auto AI: Off";
        }

        /// <summary>
        /// Build the label for the Start All AI button with the current AI count.
        /// </summary>
        private string GetStartAllAiLabel()
        {
            return $"Start All AI ({currentAiCount})";
        }

        /// <summary>
        /// Build the label for the layout visualization toggle button.
        /// </summary>
        private string GetLayoutToggleLabel()
        {
            return layoutVisualizationEnabled ? "Hide Layout" : "Show Layout";
        }

        /// <summary>
        /// Build the label for the debug visibility toggle.
        /// </summary>
        private string GetDebugButtonsToggleLabel()
        {
            return debugButtonsVisible ? "Hide Debug HUD" : "Show Debug HUD";
        }

        /// <summary>
        /// Update the layout toggle button label to reflect current state.
        /// </summary>
        public void UpdateLayoutToggleState(bool enabled)
        {
            layoutVisualizationEnabled = enabled;
            if (!uiInitialized || uiHidden || toggleLayoutRow == 0) return;
            CreateButton(ToggleLayoutId, GetLayoutToggleLabel(), LEFT_COL, toggleLayoutRow);
        }

        /// <summary>
        /// Map a remove button ClickID to the AI it controls.
        /// </summary>
        public bool TryGetAiForRemoveButton(byte clickId, out byte aiId)
        {
            return aiRemoveButtons.TryGetValue(clickId, out aiId);
        }

        /// <summary>
        /// Map an AI label ClickID to the PLID it represents.
        /// </summary>
        public bool TryGetAiForLabelButton(byte clickId, out byte aiId)
        {
            return aiLabelButtons.TryGetValue(clickId, out aiId);
        }

        /// <summary>
        /// Map an AI mode ClickID to the PLID it represents.
        /// </summary>
        public bool TryGetAiForModeButton(byte clickId, out byte aiId)
        {
            return aiModeButtons.TryGetValue(clickId, out aiId);
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
            insim.SendUi(btn);
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
            insim.SendUi(del);
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
            insim.SendUi(btn);
        }

        /// <summary>
        /// Render unified route selection buttons used for recording and visualization.
        /// </summary>
        private void RenderRouteSelectors()
        {
            if (!uiInitialized || uiHidden) return;

            RenderRecordingLabel();

            var pageSize = GetRouteButtonPageSize();
            var pageCount = GetRoutePageCount(pageSize);
            routePageIndex = Math.Max(0, Math.Min(routePageIndex, pageCount - 1));
            var startIndex = routePageIndex * pageSize;
            var count = Math.Min(pageSize, Math.Max(0, routeButtons.Count - startIndex));

            var startTop = VISUAL_ROUTE_START_ROW;
            var left = VISUAL_COLUMN_LEFT;
            var routeIdsToClear = pageSize;

            for (var i = 0; i < routeIdsToClear; i++)
            {
                DeleteButton((byte)(ROUTE_BUTTON_START_ID + i));
            }

            for (var i = 0; i < count; i++)
            {
                var option = routeButtons[startIndex + i];
                var top = (byte)(startTop + i * ROW_HEIGHT);
                var text = FormatRouteLabel(option);
                CreateButton((byte)(ROUTE_BUTTON_START_ID + i), text, left, top);
            }

            var pagerTop = (byte)(startTop + pageSize * ROW_HEIGHT);
            DeleteButton(RoutePagePreviousId);
            DeleteButton(RoutePageLabelId);
            DeleteButton(RoutePageNextId);
            if (pageCount > 1)
            {
                CreateButton(RoutePagePreviousId, "<", left, pagerTop, 5);
                CreateButton(RoutePageLabelId, $"{routePageIndex + 1}/{pageCount}", (byte)(left + 5), pagerTop, 10,
                    ROW_HEIGHT, false);
                CreateButton(RoutePageNextId, ">", (byte)(left + 15), pagerTop, 5);
            }

            // Add a button to create/confirm a new route using the Route Name input at the bottom of the column.
            var inputStartTop = (byte)(pagerTop + (pageCount > 1 ? ROW_HEIGHT : 0));
            var routeNameTop = (byte)Math.Min(200 - ROW_HEIGHT * 2, (int)inputStartTop);
            var addTop = (byte)Math.Min(200 - ROW_HEIGHT, routeNameTop + ROW_HEIGHT);
            DeleteButton(RouteNameInputId);
            CreateInputButton(RouteNameInputId, left, routeNameTop, "Route Name", 24);
            DeleteButton(AddRouteButtonId);
            CreateButton(AddRouteButtonId, "Add Route", left, addTop);
        }

        /// <summary>
        /// Build a labeled route name with color indicating recording state.
        /// </summary>
        private string FormatRouteLabel(RouteButtonInfo option)
        {
            var color = option.HasRecording ? "^2" : "^1";
            const string reset = "^7";
            if (option.Name.Equals(selectedRoute, StringComparison.OrdinalIgnoreCase))
            {
                // Keep brackets white while coloring the route name.
                return $"{reset}[{color}{option.Name}{reset}]";
            }

            return $"{color}{option.Name}";
        }

        /// <summary>
        /// Render route buttons for selecting a recorded route to visualize.
        /// </summary>
        private void RenderVisualizationRouteSelectors()
        {
            // Visualization uses the same unified route buttons; no separate rendering needed.
        }


        /// <summary>
        /// Render the detail selector used for visualization sampling.
        /// </summary>
        private void RenderVisualizationDetailControls()
        {
            if (!uiInitialized || uiHidden) return;

            var detailWidth = DETAIL_BTN_W + DETAIL_BTN_SPACING + DETAIL_LABEL_W + DETAIL_BTN_SPACING + DETAIL_BTN_W;
            // Offset detail controls to the left of the route column to avoid overlap.
            var left = (byte)Math.Max(0, VISUAL_COLUMN_LEFT - detailWidth - SELECT_BTN_SPACING);
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
            var left = VISUAL_COLUMN_LEFT;
            CreateButton(RECORD_LABEL_ID, "Routes", left, RECORD_LABEL_ROW, RECORD_LABEL_W, ROW_HEIGHT, false);
        }

        /// <summary>
        /// Update visual state when the recording route changes.
        /// </summary>
        public void UpdateRecordingRouteSelection(string routeName)
        {
            var normalized = string.IsNullOrWhiteSpace(routeName) ? selectedRoute : routeName;
            selectedRoute = normalized;
            if (!string.IsNullOrWhiteSpace(normalized))
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
        public void SetRouteOptions(IEnumerable<(string name, bool recorded)> routeOptions, string preferredSelection)
        {
            selectedRoute = string.IsNullOrWhiteSpace(preferredSelection) ? selectedRoute : preferredSelection;
            var unique = new List<(string name, bool recorded)>();
            routeRecordingStatus.Clear();

            if (routeOptions != null)
            {
                foreach (var option in routeOptions)
                {
                    if (string.IsNullOrWhiteSpace(option.name)) continue;
                    var existingIndex = unique.FindIndex(r => r.name.Equals(option.name, StringComparison.OrdinalIgnoreCase));
                    if (existingIndex >= 0)
                    {
                        var mergedRecorded = unique[existingIndex].recorded || option.recorded;
                        unique[existingIndex] = (unique[existingIndex].name, mergedRecorded);
                    }
                    else
                    {
                        unique.Add(option);
                    }

                    if (routeRecordingStatus.TryGetValue(option.name, out var stored))
                        routeRecordingStatus[option.name] = stored || option.recorded;
                    else
                        routeRecordingStatus[option.name] = option.recorded;
                }
            }

            if (!unique.Exists(r => r.name.Equals(selectedRoute, StringComparison.OrdinalIgnoreCase)))
            {
                var recorded = routeRecordingStatus.TryGetValue(selectedRoute, out var hasRecording) && hasRecording;
                unique.Insert(0, (selectedRoute, recorded));
                routeRecordingStatus[selectedRoute] = recorded;
            }

            routeButtons.Clear();
            foreach (var option in unique)
            {
                var recorded = routeRecordingStatus.TryGetValue(option.name, out var hasRecording)
                    ? hasRecording
                    : option.recorded;
                routeButtons.Add(new RouteButtonInfo
                {
                    Name = option.name,
                    HasRecording = recorded
                });
            }

            routePageIndex = GetPageIndexForRoute(selectedRoute);

            if (uiInitialized && !uiHidden)
            {
                RenderRouteSelectors();
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
            if (uiInitialized && !uiHidden) RenderVisualizationDetailControls();
        }

        /// <summary>
        /// Map a route selection button ClickID back to the route name it represents.
        /// </summary>
        public bool TryGetRouteNameForButton(byte clickId, out string routeName)
        {
            var pageSize = GetRouteButtonPageSize();
            var pageOffset = clickId - ROUTE_BUTTON_START_ID;
            if (clickId >= ROUTE_BUTTON_START_ID && pageOffset < pageSize)
            {
                var index = routePageIndex * pageSize + pageOffset;
                if (index >= 0 && index < routeButtons.Count)
                {
                    routeName = routeButtons[index].Name;
                    return true;
                }
            }

            routeName = string.Empty;
            return false;
        }

        /// <summary>
        /// Map a track-layout option button ClickID back to the layout name it represents.
        /// </summary>
        public bool TryGetTrackLayoutNameForButton(byte clickId, out string layoutName)
        {
            foreach (var button in trackLayoutButtons)
            {
                if (button.Id != clickId)
                    continue;

                layoutName = button.Name;
                return true;
            }

            layoutName = string.Empty;
            return false;
        }

        /// <summary>
        /// Map a visualization route button ClickID back to the route name it represents.
        /// </summary>
        public bool TryGetVisualizationRouteNameForButton(byte clickId, out string routeName)
        {
            return TryGetRouteNameForButton(clickId, out routeName);
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
            EnsureRouteButton(normalized);
        }

        /// <summary>
        /// Show the previous page of route buttons when multiple pages are available.
        /// </summary>
        public void ShowPreviousRoutePage()
        {
            if (routePageIndex <= 0)
                return;

            routePageIndex--;
            RenderRouteSelectors();
        }

        /// <summary>
        /// Show the next page of route buttons when multiple pages are available.
        /// </summary>
        public void ShowNextRoutePage()
        {
            var pageCount = GetRoutePageCount(GetRouteButtonPageSize());
            if (routePageIndex >= pageCount - 1)
                return;

            routePageIndex++;
            RenderRouteSelectors();
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
            var left = (byte)Math.Max(0, (SCREEN_WIDTH - LAYOUT_STATUS_W) / 2);
            CreateButton(LayoutSelectionLabelId, layoutSelectionStatus, left, LAYOUT_STATUS_ROW, LAYOUT_STATUS_W,
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
        /// Update the performance status indicator shown beneath the Refresh AXM control.
        /// </summary>
        public void UpdatePerformanceStatus(string label)
        {
            performanceStatusLabel = string.IsNullOrWhiteSpace(label) ? "AI Perf: waiting" : label;
            RenderPerformanceStatus();
        }

        /// <summary>
        /// Update the track/layout status label shown beneath the performance indicator.
        /// </summary>
        public void UpdateTrackLayoutStatus(string trackCode, string layoutName)
        {
            var track = string.IsNullOrWhiteSpace(trackCode) ? "UnknownTrack" : trackCode.Trim();
            var layout = string.IsNullOrWhiteSpace(layoutName) ? "DefaultLayout" : layoutName.Trim();
            selectedTrackLayoutName = layout;
            trackLayoutStatusLabel = $"Track: {track} / {layout}";
            RenderTrackLayoutStatus();
        }

        /// <summary>
        /// Update the selectable layouts shown beneath the active track button.
        /// </summary>
        public void SetTrackLayoutOptions(IEnumerable<string> layouts, string selectedLayout)
        {
            selectedTrackLayoutName = string.IsNullOrWhiteSpace(selectedLayout) ? "DefaultLayout" : selectedLayout.Trim();
            var unique = new List<string>();

            if (layouts != null)
            {
                foreach (var layout in layouts)
                {
                    if (string.IsNullOrWhiteSpace(layout))
                        continue;

                    var trimmed = layout.Trim();
                    if (!unique.Exists(existing => existing.Equals(trimmed, StringComparison.OrdinalIgnoreCase)))
                        unique.Add(trimmed);
                }
            }

            unique.Sort(StringComparer.OrdinalIgnoreCase);
            trackLayoutButtons.Clear();

            byte id = TRACK_LAYOUT_OPTION_START_ID;
            foreach (var layout in unique)
            {
                if (trackLayoutButtons.Count >= TRACK_LAYOUT_OPTION_MAX_COUNT)
                    break;

                trackLayoutButtons.Add(new TrackLayoutOptionInfo
                {
                    Id = id,
                    Name = layout
                });
                id++;
            }

            if (trackLayoutButtons.Count == 0)
                trackLayoutDropdownExpanded = false;

            if (!uiInitialized || uiHidden) return;

            RenderTrackLayoutStatus();
        }

        /// <summary>
        /// Toggle the visibility of the layout list beneath the track status label.
        /// </summary>
        public void ToggleTrackLayoutDropdown()
        {
            if (trackLayoutButtons.Count == 0)
                return;

            trackLayoutDropdownExpanded = !trackLayoutDropdownExpanded;
            RenderTrackLayoutStatus();
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
        /// Render the current AI performance status label.
        /// </summary>
        private void RenderPerformanceStatus()
        {
            if (!uiInitialized || uiHidden) return;

            DeleteButton(PerformanceStatusId);
            CreateButton(PerformanceStatusId, performanceStatusLabel, LEFT_COL, PERFORMANCE_STATUS_ROW, LAYOUT_STATUS_W,
                ROW_HEIGHT, false);
        }

        /// <summary>
        /// Render the current track/layout label beneath the performance indicator.
        /// </summary>
        private void RenderTrackLayoutStatus()
        {
            if (!uiInitialized || uiHidden) return;

            DeleteButton(TrackLayoutStatusId);
            CreateButton(TrackLayoutStatusId, trackLayoutStatusLabel, LEFT_COL, TRACK_LAYOUT_STATUS_ROW, LAYOUT_STATUS_W,
                ROW_HEIGHT);
            RenderTrackLayoutOptions();
        }

        /// <summary>
        /// Render the available layouts stacked beneath the track status button when expanded.
        /// </summary>
        private void RenderTrackLayoutOptions()
        {
            foreach (var option in trackLayoutButtons)
                DeleteButton(option.Id);

            if (!trackLayoutDropdownExpanded)
                return;

            for (var i = 0; i < trackLayoutButtons.Count; i++)
            {
                var option = trackLayoutButtons[i];
                var top = (byte)(TRACK_LAYOUT_STATUS_ROW + ROW_HEIGHT * (i + 1));
                var text = option.Name.Equals(selectedTrackLayoutName, StringComparison.OrdinalIgnoreCase)
                    ? $"^2> {option.Name}"
                    : option.Name;
                CreateButton(option.Id, text, LEFT_COL, top, LAYOUT_STATUS_W, ROW_HEIGHT);
            }
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
            insim.SendUi(clear);
        }

        /// <summary>
        /// Render the AI list using the latest snapshot of AI ids and names.
        /// </summary>
        private void RenderAIList(IEnumerable<AiListEntry> ai)
        {
            if (!uiInitialized || uiHidden) return;

            byte row = AI_LIST_START_ROW;
            foreach (var buttonId in aiLabelButtons.Keys) DeleteButton(buttonId);
            foreach (var buttonId in aiModeButtons.Keys) DeleteButton(buttonId);
            foreach (var b in aiRemoveButtons.Keys) DeleteButton(b);
            aiLabelButtons.Clear();
            aiModeButtons.Clear();
            aiRemoveButtons.Clear();

            byte index = 0;
            foreach (var entry in ai)
            {
                if (index >= GetAiListCapacity())
                    break;

                var labelId = ButtonIds.AiLabel(index);
                var modeId = ButtonIds.AiMode(index);
                var removeId = ButtonIds.AiRemove(index);
                var top = (byte)(row + ROW_HEIGHT * index);
                var removeLeft = (byte)Math.Max(0, AI_LIST_COL - REMOVE_BTN_W - 2);
                var modeLeft = (byte)(AI_LIST_COL + AI_LABEL_W + AI_MODE_SPACING);

                CreateButton(removeId, "X", removeLeft, top, REMOVE_BTN_W);
                CreateButton(labelId, entry.Name ?? string.Empty, AI_LIST_COL, top, AI_LABEL_W);
                CreateButton(modeId, entry.ModeLabel ?? "Cruise", modeLeft, top, AI_MODE_W);

                aiLabelButtons[labelId] = entry.Id;
                aiModeButtons[modeId] = entry.Id;
                aiRemoveButtons[removeId] = entry.Id;
                index++;
            }
        }

        /// <summary>
        /// Refresh the Start All AI button to display the current AI count.
        /// </summary>
        private void RenderAiCountLabel()
        {
            if (!uiInitialized || uiHidden || startAllAisRow == 0) return;
            CreateButton(StartAllAisId, GetStartAllAiLabel(), RIGHT_COL, startAllAisRow);
        }

        /// <summary>
        /// Make sure the selected route has a button so it can be toggled.
        /// </summary>
        private void EnsureRouteButton(string routeName)
        {
            if (routeButtons.Exists(b => b.Name.Equals(routeName, StringComparison.OrdinalIgnoreCase)))
            {
                routePageIndex = GetPageIndexForRoute(routeName);
                RenderRouteSelectors();
                RenderRecordButton();
                return;
            }

            var recorded = routeRecordingStatus.TryGetValue(routeName, out var hasRecording) && hasRecording;
            routeButtons.Insert(0, new RouteButtonInfo
            {
                Name = routeName,
                HasRecording = recorded
            });
            routeRecordingStatus[routeName] = recorded;
            routePageIndex = GetPageIndexForRoute(routeName);

            RenderRouteSelectors();
            RenderRecordButton();
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

        /// <summary>
        /// Reserve one row for paging controls so long route lists can be browsed in place.
        /// </summary>
        private int GetRouteButtonPageSize()
        {
            return Math.Max(1, GetVisualizationRouteButtonCapacity() - 1);
        }

        /// <summary>
        /// Determine which route page should be shown for the supplied route selection.
        /// </summary>
        private int GetPageIndexForRoute(string routeName)
        {
            if (string.IsNullOrWhiteSpace(routeName) || routeButtons.Count == 0)
                return 0;

            var routeIndex = routeButtons.FindIndex(button =>
                button.Name.Equals(routeName, StringComparison.OrdinalIgnoreCase));
            if (routeIndex < 0)
                return 0;

            var pageSize = GetRouteButtonPageSize();
            return routeIndex / pageSize;
        }

        /// <summary>
        /// Calculate how many route pages are required for the current route list.
        /// </summary>
        private int GetRoutePageCount(int pageSize)
        {
            if (pageSize <= 0)
                return 1;

            return Math.Max(1, (routeButtons.Count + pageSize - 1) / pageSize);
        }

        /// <summary>
        /// Determine how many AI rows fit on screen while preserving dedicated label, mode, and remove IDs.
        /// </summary>
        private int GetAiListCapacity()
        {
            var bottomRow = 200 - ROW_HEIGHT;
            var availableRows = (bottomRow - AI_LIST_START_ROW) / ROW_HEIGHT + 1;
            if (availableRows <= 0) availableRows = 1;
            return Math.Min(35, availableRows);
        }
    }
}
