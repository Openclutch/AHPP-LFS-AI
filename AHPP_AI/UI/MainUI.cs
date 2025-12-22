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
        private const byte BTN_W = 40;
        private const byte SELECT_BTN_W = 30;
        private const byte SELECT_BTN_SPACING = 2;
        private const byte SELECT_ROW = 170;
        private const byte RECORD_ROW = 180;

        public const byte AddAiDialogId = 150;
        public const byte SpeedInputId = 151;

        private readonly Dictionary<byte, byte> aiListButtons = new Dictionary<byte, byte>();
        private readonly (byte id, string name, string label)[] routeOptions =
        {
            (11, "main_loop", "Main"),
            (12, "pit_entry", "Pit"),
            (13, "detour1", "Detour1"),
            (14, "detour2", "Detour2"),
            (15, "detour3", "Detour3")
        };

        private string selectedRoute = "main_loop";
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
            CreateButton(103, "Stop All AI", RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(104, "Spec All AI", RIGHT_COL, row);

            RenderRecordingSelectors();
            RenderRecordButton();
        }

        /// <summary>
        /// Update the AI list on the right panel.
        /// </summary>
        public void UpdateAIList(IEnumerable<(byte id, string name)> ai)
        {
            byte row = (byte)(5 + ROW_HEIGHT * 4);
            foreach (var b in aiListButtons.Values)
            {
                DeleteButton(b);
            }
            aiListButtons.Clear();

            byte index = 0;
            foreach (var (id, name) in ai)
            {
                byte btnId = (byte)(120 + index);
                CreateButton(btnId, $"{name}", AI_LIST_COL, (byte)(row + ROW_HEIGHT * index));
                aiListButtons[id] = btnId;
                index++;
            }
        }

        /// <summary>
        /// Create a clickable button in the UI.
        /// </summary>
        private void CreateButton(byte id, string text, byte left, byte top)
        {
            var btn = new IS_BTN
            {
                ReqI = 1,
                UCID = 0,
                ClickID = id,
                Inst = 0,
                BStyle = ButtonStyles.ISB_DARK | ButtonStyles.ISB_CLICK,
                L = left,
                T = top,
                W = BTN_W,
                H = ROW_HEIGHT,
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
        private void CreateInputButton(byte id, byte left, byte top, string caption)
        {
            var btn = new IS_BTN
            {
                ReqI = 1,
                UCID = 0,
                ClickID = id,
                Inst = 0,
                BStyle = ButtonStyles.ISB_DARK | ButtonStyles.ISB_CLICK,
                TypeIn = 3,
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

            byte totalWidth = (byte)(routeOptions.Length * SELECT_BTN_W + (routeOptions.Length - 1) * SELECT_BTN_SPACING);
            var startLeft = (byte)Math.Max(0, (200 - totalWidth) / 2);

            for (var i = 0; i < routeOptions.Length; i++)
            {
                var option = routeOptions[i];
                var left = (byte)(startLeft + i * (SELECT_BTN_W + SELECT_BTN_SPACING));
                var text = option.name.Equals(selectedRoute, StringComparison.OrdinalIgnoreCase)
                    ? $"[{option.label}]"
                    : option.label;
                DeleteButton(option.id);
                CreateButton(option.id, text, left, SELECT_ROW);
            }
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
            selectedRoute = string.IsNullOrWhiteSpace(routeName) ? "main_loop" : routeName;
            RenderRecordingSelectors();
            RenderRecordButton();
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
    }
}
