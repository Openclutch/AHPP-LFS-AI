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
        private const byte ADD_AI_DIALOG_OFFSET = 21; // Position input below AI controls to avoid overlapping other buttons

        public const byte AddAiDialogId = 150;

        private readonly Dictionary<byte, byte> aiListButtons = new Dictionary<byte, byte>();
        
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
            byte row = 70;
            CreateButton(1, "Record", LEFT_COL, row); row += ROW_HEIGHT;
            CreateButton(2, "Load", LEFT_COL, row); row += ROW_HEIGHT;
            CreateButton(3, "Unload", LEFT_COL, row); row += ROW_HEIGHT;
            CreateButton(4, "Edit", LEFT_COL, row);
            row += ROW_HEIGHT;
            CreateButton(5, "Layout", LEFT_COL, row);

            row = 70;
            CreateButton(101, "Add AI", RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(102, "Set Speed", RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(103, "Stop All", RIGHT_COL, row); row += ROW_HEIGHT;
            CreateButton(104, "Spec All", RIGHT_COL, row);
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

        public void ShowAddAIDialog()
        {
            var btn = new IS_BTN
            {
                ReqI = 1,
                UCID = 0,
                ClickID = AddAiDialogId,
                Inst = 0,
                BStyle = ButtonStyles.ISB_DARK | ButtonStyles.ISB_CLICK,
                TypeIn = 3,
                L = RIGHT_COL,
                // Keep input box near the AI control cluster without covering AI list or debug buttons.
                T = (byte)(70 + ROW_HEIGHT * 4 + ADD_AI_DIALOG_OFFSET),
                W = BTN_W,
                H = ROW_HEIGHT,
                Text = string.Empty,
                Caption = "Num AIs"
            };
            insim.Send(btn);
        }

        public void HideAddAIDialog()
        {
            DeleteButton(AddAiDialogId);
        }
    }
}
