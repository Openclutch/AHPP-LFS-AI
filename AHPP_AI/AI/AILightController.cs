using System;
using System.Collections.Generic;
using AHPP_AI.Debug;
using InSimDotNet;
using InSimDotNet.Packets;
using InSimClient = InSimDotNet.InSimClient;

namespace AHPP_AI.AI
{
    /// <summary>
    ///     Handles indicators, headlights, flash and horn control for AI drivers.
    /// </summary>
    public class AILightController
    {
        public enum IndicatorState
        {
            Off,
            Left,
            Right,
            Hazard
        }

        public enum HeadlightMode
        {
            Off = 1,
            SideLights = 2,
            LowBeam = 3,
            HighBeam = 4
        }

        private readonly InSimClient insim;
        private readonly Logger logger;

        private readonly Dictionary<byte, IndicatorState> indicatorStates = new Dictionary<byte, IndicatorState>();
        private readonly Dictionary<byte, DateTime> indicatorExpiry = new Dictionary<byte, DateTime>();
        private readonly Dictionary<byte, HeadlightMode> headlightStates = new Dictionary<byte, HeadlightMode>();

        /// <summary>
        ///     Create a new light controller bound to an InSim client.
        /// </summary>
        public AILightController(InSimClient insim, Logger logger)
        {
            this.insim = insim;
            this.logger = logger;
        }

        /// <summary>
        ///     Reset cached state for a specific AI so indicators and lights start clean.
        /// </summary>
        public void Reset(byte plid)
        {
            indicatorStates[plid] = IndicatorState.Off;
            indicatorExpiry.Remove(plid);
            headlightStates.Remove(plid);
        }

        /// <summary>
        ///     Apply indicator state and optionally schedule an auto-cancel.
        /// </summary>
        public void SetIndicators(byte plid, IndicatorState state, TimeSpan? duration = null)
        {
            if (indicatorStates.TryGetValue(plid, out var current) && current == state && duration == null)
                return;

            var value = state switch
            {
                IndicatorState.Left => 2,
                IndicatorState.Right => 3,
                IndicatorState.Hazard => 4,
                _ => 1
            };

            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_INDICATORS, Value = value } })
                { PLID = plid });

            indicatorStates[plid] = state;

            if (duration.HasValue && duration.Value.TotalMilliseconds > 0)
                indicatorExpiry[plid] = DateTime.Now.Add(duration.Value);
            else
                indicatorExpiry.Remove(plid);

            logger.Log($"PLID={plid} Indicators set to {state}");
        }

        /// <summary>
        ///     Turn off indicators immediately.
        /// </summary>
        public void CancelIndicators(byte plid)
        {
            SetIndicators(plid, IndicatorState.Off);
        }

        /// <summary>
        ///     Auto-cancel indicators when their hold duration expires.
        /// </summary>
        public void Update(byte plid)
        {
            if (!indicatorExpiry.TryGetValue(plid, out var expiry)) return;

            if (DateTime.Now >= expiry)
            {
                SetIndicators(plid, IndicatorState.Off);
                indicatorExpiry.Remove(plid);
            }
        }

        /// <summary>
        ///     Set headlight mode (off/side/low/high) for an AI.
        /// </summary>
        public void SetHeadlights(byte plid, HeadlightMode mode)
        {
            if (headlightStates.TryGetValue(plid, out var current) && current == mode)
                return;

            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_HEADLIGHTS, Value = (int)mode } })
                { PLID = plid });

            headlightStates[plid] = mode;
            logger.Log($"PLID={plid} Headlights set to {mode}");
        }

        /// <summary>
        ///     Convenience wrapper to switch high beams on or off.
        /// </summary>
        public void SetHighBeams(byte plid, bool enabled)
        {
            SetHeadlights(plid, enabled ? HeadlightMode.HighBeam : HeadlightMode.LowBeam);
        }

        /// <summary>
        ///     Flash high beams for a brief duration.
        /// </summary>
        public void FlashHighBeams(byte plid, byte durationHundredths = 15)
        {
            insim.Send(new IS_AIC(new List<AIInputVal>
                { new AIInputVal { Input = AicInputType.CS_FLASH, Time = durationHundredths, Value = 1 } })
                { PLID = plid });
        }

        /// <summary>
        ///     Honk the horn using one of the available tones.
        /// </summary>
        public void Honk(byte plid, byte hornTone = 1, byte durationHundredths = 20)
        {
            insim.Send(new IS_AIC(new List<AIInputVal>
            {
                new AIInputVal
                {
                    Input = AicInputType.CS_HORN,
                    Time = durationHundredths,
                    Value = hornTone
                }
            }) { PLID = plid });
        }

        /// <summary>
        ///     Toggle hazard lights with optional auto-cancel.
        /// </summary>
        public void SetHazards(byte plid, bool enabled, TimeSpan? duration = null)
        {
            SetIndicators(plid, enabled ? IndicatorState.Hazard : IndicatorState.Off, duration);
        }
    }
}
