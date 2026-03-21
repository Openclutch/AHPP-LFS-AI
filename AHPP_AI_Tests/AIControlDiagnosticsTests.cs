using System.Linq;
using AHPP_AI.AI;
using InSimDotNet.Packets;
using NUnit.Framework;

namespace AHPP_AI_Tests
{
    /// <summary>
    ///     Tests that document how AI control packets and overlap detection should behave.
    /// </summary>
    [TestFixture]
    public class AIControlDiagnosticsTests
    {
        /// <summary>
        ///     Resetting inputs per tick should prepend a reset command before the new controls.
        /// </summary>
        [Test]
        public void BuildInputPacket_PrependsReset_WhenEnabled()
        {
            var inputs = AIControlDiagnostics.BuildInputPacket(32768, 12000, 0, 2, 65535, false, true);

            Assert.That(inputs.First().Input, Is.EqualTo(AicInputType.CS_RESET_INPUTS));
            Assert.That(inputs.Skip(1).Select(input => input.Input), Is.EqualTo(new[]
            {
                AicInputType.CS_MSX,
                AicInputType.CS_THROTTLE,
                AicInputType.CS_BRAKE,
                AicInputType.CS_GEAR,
                AicInputType.CS_CLUTCH
            }));
        }

        /// <summary>
        ///     Automatic transmission mode should leave gear and clutch to LFS.
        /// </summary>
        [Test]
        public void BuildInputPacket_OmitsGearAndClutch_WhenAutomaticTransmission()
        {
            var inputs = AIControlDiagnostics.BuildInputPacket(32768, 12000, 0, 4, 20000, true, false);

            Assert.That(inputs.Select(input => input.Input), Is.EqualTo(new[]
            {
                AicInputType.CS_MSX,
                AicInputType.CS_THROTTLE,
                AicInputType.CS_BRAKE
            }));
        }

        /// <summary>
        ///     Throttle with a heavily engaged clutch should be treated as suspicious once the car is beyond launch.
        /// </summary>
        [Test]
        public void HasSuspiciousOverlap_FlagsThrottleAndClutchOutsideLaunch()
        {
            var suspicious = AIControlDiagnostics.HasSuspiciousOverlap(
                24000,
                0,
                50000,
                false,
                4,
                42.0,
                10000,
                65535,
                false,
                out var reason);

            Assert.That(suspicious, Is.True);
            Assert.That(reason, Is.EqualTo("Throttle+clutch overlap"));
        }

        /// <summary>
        ///     Low-speed first-gear launches are allowed to blend throttle and clutch.
        /// </summary>
        [Test]
        public void HasSuspiciousOverlap_AllowsLowSpeedFirstGearLaunch()
        {
            var suspicious = AIControlDiagnostics.HasSuspiciousOverlap(
                20000,
                0,
                50000,
                false,
                2,
                3.0,
                10000,
                65535,
                false,
                out var reason);

            Assert.That(suspicious, Is.False);
            Assert.That(reason, Is.Empty);
        }

        /// <summary>
        ///     The compact HUD string should show when per-tick reset mode is active.
        /// </summary>
        [Test]
        public void FormatCompactControlInfo_AppendsResetMarker_WhenEnabled()
        {
            var info = AIControlDiagnostics.FormatCompactControlInfo(32768, 32768, 12000, 0, 2, 65535, false, true);

            Assert.That(info, Does.Contain("R:1"));
        }
    }
}
