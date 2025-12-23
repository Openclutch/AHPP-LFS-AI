using AHPP_AI.Util;
using NUnit.Framework;

namespace AHPP_AI_Tests
{
    /// <summary>
    ///     Smoke tests to ensure the test harness runs and core utilities behave as expected.
    /// </summary>
    [TestFixture]
    public class SmokeTests
    {
        /// <summary>
        ///     Verify heading normalization wraps into the 0-65535 range.
        /// </summary>
        [Test]
        public void NormalizeHeading_WrapsIntoRange()
        {
            var normalized = CoordinateUtils.NormalizeHeading(70000);
            Assert.That(normalized, Is.InRange(0, CoordinateUtils.FULL_CIRCLE));
        }

        /// <summary>
        ///     Verify heading error is zero when headings match.
        /// </summary>
        [Test]
        public void CalculateHeadingError_ReturnsZeroForEqualHeadings()
        {
            var error = CoordinateUtils.CalculateHeadingError(12345, 12345);
            Assert.That(error, Is.EqualTo(0));
        }
    }
}
