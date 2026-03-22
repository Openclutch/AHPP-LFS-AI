using AHPP_AI.Util;
using NUnit.Framework;

namespace AHPP_AI_Tests
{
    /// <summary>
    ///     Tests for path projection geometry relied on by spawn-route transitions.
    /// </summary>
    [TestFixture]
    public class PathProjectionTests
    {
        /// <summary>
        ///     Non-loop paths should report direct forward distance between coordinates.
        /// </summary>
        [Test]
        public void GetForwardDistance_ReturnsDirectDelta_ForNonLoopPath()
        {
            var path = TestRouteFixtures.StraightPath(0.0, 10.0, 20.0, 30.0);
            var geometry = PathProjection.GetGeometry(path, 5.0);

            Assert.That(geometry, Is.Not.Null);
            Assert.That(PathProjection.GetForwardDistance(geometry!, 5.0, 25.0), Is.EqualTo(20.0).Within(0.001));
            Assert.That(PathProjection.GetBackwardDistance(geometry!, 25.0, 5.0), Is.EqualTo(20.0).Within(0.001));
        }

        /// <summary>
        ///     Loop paths should wrap forward distance across the end of the path.
        /// </summary>
        [Test]
        public void GetForwardDistance_WrapsAcrossLoopClosure_ForLoopPath()
        {
            var path = TestRouteFixtures.SquareLoop(10.0);
            var geometry = PathProjection.GetGeometry(path, 5.0);

            Assert.That(geometry, Is.Not.Null);
            Assert.That(geometry!.IsLoop, Is.True);
            Assert.That(PathProjection.GetForwardDistance(geometry, 35.0, 5.0), Is.EqualTo(10.0).Within(0.001));
            Assert.That(PathProjection.GetBackwardDistance(geometry, 35.0, 5.0), Is.EqualTo(30.0).Within(0.001));
        }

        /// <summary>
        ///     Paths whose last point closes back to the first inside the configured threshold should be treated as loops.
        /// </summary>
        [Test]
        public void GetGeometry_TreatsNearClosureAsLoop_WhenWithinThreshold()
        {
            var path = TestRouteFixtures.SquareLoop(8.0);

            var geometry = PathProjection.GetGeometry(path, 5.0);

            Assert.That(geometry, Is.Not.Null);
            Assert.That(geometry!.IsLoop, Is.True);
        }

        /// <summary>
        ///     Projecting onto a simple line should return the expected distance-along-path.
        /// </summary>
        [Test]
        public void TryProjectToPath_ComputesDistanceAlongPath_OnStraightSegment()
        {
            var path = TestRouteFixtures.StraightPath(0.0, 10.0, 20.0, 30.0);
            var geometry = PathProjection.GetGeometry(path, 5.0);

            var projected = PathProjection.TryProjectToPath(path, geometry!, 18.0, 2.0, out var projection);

            Assert.That(projected, Is.True);
            Assert.That(projection.DistanceAlongPathMeters, Is.EqualTo(18.0).Within(0.001));
            Assert.That(projection.DistanceToPathMeters, Is.EqualTo(2.0).Within(0.001));
        }
    }
}
