using AHPP_AI.AI;
using AHPP_AI.Util;
using NUnit.Framework;

namespace AHPP_AI_Tests
{
    /// <summary>
    ///     Tests for deterministic spawn-route handoff policy.
    /// </summary>
    [TestFixture]
    public class SpawnRouteTransitionPlannerTests
    {
        /// <summary>
        ///     Non-loop spawn routes should remain active until the car is near the end of the path.
        /// </summary>
        [Test]
        public void Evaluate_StaysOnSpawnRoute_WhenDistanceToEndIsAboveThreshold()
        {
            var path = TestRouteFixtures.StraightPath(0.0, 10.0, 20.0, 30.0);

            var result = SpawnRouteTransitionPlanner.Evaluate(
                path,
                5.0,
                0.0,
                10.0,
                5.0,
                "warehouse_pit",
                _ => "warehouse_route_loop");

            Assert.That(result.Action, Is.EqualTo(SpawnTransitionAction.StayOnSpawnRoute));
            Assert.That(result.ReasonCode, Is.EqualTo("spawn-continue"));
            Assert.That(result.DistanceToEndMeters, Is.GreaterThan(10.0));
        }

        /// <summary>
        ///     Looping spawn paths should be treated as transition-ready because distance-to-end is not meaningful.
        /// </summary>
        [Test]
        public void Evaluate_TreatsLoopingSpawnRouteAsTransitionReady()
        {
            var path = TestRouteFixtures.SquareLoop();

            var result = SpawnRouteTransitionPlanner.Evaluate(
                path,
                1.0,
                1.0,
                10.0,
                5.0,
                "loop_pit",
                _ => "loop_route_loop");

            Assert.That(result.Action, Is.EqualTo(SpawnTransitionAction.TransitionToDrivingRoute));
            Assert.That(result.ReasonCode, Is.EqualTo("spawn-loop-route"));
            Assert.That(result.TargetRouteName, Is.EqualTo("loop_route_loop"));
        }

        /// <summary>
        ///     Projection failures should still resolve safely into a transition when a target route exists.
        /// </summary>
        [Test]
        public void Evaluate_TransitionsSafely_WhenProjectionFails()
        {
            var path = TestRouteFixtures.StraightPath(0.0, 0.0, 0.0);

            var result = SpawnRouteTransitionPlanner.Evaluate(
                path,
                50.0,
                50.0,
                10.0,
                5.0,
                "broken_pit",
                _ => "main_route_loop");

            Assert.That(result.Action, Is.EqualTo(SpawnTransitionAction.TransitionToDrivingRoute));
            Assert.That(result.ReasonCode, Is.EqualTo("spawn-geometry-missing"));
            Assert.That(result.TargetRouteName, Is.EqualTo("main_route_loop"));
        }

        /// <summary>
        ///     Missing driving routes should fail explicitly instead of silently transitioning nowhere.
        /// </summary>
        [Test]
        public void Evaluate_Fails_WhenNoDrivingRouteCanBeResolved()
        {
            var path = TestRouteFixtures.StraightPath(0.0, 10.0, 20.0, 30.0);

            var result = SpawnRouteTransitionPlanner.Evaluate(
                path,
                29.0,
                0.0,
                5.0,
                5.0,
                "warehouse_pit",
                _ => string.Empty);

            Assert.That(result.Action, Is.EqualTo(SpawnTransitionAction.Fail));
            Assert.That(result.ReasonCode, Is.EqualTo("spawn-target-missing"));
        }

        /// <summary>
        ///     Near the end of a non-loop spawn route, the policy should hand off to the resolved driving route.
        /// </summary>
        [Test]
        public void Evaluate_TransitionsToResolvedDrivingRoute_WhenNearRouteEnd()
        {
            var path = TestRouteFixtures.StraightPath(0.0, 10.0, 20.0, 30.0);

            var result = SpawnRouteTransitionPlanner.Evaluate(
                path,
                28.0,
                0.0,
                5.0,
                5.0,
                "warehouse_pit",
                _ => "warehouse_route_alt");

            Assert.That(result.Action, Is.EqualTo(SpawnTransitionAction.TransitionToDrivingRoute));
            Assert.That(result.ReasonCode, Is.EqualTo("spawn-transition-ready"));
            Assert.That(result.TargetRouteName, Is.EqualTo("warehouse_route_alt"));
        }
    }
}
