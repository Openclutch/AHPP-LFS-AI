using System.Collections.Generic;
using AHPP_AI.AI;
using AHPP_AI.Util;
using NUnit.Framework;

namespace AHPP_AI_Tests
{
    /// <summary>
    ///     Tests for deterministic initial route candidate creation and selection.
    /// </summary>
    [TestFixture]
    public class InitialRoutePlannerTests
    {
        /// <summary>
        ///     Preferred spawn route should be kept when it still plausibly matches the live spawn.
        /// </summary>
        [Test]
        public void SelectSpawnRouteCandidate_PrefersPlannedRoute_WhenCloseEnough()
        {
            var preferred = new RouteCandidate { RouteName = "area_a_pit", Score = 20.0, NearestDistance = 25.0 };
            var live = new RouteCandidate { RouteName = "area_b_pit", Score = 19.0, NearestDistance = 30.0 };

            var result = InitialRoutePlanner.SelectSpawnRouteCandidate(preferred, 30.0, live, 5.0, 40.0, 20.0);

            Assert.That(result.SelectedCandidate, Is.SameAs(preferred));
            Assert.That(result.ReasonCode, Is.EqualTo("spawn-preferred-close-enough"));
        }

        /// <summary>
        ///     A much closer live route should override the planned route when the preferred route is implausibly far.
        /// </summary>
        [Test]
        public void SelectSpawnRouteCandidate_FallsBackToLiveRoute_WhenPreferredIsFar()
        {
            var preferred = new RouteCandidate { RouteName = "area_a_pit", Score = 20.0, NearestDistance = 120.0 };
            var live = new RouteCandidate { RouteName = "area_b_pit", Score = 25.0, NearestDistance = 15.0 };

            var result = InitialRoutePlanner.SelectSpawnRouteCandidate(preferred, 90.0, live, 10.0, 40.0, 20.0);

            Assert.That(result.SelectedCandidate, Is.SameAs(live));
            Assert.That(result.ReasonCode, Is.EqualTo("spawn-live-overrides-preferred"));
        }

        /// <summary>
        ///     On-track routes should win when the spawn route is very far and the score advantage is meaningful.
        /// </summary>
        [Test]
        public void SelectInitialRoute_PicksTrackCandidate_WhenSpawnCandidateIsFarAndWeak()
        {
            var spawn = new RouteCandidate { RouteName = "pit_a", Score = 80.0, NearestDistance = 150.0 };
            var track = new RouteCandidate { RouteName = "highway_route_loop", Score = 40.0, NearestDistance = 15.0 };

            var result = InitialRoutePlanner.SelectInitialRoute(spawn, track, 120.0, 25.0, 8.0);

            Assert.That(result.SelectedCandidate, Is.SameAs(track));
            Assert.That(result.ReasonCode, Is.EqualTo("initial-track-overrides-far-spawn"));
        }

        /// <summary>
        ///     Overly large heading errors should reject on-track candidates.
        /// </summary>
        [Test]
        public void IsViableTrackCandidate_RejectsLargeHeadingError()
        {
            var candidate = new RouteCandidate { RouteName = "highway_route_loop", NearestDistance = 20.0, HeadingError = 25000 };

            var viable = InitialRoutePlanner.IsViableTrackCandidate(candidate, 120.0, 100.0);

            Assert.That(viable, Is.False);
        }

        /// <summary>
        ///     Excessive distance should reject on-track candidates even with a reasonable heading match.
        /// </summary>
        [Test]
        public void IsViableTrackCandidate_RejectsLargeDistance()
        {
            var candidate = new RouteCandidate { RouteName = "highway_route_loop", NearestDistance = 140.0, HeadingError = 2000 };

            var viable = InitialRoutePlanner.IsViableTrackCandidate(candidate, 120.0, 100.0);

            Assert.That(viable, Is.False);
        }

        /// <summary>
        ///     Spawn-route candidates farther than the configured limit should be rejected outright.
        /// </summary>
        [Test]
        public void IsViableSpawnCandidate_RejectsRouteBeyondOneHundredMeters()
        {
            var candidate = new RouteCandidate { RouteName = "tram_pit", NearestDistance = 353.1 };

            var viable = InitialRoutePlanner.IsViableSpawnCandidate(candidate, 100.0);

            Assert.That(viable, Is.False);
        }

        /// <summary>
        ///     Nearby spawn-route candidates should remain eligible for classification.
        /// </summary>
        [Test]
        public void IsViableSpawnCandidate_AllowsNearbyRoute()
        {
            var candidate = new RouteCandidate { RouteName = "taf_pit", NearestDistance = 0.2 };

            var viable = InitialRoutePlanner.IsViableSpawnCandidate(candidate, 100.0);

            Assert.That(viable, Is.True);
        }

        /// <summary>
        ///     Candidate creation should reverse paths and remap the start index when the route fit says to drive against
        ///     the recorded order.
        /// </summary>
        [Test]
        public void CreateCandidate_ReversesPathAndRemapsStartIndex_WhenCounterClockwiseIsSelected()
        {
            var path = TestRouteFixtures.StraightPath(0.0, 10.0, 20.0);
            var position = Vec.FromMeters(9.0, 0.0);

            var candidate = InitialRoutePlanner.CreateCandidate(
                "spawn",
                "warehouse_pit",
                path,
                null,
                true,
                false,
                AIConfig.DrivingMode.Cruise,
                null,
                position,
                0,
                (pos, heading, routePath, isLoop, preferLoop) => (10.0, 1, false),
                TestRouteFixtures.DistanceToWaypoint);

            Assert.That(candidate, Is.Not.Null);
            Assert.That(candidate!.StartIndex, Is.EqualTo(1));
            Assert.That(candidate.Path[0].Position.X, Is.EqualTo(path[2].Position.X));
            Assert.That(candidate.Path[2].Position.X, Is.EqualTo(path[0].Position.X));
        }

        /// <summary>
        ///     Characterization test for the configured track-score advantage threshold.
        /// </summary>
        [Test]
        public void SelectInitialRoute_KeepsSpawnCandidate_WhenTrackAdvantageIsTooSmall()
        {
            var spawn = new RouteCandidate { RouteName = "pit_a", Score = 50.0, NearestDistance = 25.0 };
            var track = new RouteCandidate { RouteName = "main_loop", Score = 43.0, NearestDistance = 12.0 };

            var result = InitialRoutePlanner.SelectInitialRoute(spawn, track, 120.0, 25.0, 8.0);

            Assert.That(result.SelectedCandidate, Is.SameAs(spawn));
            Assert.That(result.ReasonCode, Is.EqualTo("initial-spawn-kept"));
        }
    }
}
