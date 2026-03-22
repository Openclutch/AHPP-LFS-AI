using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using AHPP_AI.Debug;

namespace AHPP_AI.AI
{
    /// <summary>
    /// Controls the overall AI population and segment distribution based on human presence and pit-area tags.
    /// </summary>
    public class AIPopulationManager : IDisposable
    {
        private readonly AIController aiController;
        private readonly AIConfig config;
        private readonly Logger logger;
        private readonly object syncRoot = new object();
        private readonly HashSet<int> humanUcids = new HashSet<int>();
        private readonly Dictionary<byte, int> humanPlidToUcid = new Dictionary<byte, int>();
        private Timer? adjustTimer;
        private bool disposed;
        private bool autoAdjustEnabled = true;

        public AIPopulationManager(
            AIController aiController,
            AIConfig config,
            Logger logger)
        {
            this.aiController = aiController ?? throw new ArgumentNullException(nameof(aiController));
            this.config = config ?? throw new ArgumentNullException(nameof(config));
            this.logger = logger ?? throw new ArgumentNullException(nameof(logger));

            RefreshSettings();
        }

        /// <summary>
        /// Update timer intervals to match current configuration.
        /// </summary>
        public void RefreshSettings()
        {
            lock (syncRoot)
            {
                if (disposed) return;

                var interval = Math.Max(1000, config.AdjustIntervalMs);
                if (adjustTimer == null)
                {
                    adjustTimer = new Timer(_ => SafeReconcile("timer"), null, interval, interval);
                }
                else
                {
                    adjustTimer.Change(interval, interval);
                }
            }
        }

        /// <summary>
        /// Enable or disable automatic AI population adjustments.
        /// </summary>
        public void SetAutoAdjustEnabled(bool enabled, bool reconcileNow)
        {
            var shouldReconcile = false;

            lock (syncRoot)
            {
                if (disposed) return;
                autoAdjustEnabled = enabled;
                shouldReconcile = enabled && reconcileNow;
            }

            if (shouldReconcile)
                SafeReconcile("manual-enable");
        }

        /// <summary>
        /// Report whether automatic AI adjustments are currently active.
        /// </summary>
        public bool AutoAdjustEnabled
        {
            get
            {
                lock (syncRoot)
                {
                    return autoAdjustEnabled;
                }
            }
        }

        /// <summary>
        /// Track a new human connection to keep player counts current.
        /// </summary>
        public void RegisterConnection(int ucid)
        {
            lock (syncRoot)
            {
                if (disposed) return;
                humanUcids.Add(ucid);
            }
            SafeReconcile("connection");
        }

        /// <summary>
        /// Track a human player joining with a specific PLID.
        /// </summary>
        public void RegisterHuman(int ucid, byte plid)
        {
            lock (syncRoot)
            {
                if (disposed) return;
                humanUcids.Add(ucid);
                if (plid != 0) humanPlidToUcid[plid] = ucid;
            }
            SafeReconcile("human join");
        }

        /// <summary>
        /// Remove a human from tracking when they leave by PLID or UCID.
        /// </summary>
        public void UnregisterHuman(byte plid, int? ucidOverride = null)
        {
            lock (syncRoot)
            {
                if (disposed) return;

                if (ucidOverride.HasValue)
                {
                    humanUcids.Remove(ucidOverride.Value);
                    if (plid != 0) humanPlidToUcid.Remove(plid);
                }
                else if (plid != 0 && humanPlidToUcid.TryGetValue(plid, out var ucid))
                {
                    humanPlidToUcid.Remove(plid);
                    humanUcids.Remove(ucid);
                }
            }
            SafeReconcile("human leave");
        }

        /// <summary>
        /// Remove all tracked humans for a UCID when a connection closes.
        /// </summary>
        public void OnConnectionClosed(int ucid)
        {
            lock (syncRoot)
            {
                if (disposed) return;

                humanUcids.Remove(ucid);
                var toRemove = humanPlidToUcid
                    .Where(kvp => kvp.Value == ucid)
                    .Select(kvp => kvp.Key)
                    .ToList();
                foreach (var plid in toRemove) humanPlidToUcid.Remove(plid);
            }
            SafeReconcile("connection closed");
        }

        /// <summary>
        /// Trigger reconciliation when an AI joins the server.
        /// </summary>
        public void OnAiJoined(byte plid)
        {
            SafeReconcile("ai join");
        }

        /// <summary>
        /// Trigger reconciliation when an AI leaves the server.
        /// </summary>
        public void OnAiLeft(byte plid)
        {
            SafeReconcile("ai leave");
        }

        /// <summary>
        /// Request reconciliation immediately (for example after config changes).
        /// </summary>
        public void RequestReconcile(string reason)
        {
            SafeReconcile(reason);
        }

        /// <summary>
        /// Guard reconciliation so timer callbacks and event handlers can fail safely.
        /// </summary>
        private void SafeReconcile(string reason)
        {
            lock (syncRoot)
            {
                if (disposed || !autoAdjustEnabled) return;
            }

            try
            {
                ReconcilePopulation(reason);
            }
            catch (Exception ex)
            {
                logger.LogException(ex, $"AI population reconcile failed ({reason})");
            }
        }

        /// <summary>
        /// Calculate the desired AI population and align counts to configured pit-area segments.
        /// </summary>
        private void ReconcilePopulation(string reason)
        {
            Dictionary<string, int> segmentTargets;
            Dictionary<byte, string> assignments;
            Dictionary<string, int> segmentCounts;
            List<byte> activePlids;
            int desiredTotal;

            lock (syncRoot)
            {
                if (disposed || !autoAdjustEnabled) return;

                var humanCount = humanUcids.Count;
                desiredTotal = CalculateDesiredTotal(humanCount);
                segmentTargets = CalculateSegmentTargets(desiredTotal);
                assignments = aiController.GetAssignedSegmentsSnapshot();
                activePlids = aiController.GetActiveAiPlids();
                segmentCounts = BuildSegmentCounts(assignments, activePlids, segmentTargets.Keys);
            }

            var currentTotal = activePlids.Count;
            var removedForRedistribution = 0;

            if (currentTotal > desiredTotal)
            {
                var toRemove = Math.Min(config.RemoveBatchSize, currentTotal - desiredTotal);
                RemoveAis(segmentCounts, segmentTargets, assignments, activePlids, toRemove);

                assignments = aiController.GetAssignedSegmentsSnapshot();
                activePlids = aiController.GetActiveAiPlids();
                segmentCounts = BuildSegmentCounts(assignments, activePlids, segmentTargets.Keys);
                currentTotal = activePlids.Count;
            }
            else if (currentTotal == desiredTotal)
            {
                removedForRedistribution = Math.Min(
                    config.RemoveBatchSize,
                    CalculateRedistributionCount(segmentCounts, segmentTargets));
                if (removedForRedistribution > 0)
                {
                    RemoveAis(segmentCounts, segmentTargets, assignments, activePlids, removedForRedistribution);

                    assignments = aiController.GetAssignedSegmentsSnapshot();
                    activePlids = aiController.GetActiveAiPlids();
                    segmentCounts = BuildSegmentCounts(assignments, activePlids, segmentTargets.Keys);
                    currentTotal = activePlids.Count;
                }
            }

            if (currentTotal < desiredTotal)
            {
                var spawnBudget = desiredTotal - currentTotal;
                if (removedForRedistribution > 0)
                    spawnBudget = Math.Max(spawnBudget, removedForRedistribution);

                var toSpawn = Math.Min(config.SpawnBatchSize, spawnBudget);
                var plannedSegments = PlanSpawnSegments(segmentCounts, segmentTargets, toSpawn);
                if (plannedSegments.Count > 0)
                {
                    aiController.SpawnPlannedAICars(plannedSegments, true);
                }
            }
        }

        /// <summary>
        /// Count how many removals are needed to migrate AI from surplus segments into deficit segments.
        /// </summary>
        private int CalculateRedistributionCount(
            Dictionary<string, int> segmentCounts,
            Dictionary<string, int> segmentTargets)
        {
            var surplusTotal = segmentCounts.Sum(kvp => Math.Max(0, kvp.Value - GetCount(segmentTargets, kvp.Key)));
            var deficitTotal = segmentTargets.Sum(kvp => Math.Max(0, kvp.Value - GetCount(segmentCounts, kvp.Key)));
            return Math.Min(surplusTotal, deficitTotal);
        }

        /// <summary>
        /// Plan which segments should receive the next spawned AI cars.
        /// </summary>
        private List<string> PlanSpawnSegments(
            Dictionary<string, int> segmentCounts,
            Dictionary<string, int> segmentTargets,
            int toSpawn)
        {
            var planned = new List<string>();
            if (toSpawn <= 0) return planned;

            var deficits = segmentTargets
                .ToDictionary(kvp => kvp.Key, kvp => kvp.Value - GetCount(segmentCounts, kvp.Key), StringComparer.OrdinalIgnoreCase);

            for (var i = 0; i < toSpawn; i++)
            {
                var nextSegment = deficits
                    .Where(kvp => kvp.Value > 0)
                    .OrderByDescending(kvp => kvp.Value)
                    .ThenBy(kvp => kvp.Key, StringComparer.OrdinalIgnoreCase)
                    .Select(kvp => kvp.Key)
                    .FirstOrDefault();

                if (string.IsNullOrWhiteSpace(nextSegment))
                    nextSegment = ResolveFallbackSegmentTag();
                if (string.IsNullOrWhiteSpace(nextSegment))
                    break;

                planned.Add(nextSegment);
                if (deficits.ContainsKey(nextSegment)) deficits[nextSegment] -= 1;
            }

            return planned;
        }

        /// <summary>
        /// Remove AI cars from the most overfilled segments first.
        /// </summary>
        private void RemoveAis(
            Dictionary<string, int> segmentCounts,
            Dictionary<string, int> segmentTargets,
            Dictionary<byte, string> assignments,
            List<byte> activePlids,
            int toRemove)
        {
            for (var i = 0; i < toRemove; i++)
            {
                var surplusSegment = segmentCounts
                    .Where(kvp => kvp.Value > GetCount(segmentTargets, kvp.Key))
                    .OrderByDescending(kvp => kvp.Value - GetCount(segmentTargets, kvp.Key))
                    .ThenBy(kvp => kvp.Key, StringComparer.OrdinalIgnoreCase)
                    .Select(kvp => kvp.Key)
                    .FirstOrDefault();

                var segmentTag = string.IsNullOrWhiteSpace(surplusSegment)
                    ? assignments.Values.FirstOrDefault()
                    : surplusSegment;

                if (string.IsNullOrWhiteSpace(segmentTag))
                    continue;

                var plid = SelectPlidFromSegment(assignments, activePlids, segmentTag);
                if (plid == 0 && assignments.Count > 0)
                    plid = assignments.Keys.First();
                if (plid == 0)
                    return;

                aiController.RemoveAICar(plid, false);
                assignments.Remove(plid);

                if (segmentCounts.ContainsKey(segmentTag))
                    segmentCounts[segmentTag] -= 1;
            }
        }

        /// <summary>
        /// Pick an AI currently assigned to the specified segment.
        /// </summary>
        private byte SelectPlidFromSegment(
            Dictionary<byte, string> assignments,
            List<byte> activePlids,
            string segmentTag)
        {
            if (string.IsNullOrWhiteSpace(segmentTag)) return 0;

            foreach (var plid in activePlids)
            {
                if (assignments.TryGetValue(plid, out var assignedSegment) &&
                    assignedSegment.Equals(segmentTag, StringComparison.OrdinalIgnoreCase))
                    return plid;
            }

            return 0;
        }

        /// <summary>
        /// Build current AI counts per configured segment tag.
        /// </summary>
        private Dictionary<string, int> BuildSegmentCounts(
            Dictionary<byte, string> assignments,
            IEnumerable<byte> activePlids,
            IEnumerable<string> configuredTags)
        {
            var counts = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            foreach (var tag in configuredTags.Where(tag => !string.IsNullOrWhiteSpace(tag)))
                counts[tag] = 0;

            foreach (var plid in activePlids)
            {
                var segmentTag = assignments.TryGetValue(plid, out var assigned) && !string.IsNullOrWhiteSpace(assigned)
                    ? assigned
                    : ResolveFallbackSegmentTag();
                if (string.IsNullOrWhiteSpace(segmentTag))
                    continue;

                counts[segmentTag] = GetCount(counts, segmentTag) + 1;
            }

            return counts;
        }

        /// <summary>
        /// Calculate the total desired AI count after reserving player buffer slots.
        /// </summary>
        private int CalculateDesiredTotal(int humanCount)
        {
            var availableSlots = Math.Max(0, config.MaxPlayers - config.ReservedSlots - humanCount);
            var desired = (int)Math.Round(availableSlots * config.AiFillRatio);
            desired = Math.Max(config.MinAIs, desired);
            desired = Math.Min(desired, Math.Min(config.MaxAIs, availableSlots));
            return desired;
        }

        /// <summary>
        /// Translate segment fill percentages into concrete AI targets for the current desired total.
        /// </summary>
        private Dictionary<string, int> CalculateSegmentTargets(int desiredTotal)
        {
            var segments = GetEligibleSegments();
            var targets = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            foreach (var segment in segments) targets[segment.Tag] = 0;

            if (desiredTotal <= 0)
                return targets;

            if (segments.Count == 0)
            {
                var fallbackTag = ResolveFallbackSegmentTag();
                if (!string.IsNullOrWhiteSpace(fallbackTag))
                    targets[fallbackTag] = desiredTotal;
                return targets;
            }

            var totalPercent = Math.Max(0.0001, segments.Sum(segment => Math.Max(0.0, segment.FillPercent)));
            var allocations = AllocateWithLargestRemainder(
                segments,
                segment => desiredTotal * Math.Max(0.0, segment.FillPercent) / totalPercent,
                desiredTotal);

            foreach (var kvp in allocations)
                targets[kvp.Key] = kvp.Value;

            return targets;
        }

        /// <summary>
        /// Return the configured population segments that actively participate in auto-population.
        /// </summary>
        private List<SegmentAllocation> GetEligibleSegments()
        {
            return config.PitSpawnAreas.Values
                .Where(area => area != null &&
                               !string.IsNullOrWhiteSpace(area.Tag) &&
                               area.FillPercent > 0.0)
                .OrderBy(area => area.Tag, StringComparer.OrdinalIgnoreCase)
                .Select(area => new SegmentAllocation(area.Tag, area.FillPercent))
                .ToList();
        }

        /// <summary>
        /// Pick the default segment tag used for fallback counting and spawning.
        /// </summary>
        private string ResolveFallbackSegmentTag()
        {
            var configured = config.PitSpawnAreas.Values
                .Where(area => area != null && !string.IsNullOrWhiteSpace(area.Tag))
                .OrderByDescending(area => area.FillPercent)
                .ThenBy(area => area.Tag, StringComparer.OrdinalIgnoreCase)
                .FirstOrDefault();

            return configured?.Tag ?? string.Empty;
        }

        /// <summary>
        /// Allocate integer targets from fractional desired values using largest remainder rounding.
        /// </summary>
        private Dictionary<string, int> AllocateWithLargestRemainder(
            IEnumerable<SegmentAllocation> segments,
            Func<SegmentAllocation, double> rawSelector,
            int targetTotal)
        {
            var allocations = new List<SegmentAllocationResult>();
            foreach (var segment in segments)
            {
                var raw = Math.Max(0, rawSelector(segment));
                var floor = (int)Math.Floor(raw);
                allocations.Add(new SegmentAllocationResult(segment, raw, floor));
            }

            var result = allocations.ToDictionary(
                allocation => allocation.Segment.Tag,
                allocation => allocation.Floor,
                StringComparer.OrdinalIgnoreCase);

            var floorSum = allocations.Sum(allocation => allocation.Floor);
            var remaining = Math.Max(0, targetTotal - floorSum);

            if (floorSum > targetTotal)
            {
                var toTrim = floorSum - targetTotal;
                foreach (var allocation in allocations
                             .OrderBy(allocation => allocation.Remainder)
                             .ThenBy(allocation => allocation.Segment.Tag, StringComparer.OrdinalIgnoreCase))
                {
                    if (toTrim <= 0) break;
                    if (result[allocation.Segment.Tag] <= 0) continue;
                    result[allocation.Segment.Tag] -= 1;
                    toTrim--;
                }
                return result;
            }

            foreach (var allocation in allocations
                         .OrderByDescending(allocation => allocation.Remainder)
                         .ThenBy(allocation => allocation.Segment.Tag, StringComparer.OrdinalIgnoreCase))
            {
                if (remaining <= 0) break;
                result[allocation.Segment.Tag] += 1;
                remaining--;
            }

            return result;
        }

        public void Dispose()
        {
            lock (syncRoot)
            {
                if (disposed) return;
                disposed = true;
                adjustTimer?.Dispose();
            }
        }

        /// <summary>
        /// Read a value from a dictionary while defaulting missing entries to zero.
        /// </summary>
        private static int GetCount(Dictionary<string, int> map, string key)
        {
            if (map == null || string.IsNullOrWhiteSpace(key)) return 0;
            return map.TryGetValue(key, out var value) ? value : 0;
        }

        private sealed class SegmentAllocation
        {
            public SegmentAllocation(string tag, double fillPercent)
            {
                Tag = tag;
                FillPercent = fillPercent;
            }

            public string Tag { get; }
            public double FillPercent { get; }
        }

        private sealed class SegmentAllocationResult
        {
            public SegmentAllocationResult(SegmentAllocation segment, double raw, int floor)
            {
                Segment = segment;
                Raw = raw;
                Floor = floor;
            }

            public SegmentAllocation Segment { get; }
            public double Raw { get; }
            public int Floor { get; }
            public double Remainder => Raw - Floor;
        }
    }
}
