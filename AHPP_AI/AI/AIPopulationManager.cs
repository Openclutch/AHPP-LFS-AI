using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using AHPP_AI.Debug;
using AHPP_AI.Waypoint;

namespace AHPP_AI.AI
{
    /// <summary>
    /// Controls the overall AI population and route distribution based on human presence and route metadata.
    /// </summary>
    public class AIPopulationManager : IDisposable
    {
        private readonly AIController aiController;
        private readonly PathManager pathManager;
        private readonly RouteLibrary routeLibrary;
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
            PathManager pathManager,
            RouteLibrary routeLibrary,
            AIConfig config,
            Logger logger)
        {
            this.aiController = aiController ?? throw new ArgumentNullException(nameof(aiController));
            this.pathManager = pathManager ?? throw new ArgumentNullException(nameof(pathManager));
            this.routeLibrary = routeLibrary ?? throw new ArgumentNullException(nameof(routeLibrary));
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
        /// Request reconciliation immediately (e.g., after config changes).
        /// </summary>
        public void RequestReconcile(string reason)
        {
            SafeReconcile(reason);
        }

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
                logger.LogException(ex, "AI population reconcile failed");
            }
        }

        /// <summary>
        /// Calculate the desired AI population and align per-route targets.
        /// </summary>
        private void ReconcilePopulation(string reason)
        {
            Dictionary<string, int> routeTargets;
            Dictionary<byte, string> assignments;
            Dictionary<string, int> routeCounts;
            List<byte> activePlids;
            int desiredTotal;

            lock (syncRoot)
            {
                if (disposed || !autoAdjustEnabled) return;

                var humanCount = humanUcids.Count;
                desiredTotal = CalculateDesiredTotal(humanCount);
                routeTargets = CalculateRouteTargets(desiredTotal);
                assignments = aiController.GetAssignedRoutesSnapshot();
                activePlids = aiController.GetActiveAiPlids();
                routeCounts = BuildRouteCounts(assignments, activePlids, aiController.MainRouteName);
            }

            var currentTotal = activePlids.Count;

            if (currentTotal > desiredTotal)
            {
                var toRemove = Math.Min(config.RemoveBatchSize, currentTotal - desiredTotal);
                RemoveAis(routeCounts, routeTargets, assignments, activePlids, toRemove);

                assignments = aiController.GetAssignedRoutesSnapshot();
                activePlids = aiController.GetActiveAiPlids();
                routeCounts = BuildRouteCounts(assignments, activePlids, aiController.MainRouteName);
                currentTotal = activePlids.Count;
            }

            if (currentTotal < desiredTotal)
            {
                var toSpawn = Math.Min(config.SpawnBatchSize, desiredTotal - currentTotal);
                var plannedRoutes = PlanSpawnRoutes(routeCounts, routeTargets, toSpawn, aiController.MainRouteName);
                if (plannedRoutes.Count > 0)
                {
                    aiController.SpawnPlannedAICars(plannedRoutes, false);
                }
                return;
            }

            BalanceRoutes(routeCounts, routeTargets, assignments, activePlids);
        }

        private void BalanceRoutes(
            Dictionary<string, int> routeCounts,
            Dictionary<string, int> routeTargets,
            Dictionary<byte, string> assignments,
            List<byte> activePlids)
        {
            var deficits = routeTargets
                .ToDictionary(kvp => kvp.Key, kvp => kvp.Value - GetCount(routeCounts, kvp.Key), StringComparer.OrdinalIgnoreCase);

            while (true)
            {
                var deficitRoute = deficits
                    .Where(kvp => kvp.Value > 0)
                    .OrderByDescending(kvp => kvp.Value)
                    .ThenBy(kvp => kvp.Key, StringComparer.OrdinalIgnoreCase)
                    .FirstOrDefault();

                if (deficitRoute.Equals(default(KeyValuePair<string, int>)) || deficitRoute.Value <= 0)
                    break;

                var surplusRoute = routeCounts
                    .Where(kvp => kvp.Value > GetCount(routeTargets, kvp.Key))
                    .OrderByDescending(kvp => kvp.Value - GetCount(routeTargets, kvp.Key))
                    .ThenBy(kvp => kvp.Key, StringComparer.OrdinalIgnoreCase)
                    .FirstOrDefault();

                if (surplusRoute.Equals(default(KeyValuePair<string, int>)) ||
                    surplusRoute.Value <= GetCount(routeTargets, surplusRoute.Key))
                    break;

                var plid = SelectPlidFromRoute(assignments, activePlids, surplusRoute.Key);
                if (plid == 0) break;

                aiController.SetAssignedRoute(plid, deficitRoute.Key);
                assignments[plid] = deficitRoute.Key;

                routeCounts[surplusRoute.Key] -= 1;
                routeCounts[deficitRoute.Key] = GetCount(routeCounts, deficitRoute.Key) + 1;
                deficits[deficitRoute.Key] -= 1;
            }
        }

        private List<string> PlanSpawnRoutes(
            Dictionary<string, int> routeCounts,
            Dictionary<string, int> routeTargets,
            int toSpawn,
            string mainRouteName)
        {
            var planned = new List<string>();
            if (toSpawn <= 0) return planned;

            var deficits = routeTargets
                .ToDictionary(kvp => kvp.Key, kvp => kvp.Value - GetCount(routeCounts, kvp.Key), StringComparer.OrdinalIgnoreCase);

            for (var i = 0; i < toSpawn; i++)
            {
                var nextRoute = deficits
                    .Where(kvp => kvp.Value > 0)
                    .OrderByDescending(kvp => kvp.Value)
                    .ThenBy(kvp => kvp.Key, StringComparer.OrdinalIgnoreCase)
                    .Select(kvp => kvp.Key)
                    .FirstOrDefault();

                if (string.IsNullOrWhiteSpace(nextRoute))
                    nextRoute = mainRouteName;

                planned.Add(nextRoute);
                if (deficits.ContainsKey(nextRoute)) deficits[nextRoute] -= 1;
            }

            return planned;
        }

        private void RemoveAis(
            Dictionary<string, int> routeCounts,
            Dictionary<string, int> routeTargets,
            Dictionary<byte, string> assignments,
            List<byte> activePlids,
            int toRemove)
        {
            for (var i = 0; i < toRemove; i++)
            {
                var surplusRoute = routeCounts
                    .Where(kvp => kvp.Value > GetCount(routeTargets, kvp.Key))
                    .OrderByDescending(kvp => kvp.Value - GetCount(routeTargets, kvp.Key))
                    .ThenBy(kvp => kvp.Key, StringComparer.OrdinalIgnoreCase)
                    .Select(kvp => kvp.Key)
                    .FirstOrDefault();

                var routeName = string.IsNullOrWhiteSpace(surplusRoute)
                    ? assignments.Values.FirstOrDefault()
                    : surplusRoute;

                if (string.IsNullOrWhiteSpace(routeName))
                    continue;

                var plid = SelectPlidFromRoute(assignments, activePlids, routeName);
                if (plid == 0 && assignments.Count > 0)
                {
                    plid = assignments.Keys.First();
                }

                if (plid == 0) return;

                aiController.RemoveAICar(plid, false);
                assignments.Remove(plid);

                if (!string.IsNullOrWhiteSpace(routeName) && routeCounts.ContainsKey(routeName))
                    routeCounts[routeName] -= 1;
            }
        }

        private byte SelectPlidFromRoute(
            Dictionary<byte, string> assignments,
            List<byte> activePlids,
            string routeName)
        {
            if (string.IsNullOrWhiteSpace(routeName)) return 0;

            foreach (var plid in activePlids)
            {
                if (assignments.TryGetValue(plid, out var assignedRoute) &&
                    assignedRoute.Equals(routeName, StringComparison.OrdinalIgnoreCase))
                    return plid;
            }

            return 0;
        }

        private Dictionary<string, int> BuildRouteCounts(
            Dictionary<byte, string> assignments,
            IEnumerable<byte> activePlids,
            string mainRouteName)
        {
            var counts = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);

            foreach (var plid in activePlids)
            {
                var route = assignments.TryGetValue(plid, out var assigned) && !string.IsNullOrWhiteSpace(assigned)
                    ? assigned
                    : mainRouteName;
                var normalized = routeLibrary.NormalizeRouteName(route);
                counts[normalized] = GetCount(counts, normalized) + 1;
            }

            return counts;
        }

        private int CalculateDesiredTotal(int humanCount)
        {
            var availableSlots = Math.Max(0, config.MaxPlayers - config.ReservedSlots - humanCount);
            var desired = (int)Math.Round(availableSlots * config.AiFillRatio);
            desired = Math.Max(config.MinAIs, desired);
            desired = Math.Min(desired, Math.Min(config.MaxAIs, availableSlots));
            return desired;
        }

        private Dictionary<string, int> CalculateRouteTargets(int desiredTotal)
        {
            var routes = GetEligibleRoutes();
            var targets = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            foreach (var route in routes) targets[route.Name] = 0;

            if (desiredTotal <= 0 || routes.Count == 0) return targets;

            var remaining = desiredTotal;

            var fixedRoutes = routes
                .Where(r => r.Metadata.AiTargetCount.HasValue)
                .OrderBy(r => r.Name, StringComparer.OrdinalIgnoreCase);

            foreach (var route in fixedRoutes)
            {
                var desired = Math.Max(0, route.Metadata.AiTargetCount ?? 0);
                var allocation = Math.Min(desired, remaining);
                targets[route.Name] = allocation;
                remaining -= allocation;
            }

            var percentRoutes = routes
                .Where(r => !r.Metadata.AiTargetCount.HasValue && r.Metadata.AiTargetPercent.HasValue)
                .ToList();

            if (percentRoutes.Count > 0 && remaining > 0)
            {
                var percentBase = desiredTotal;
                var rawTotal = percentRoutes.Sum(r => Math.Max(0.0, r.Metadata.AiTargetPercent ?? 0) * percentBase);
                var scaledTotal = rawTotal;
                if (scaledTotal > remaining && scaledTotal > 0.0001)
                    scaledTotal = remaining;

                var scale = rawTotal > 0.0001 ? scaledTotal / rawTotal : 0;
                var targetTotal = (int)Math.Round(scaledTotal);

                var percentAllocations = AllocateWithLargestRemainder(
                    percentRoutes,
                    r => (r.Metadata.AiTargetPercent ?? 0) * percentBase * scale,
                    targetTotal);

                foreach (var kvp in percentAllocations) targets[kvp.Key] = kvp.Value;
                remaining -= percentAllocations.Values.Sum();
            }

            var weightRoutes = routes
                .Where(r => !r.Metadata.AiTargetCount.HasValue && !r.Metadata.AiTargetPercent.HasValue)
                .Where(r => (r.Metadata.AiWeight ?? 1.0) > 0)
                .ToList();

            if (weightRoutes.Count > 0 && remaining > 0)
            {
                var totalWeight = Math.Max(0.0001, weightRoutes.Sum(w => w.Metadata.AiWeight ?? 1.0));
                var weightAllocations = AllocateWithLargestRemainder(
                    weightRoutes,
                    r => remaining * (r.Metadata.AiWeight ?? 1.0) / totalWeight,
                    remaining);

                foreach (var kvp in weightAllocations) targets[kvp.Key] = kvp.Value;
                remaining -= weightAllocations.Values.Sum();
            }

            var assignedTotal = targets.Values.Sum();
            var shortfall = desiredTotal - assignedTotal;
            if (shortfall > 0)
            {
                foreach (var route in routes.OrderBy(r => r.Name, StringComparer.OrdinalIgnoreCase))
                {
                    targets[route.Name] += 1;
                    shortfall--;
                    if (shortfall <= 0) break;
                }
            }

            return targets;
        }

        private Dictionary<string, int> AllocateWithLargestRemainder(
            IEnumerable<RouteAllocation> routes,
            Func<RouteAllocation, double> rawSelector,
            int targetTotal)
        {
            var allocations = new List<RouteAllocationResult>();
            foreach (var route in routes)
            {
                var raw = Math.Max(0, rawSelector(route));
                var floor = (int)Math.Floor(raw);
                allocations.Add(new RouteAllocationResult(route, raw, floor));
            }

            var result = allocations.ToDictionary(
                a => a.Route.Name,
                a => a.Floor,
                StringComparer.OrdinalIgnoreCase);

            var floorSum = allocations.Sum(a => a.Floor);
            var remaining = Math.Max(0, targetTotal - floorSum);

            if (floorSum > targetTotal)
            {
                var toTrim = floorSum - targetTotal;
                foreach (var allocation in allocations
                             .OrderBy(a => a.Remainder)
                             .ThenBy(a => a.Route.Name, StringComparer.OrdinalIgnoreCase))
                {
                    if (toTrim <= 0) break;
                    if (result[allocation.Route.Name] <= 0) continue;
                    result[allocation.Route.Name] -= 1;
                    toTrim--;
                }
                return result;
            }

            foreach (var allocation in allocations
                         .OrderByDescending(a => a.Remainder)
                         .ThenBy(a => a.Route.Name, StringComparer.OrdinalIgnoreCase))
            {
                if (remaining <= 0) break;
                result[allocation.Route.Name] += 1;
                remaining--;
            }

            return result;
        }

        private List<RouteAllocation> GetEligibleRoutes()
        {
            var routes = new List<RouteAllocation>();
            var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

            AddIfEligible(pathManager.MainRouteMetadata, routes, seen);
            if (pathManager.MainAlternateRoute?.Metadata != null)
                AddIfEligible(pathManager.MainAlternateRoute.Metadata, routes, seen);

            foreach (var branch in pathManager.GetBranches())
            {
                if (branch?.Metadata != null)
                    AddIfEligible(branch.Metadata, routes, seen);
            }

            if (routes.Count == 0 && !string.IsNullOrWhiteSpace(aiController.MainRouteName))
            {
                routes.Add(new RouteAllocation(
                    routeLibrary.NormalizeRouteName(aiController.MainRouteName),
                    new RouteMetadata
                    {
                        Name = aiController.MainRouteName,
                        Type = RouteType.MainLoop,
                        AiWeight = 1.0,
                        AiEnabled = true
                    }));
            }

            return routes;
        }

        private void AddIfEligible(RouteMetadata metadata, ICollection<RouteAllocation> routes, ISet<string> seen)
        {
            if (metadata == null) return;
            if (!metadata.AiEnabled) return;
            if (metadata.Type != RouteType.MainLoop &&
                metadata.Type != RouteType.AlternateMain &&
                metadata.Type != RouteType.Detour)
                return;

            var name = routeLibrary.NormalizeRouteName(metadata.Name);
            if (seen.Contains(name)) return;

            seen.Add(name);
            routes.Add(new RouteAllocation(name, metadata));
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

        private class RouteAllocation
        {
            public RouteAllocation(string name, RouteMetadata metadata)
            {
                Name = name;
                Metadata = metadata ?? new RouteMetadata { Name = name, Type = RouteType.MainLoop, AiWeight = 1.0 };
            }

            public string Name { get; }
            public RouteMetadata Metadata { get; }
        }

        /// <summary>
        /// Read a value from a dictionary while defaulting missing entries to zero.
        /// </summary>
        private static int GetCount(Dictionary<string, int> map, string key)
        {
            if (map == null || string.IsNullOrWhiteSpace(key)) return 0;
            return map.TryGetValue(key, out var value) ? value : 0;
        }

        private class RouteAllocationResult
        {
            public RouteAllocationResult(RouteAllocation route, double raw, int floor)
            {
                Route = route;
                Raw = raw;
                Floor = floor;
                Remainder = raw - floor;
            }

            public RouteAllocation Route { get; }
            public double Raw { get; }
            public int Floor { get; }
            public double Remainder { get; }
        }
    }
}
