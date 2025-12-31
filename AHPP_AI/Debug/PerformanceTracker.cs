using System;
using System.Diagnostics;

namespace AHPP_AI.Debug
{
    /// <summary>
    ///     Tracks execution timings and produces rolling snapshots for logging.
    /// </summary>
    public class PerformanceTracker
    {
        private readonly object sync = new object();
        private long totalTicks;
        private int count;
        private long maxTicks;
        private DateTime windowStart = DateTime.UtcNow;

        /// <summary>
        ///     Record a duration sample for the current window.
        /// </summary>
        public void Record(TimeSpan duration)
        {
            lock (sync)
            {
                totalTicks += duration.Ticks;
                count++;
                if (duration.Ticks > maxTicks) maxTicks = duration.Ticks;
            }
        }

        /// <summary>
        ///     Try to produce a snapshot if the window has elapsed and samples exist.
        /// </summary>
        public bool TryGetSnapshot(TimeSpan window, out PerformanceSnapshot snapshot)
        {
            lock (sync)
            {
                var elapsed = DateTime.UtcNow - windowStart;
                if (elapsed < window || count == 0)
                {
                    snapshot = default;
                    return false;
                }

                snapshot = new PerformanceSnapshot(count, totalTicks, maxTicks, elapsed);
                windowStart = DateTime.UtcNow;
                totalTicks = 0;
                count = 0;
                maxTicks = 0;
                return true;
            }
        }
    }

    /// <summary>
    ///     Aggregated timing metrics for a rolling window.
    /// </summary>
    public readonly struct PerformanceSnapshot
    {
        public PerformanceSnapshot(int count, long totalTicks, long maxTicks, TimeSpan elapsed)
        {
            Count = count;
            TotalTicks = totalTicks;
            MaxTicks = maxTicks;
            Elapsed = elapsed;
        }

        public int Count { get; }
        public long TotalTicks { get; }
        public long MaxTicks { get; }
        public TimeSpan Elapsed { get; }

        public double AverageMs => Count == 0 ? 0 : (TotalTicks / (double)Count) / TimeSpan.TicksPerMillisecond;
        public double MaxMs => MaxTicks / (double)TimeSpan.TicksPerMillisecond;
        public double TotalMs => TotalTicks / (double)TimeSpan.TicksPerMillisecond;
        public double RatePerSecond => Elapsed.TotalSeconds > 0 ? Count / Elapsed.TotalSeconds : 0;
    }
}
