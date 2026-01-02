# AHPP AI System Manual

This document outlines how the AI traffic system in `AHPP_AI` operates, what it expects from configuration, and how to drive or extend it. It is based on the current implementation in the codebase (InSim-based .NET 4.8, C# 8, x86).

## High-Level Flow
- Program startup loads `config.ini`, initializes logging, route libraries, waypoint manager, and AI subsystems (steering, gearbox, lights, driver, population manager, and debug/UI helpers).
- Recorded traffic routes are loaded up-front (main loop, pit/spawn, alternate main lane, optional detours). If a route is missing, a circular fallback path is generated around the spawn point.
- InSim connects to the configured LFS host/port and OutGauge port, registers packet handlers (connection/player events, telemetry, layout/AXM, UI buttons, and state/version packets), then initializes debug UI and optional waypoint/axis visualization.
- When enabled, auto population reconciles AI counts after connection and continues adjusting based on human player counts.
- A background key listener exits cleanly on `q/Q`, calling a shutdown routine that despawns AI and disconnects InSim/OutGauge.

## Configuration Expectations (`config.ini`)
- **InSim/OutGauge**: `InSim.Host[Online|Local]`, `InSim.Port`, `InSim.OutGaugePort`.
- **Routes**: `Routes.Main`, `Routes.Spawn`, optional `Routes.MainAlt`, `Routes.Detour1..3`. Route names are normalized and loaded from `Routes/<track>/<layout>/`.
- **AI Driving**: spawn counts/delay (`AI.NumberOfAIs`, `AI.SpawnDelayMs`), waypoint lookahead and thresholds (`LookaheadWaypoints`, `WaypointProximityMultiplier`), steering dampening/deadzone and pure-pursuit parameters, clutch/gearbox timings (`ClutchHoldAfterShiftMs`, hysteresis buffers, shift delays), recovery timings and thresholds, collision detection distances/angles/width.
- **Population Management**: capacity and tuning (`MaxPlayers`, `ReservedSlots`, `AiFillRatio`, `MinAIs`, `MaxAIs`, `AdjustIntervalMs`, `SpawnBatchSize`, `RemoveBatchSize`, `AutoManagePopulation`).
- **Lane Changes**: merge chance, cooldowns, safety envelopes (distance/half-width, parallel distance/heading limits), speed-scaled parallel lookahead (`ParallelLookaheadSeconds`, `ParallelLookaheadMinMeters`), max merge speed, signal lead/min duration, transition length and point count.
- **Recording**: minimum meters between recorded points (`Recording.IntervalMeters`).
- **Debugging**: enable flags, waypoint/axis visualization toggles, verbose InSim logging, active waypoint marker cadence, performance logging.

## Route Management and Recording
- Routes are recorded via UI controls and saved with metadata (type, loop flag, attach/rejoin indices, default speed, AI weights). Supported types: main loop, alternate main lane, pit entry, detours.
- Recording writes JSON with per-point position (meters), heading, speed/limit, throttle/brake/steering. Chalk markers visualize captured points; UI buttons show live counts and recording state.
- PathManager loads spawn/main/alternate/branch routes, attaches branches using recorded indices or nearest-point matching, and validates for missing data, loop consistency, or invalid indices.
- WaypointManager serves traffic paths and finds the best starting waypoint aligned to the car heading; it reverses the path when needed to match travel direction.
- WaypointFollower selects targets, blends lookahead steering, and can generate an approach curve when joining a path near the track. Progress is monitored to trigger recovery if distance grows, motion stalls, or the car moves away.

## AI Driving Behavior
- **Steering**: Pure-pursuit optional; steering response damping and deadzone configurable. Lookahead adjusts based on speed; heading errors feed steering angle capped by `PurePursuitMaxSteerDegrees`.
- **Speed/Throttle/Brake**: Target speeds come from route nodes; manual overrides are supported via UI/button handlers. Collision detection scans a forward arc (range/angle/half-width) to reduce speed and maintain `MinimumSafetyDistanceM`.
- **Gearbox/Clutch**: Gear selection uses speed thresholds with hysteresis and minimum time in gear. Clutch state machine presses, latches gear, holds post-shift, then releases gradually. Low-RPM protection presses clutch and may downshift to prevent stalls. Gearbox starts in 1st with clutch held on spawn.
- **Recovery**: Unified state machine handles stalls (press clutch, ignition pulse, clutch release), short/long reverse maneuvers with cooldowns, validation windows, and failure escalation. Success is based on distance moved and speed; repeated failures trigger a pit/spectate reset via the controller.
- **Collision Handling**: Detects cars ahead using configurable corridor; raises movement issues for slow progress or stuck states to feed recovery logic.
- **Lights/Indicators**: AILightController manages headlights, high beams, hazards, horn, and indicators. Lane changes and branch merges set indicators with lead/minimum durations; indicators clear on completion.

## Lane Changes and Branching
- Alternate lane (MainAlt) and detour branches are loaded with metadata weights/targets. Route assignments are stored per-AI, and lane changes rebuild merge transitions using Bezier points for smooth path handoff.
- Highway merges between Main and MainAlt only proceed when the lanes are parallel across a speed-scaled lookahead window using configurable thresholds (`MaxParallelDistanceMeters`, `MaxParallelHeadingDegrees`, `ParallelLookaheadSeconds`, `ParallelLookaheadMinMeters`). If the window fails (diverging lanes or gaps), the merge is skipped.
- Safety checks gate merges: distance/width corridor clear, heading alignment, parallel distance/heading limits, and maximum merge speed. Cooldowns and random chance (`MergeChance`) prevent spam; post-merge cooldown and recheck timers apply.
- Pending lane changes signal first, then transition begins after lead time; completion swaps the car onto the destination lane and clears indicators once the minimum duration is met.

## Population Management
- Tracks human connections/PLIDs to compute available slots against `MaxPlayers` and `ReservedSlots`.
- Uses route metadata weights/targets to distribute AI across main/alternate/branch paths. Deterministic rounding controls spawn/remove counts per reconcile.
- Auto reconcile runs on a timer (`AdjustIntervalMs`) and on connection/AI join/leave events. Manual enable/disable is exposed in the debug UI; reconcile can be forced on demand.

## Debugging, UI, and Visualization
- DebugUI and MainUI expose buttons for recording, route selection/visualization, AI spawn/despawn, population toggles, speed overrides, and layout resets. Button handlers are wired via InSim BTC/BTT packets.
- LFSLayout visualizes coordinate axes and recorded routes/waypoints using AXM objects; detail steps can be changed to reduce object count. Active waypoint markers can be throttled or disabled.
- PerformanceTracker captures rolling metrics for AI input (AII) and telemetry (MCI) handlers when enabled.
- Logger writes to `log.txt` with warnings/errors and recovery/gearbox state traces; verbose InSim logging is optional.

## Operational Requirements
- Live for Speed server accessible on the configured host/port with InSim enabled and OutGauge available.
- Recorded routes present under `Routes/<track>/<layout>/` for main/spawn/alternate/branch paths; otherwise the system falls back to a circular path.
- Sufficient InSim admin rights to send chat commands, spawn/spectate AI, place layout objects, and render buttons.
- .NET Framework 4.8 environment; build output expects supporting assemblies in the `lib` subfolder as configured by the project.
