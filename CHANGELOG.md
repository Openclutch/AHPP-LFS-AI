# Changelog

- Raised all recorded `AHPP_AI` route node heights in `AHPP_AI/bin/Debug/net6.0/Routes` by 2 meters on the Z axis to match the Live for Speed map height update.

- Fixed `AHPP_AI` initial route classification so fresh AI now use the nearest pit route again, then transition from that pit route to its matching loop route. On-track fallback no longer forces bots onto the global main route like `bmw_route_loop` when no plausible route is found; the AI is left unresolved in place so bad spawns can be debugged instead of being hidden by a fallback.

- Fixed `AHPP_AI` AI join/control state handling so per-AI runtime dictionaries used by spawn, initial-route selection, and the control loop are now synchronized. This prevents the collection-corruption errors that could start after a new AI joined and then cascade into repeated control update failures.

- Fixed `AHPP_AI` spawn-route classification so newly spawned AI once again prefer the configured pit route(s) for their assigned area tag instead of always taking the physically nearest `_pit` route. The nearest-route fallback still applies when the planned route is missing or implausibly far away.

- Fixed `AHPP_AI` route loading so the resolved alternate main route must share the same family tag as the resolved main route. Pairings like `bmw_route_loop` with `highway_route_alt` are now rejected, so loop and `_alt` routes only pair within the same tag such as `bmw_` or `highway_`.

- Fixed `AHPP_AI` initial on-track route classification so fresh AI no longer start directly on `_route_alt` lanes. Bots now begin on the primary loop route and only move to a parallel alternate lane through the existing lane-change safety checks.

- Fixed `AHPP_AI` alternate-lane switching so `_route_alt` merges now only run when an AI is actually following the paired main route path. This stops cars on unrelated loops like `island_route_loop` from trying to jump across the map into `highway_route_alt`.

- Added a loop-route traffic spacing equalizer in `AHPP_AI` cruise driving so cars now apply a small smoothed speed bias from whole-route occupancy, helping long-running traffic spread back out instead of permanently bunching behind the slowest lead car. The new behavior includes stable per-car spacing variation and new `config.ini` tuning options for enable/min-cars/gain/max-bias/variation/smoothing.

- Updated `AHPP_AI` logging so the app now writes into a `Logs` folder and appends each startup session to a dated log file instead of overwriting the previous `log.txt` on launch.

- Fixed `AHPP_AI` route-editor node selection so move/edit actions now resolve against the subset of markers actually rendered in LFS, and looped routes no longer place a duplicate closing marker on top of node `0`, which could stop the first node from being repositioned reliably.

- Simplified `AHPP_AI` spawn classification so new AI now always attach to the physically nearest `_pit` route regardless of heading, and removed the planned-vs-live spawn-route guessing/remap logic that could send cars onto the wrong route.

- Adjusted `AHPP_AI` initial track-route classification so if no heading-qualified route can be chosen, it now falls back to the physically closest on-track route instead of leaving the AI unresolved.

- Added a cumulative `AHPP_AI` spawn-route validation metric that logs the percentage of AI spawns which successfully resolve a valid initial route, and exposed the same percentage as a new `Spawn Route` status label in the main UI.

- Fixed `AHPP_AI` debug AI selection so spawning a new bot no longer steals the active debug target. The HUD now stays on the AI you picked from the list, and if that selected AI is spectated or otherwise removed its debug buttons are cleared until you manually choose another AI.

- Fixed the `AHPP_AI` control scheduler starving newer AI cars when the per-tick packet budget was lower than the active AI count. Control updates now rotate round-robin across the spawn-ordered pool instead of always servicing the first few PLIDs, and the periodic `CONTROL` log now reports how many AIs were deferred in each window so stuck cars like late spawns are easier to diagnose.

- Fixed `AHPP_AI` planned auto-population spawns dropping their target pit-area segment before `/ai` was issued, which meant fresh bots were always classified only from whatever live slot LFS happened to use and could all pile onto routes like `bmw_route_loop`. Planned segment tags are now queued through spawn and applied on the matching AI join before initial route selection runs.

- Refactored `AHPP_AI` driving so each active driver state now produces a complete control intent for the tick instead of stacking single-input overrides afterward. Normal route following, warmup hold, merge yield, reverse recovery, and engine restart now all flow through the same full-frame apply path, with ignition toggles included in the engine-restart state output.

- Overhauled `AHPP_AI` manual gearbox control so shifts are now driven by a smaller verified state machine that waits for `IS_AII` to confirm the actual LFS gear before releasing the clutch, preventing the driver from assuming first gear while the HUD still shows neutral.

- Removed the low-speed clutch hold and standstill full-throttle launch override that could leave AI revving in neutral or with slipping clutch, and updated launch diagnostics to show commanded gear, confirmed actual gear, target gear, and shift phase for faster drivetrain debugging.

- Initial spawn classification now rejects pit-entry routes more than 100m from the AI's actual spawn position, so distant routes like `tram_pit` are treated as unresolved instead of being accepted and sending the AI into pointless recovery loops.

- Added focused `LAUNCH DIAG` logging for near-stationary AI so spawn failures now record engine state, recovery state, warmup flags, target waypoint context, gear, clutch, throttle, brake, steering, and whether throttle is currently allowed by the gearbox logic.

- Extracted spawn and initial-route classification into dedicated route-planning policies with explicit decision/result models, so `AHPP_AI` now applies deterministic spawn-route fallback, on-track override, and spawn-to-driving-route handoff rules from a single testable layer instead of scattered controller-only heuristics.

- Added in-memory routing and path-projection tests covering planned-vs-live spawn selection, on-track viability thresholds, reversed candidate creation, spawn-route handoff decisions, and path distance/projection geometry so the core spawn reliability rules can be regression-tested without running Live for Speed.

- Tightened AI spawn classification so on-track routes are rejected when they are still implausibly far away or require a large heading flip from the fresh spawn position, preventing cars from locking onto distant highway/alternate lanes instead of their nearby pit route.

- Stopped the driver from applying unconditional launch throttle while stationary and badly misaligned to the target waypoint, so a wrong route choice no longer turns into sustained clutch-in/full-throttle behavior before reverse recovery kicks in.

- Neutralized the initial spawn throttle frame sent to new AI cars, reducing bad takeoff behavior during the first route-classification ticks and making spawn failures easier to diagnose from logs.

- Stopped importing legacy `Routes:Detour1-3` placeholders from config during startup, so `AHPP_AI` no longer tries to load missing branch names like `detour1`, `detour2`, or `detour3` when route discovery is supposed to come only from recorded files and metadata.

- Removed local AI spawn planning/ownership matching so this app no longer pre-assigns segment tags to future `/ai` requests or ignores AI that were spawned by another InSim controller. AI joins are now accepted from the live server state and classified only from their actual spawn position, while reset/spectate no longer queues a local replacement spawn.

- AI that still cannot resolve any valid initial route after the spawn warmup window is now spectated instead of sitting indefinitely with no usable path.

- Fixed AI spawn classification so remapped or on-road spawns no longer get forced onto pit-entry routes that are hundreds of metres away. The controller now compares pit-route distance against nearby drivable routes, promotes clearly on-track starts onto the nearest valid loop/branch even outside race mode, and logs the chosen spawn distance so bad slot mappings are obvious during debugging.

- Removed the remaining generic route-slot scaffolding from `AHPP_AI` so route bootstrap, recording selection, and UI route lists now rely on actual recorded route files instead of placeholder names like `main_alt`, `detour1-3`, or legacy `route1-3` mappings. Alternate and detour routes are now discovered from recorded metadata for the active layout.

- Cleaned up route branch discovery so resolved alternate-main files are no longer reprocessed as generic branches, and missing optional detour placeholders like `detour1-3` are ignored unless a real recorded detour exists for the active layout. This removes the false "route not found" and duplicate branch warnings that were appearing alongside valid `*_route_*` loads.

- Fixed AI spawn classification so bots now try their planned pit area's spawn routes first, but still fall back to the physically nearest pit route when LFS places them in a different bay, preventing fresh spawns from targeting another area's waypoints or getting stuck on a bad first route. Defensive PLID de-duplication was also added so repeated join tracking cannot leave duplicate AI rows in the UI list.

- Simplified AI spawn routing so live spawned position is now the sole source of truth for pit-route selection, and the AI's assigned population segment is remapped from that actual spawn route instead of trusting the pre-spawn segment guess.

- Spawn routing now cross-checks the planned pit-area route against the AI's live spawn position and falls back to the nearest actual pit route when LFS places the car in a different slot, preventing long-standing misroutes caused by planned area tags not matching the car's real spawn point.

- Fixed pit-area spawn classification so newly spawned AI first uses that area's configured pit route instead of picking the nearest pit route anywhere on the map, preventing bots from spawning in one area and immediately targeting a waypoint hundreds of metres away on another pit path.

- Stopped MCI-only engine-state fallback from treating stationary AI as stalled after the telemetry warmup window, which was falsely triggering the restart sequence and leaving bots holding clutch, throttle, and steering when more traffic was added.

- Stuck AI recovery no longer uses `/pitlane`, so failed recoveries now spectate the bot and queue a fresh spawn instead of forcing it back to pit slot 1 and away from its assigned route.

- AI spawns now wait for the actual nearest pit-route classification before assigning a driving route, and pit-route exit handoff now follows that pit family's recorded loop or alt route instead of snapping back to the default pit/main fallback.

- Route JSON loading in `AHPP_AI` now inspects each file's shape before deserializing, avoiding the repeated `System.Text.Json.JsonException` probes that were firing in the debugger for legacy route formats.

- AI spawn classification now always picks the physically closest recorded pit-entry route from the car's live position, so bad segment-to-pit mappings no longer send fresh spawns hundreds of metres toward the wrong route.

- Fixed the recent auto race-mode gating regression that left `trackCandidate` unassigned and broke `AHPP_AI` compilation.

- Restricted automatic AI race-mode selection to active race sessions without a pit-area spawn assignment, so normal pit traffic no longer flips into race/highway routing from the initial spawn heuristic.

- Grouped pit-route selection now locks the AI onto the same route family, so choosing a spawn like `da_pit` also keeps later routing on matching `da_*` loop/alt routes instead of crossing into another group.

- Restored segment-aware AI spawn routing so bots assigned to a pit area prefer that area's recorded pit route (for example `da_pit`) before falling back to nearest pit-route matching.

- Fixed the debug route label during AI spawn so it shows the per-AI pit route actually selected at spawn time instead of always displaying the configured default spawn route.

- Updated the debug `AI_ST` status label to show the AI's resolved active route name, so multi-route spawn/main/branch decisions are visible in the HUD.

- AI spawn classification now picks the physically nearest recorded pit-entry route when a bot appears, instead of preferring segment-tag pit routes or heading-weighted spawn scoring.

- Moved the in-game AI route list up by 40 UI units so more of the route selector sits clear of the lower controls.

- Fixed the AI list mode button so clicking `Race` again now switches the same AI back to `Cruise`, reapplying a spawn-route cruise path when current telemetry is available.

- Added support for segment-prefixed route names like `bmw_pit`, `bmw_route_loop`, and `bmw_route_alt`, and AI spawn selection now prefers the assigned segment tag's `<tag>_pit` file before falling back to generic pit-route matching.

- Removed legacy red fallback route entries from the AI route selector so detour/main placeholders are no longer shown when only custom recorded routes should be selectable.

- Added paging controls to the AI route selector so more than one screen of routes can be browsed in-game with `<` and `>` buttons instead of silently truncating the list.

- Reworked auto-population to target `PitSpawnArea` segment tags from `config.ini` instead of per-route JSON metadata, so AI counts are now distributed by segment `FillPercent` and spawned with that segment's configured mod/setup/colour presets.
- Added segment assignment tracking for spawned AI so auto-population removes surplus traffic from the correct segment when players join and keeps the configured player buffer intact.
- Updated the sample AI config to reserve an 8-player join buffer on a 48-slot server, cap auto traffic at 40 AI, and express the current highway segment as `FillPercent=100`.

- Added optional AI control diagnostics with per-state trace logging, suspicious throttle/clutch overlap warnings, and an `AI.ResetInputsEveryTick` toggle for debugging stale control inputs.
- Normal driving, warmup hold, merge hold, and recovery paths now all record and trace their control frames consistently so bad AI behavior can be tied back to the exact state/status that sent it.
- Added AI control unit tests that document reset-packet construction and the expected clutch/throttle overlap rules for launch versus abnormal driving.

- Changed AI spawn delay config and in-game input to use `SpawnDelaySeconds` instead of `SpawnDelayMs`, while still converting to milliseconds internally for the existing spawn timing logic.

- Route identity now follows the JSON filename instead of the embedded metadata name, so layout folders can use names like `pit_highway`, `route_highway_loop`, and `route_highway_alt` directly.
- Added route-name auto resolution per layout so missing legacy names like `pit_entry` and `main_loop` fall back to discovered `pit_*`, `route_*_loop`, and `route_*_alt` files in the active track/layout folder.

- Added a clickable layout picker under the AI track status so saved layout folders for the active track can be selected in-game.
- Persisted per-track layout defaults in `config.ini` and use them as the startup fallback before LFS reports the live layout name.

- Added per-area `ModPreset.<mod name>=setup,colour` support in the AI config and applied those presets during `/ai` spawning by sending `/mod`, `/setup`, and `/colour` before the spawn command.

- Kept AI spawning on the normal `/ai` flow and stopped using `/grid static` for AI pit-slot assignment; the pit spawn config remains available for future non-AI or higher-level area management.

- Treated `IS_AXI` receipt as successful layout discovery even when LFS does not report a layout filename, preventing repeated startup retries and warnings while keeping the current route context.

- Added configurable `[PitSpawn]` and `[PitSpawnArea.*]` sections to the AI app so pit slots 41-48 can be tagged as reserved AI spawn areas with per-area capacity, route lists, and allowed mod lists.

- Added a per-AI mode button in the right-hand AI list so each row shows `Cruise` or `Race`, and clicking the button promotes that AI into race mode.
- Added spawn classification so AI that appear on pit/spawn routes stay in cruise mode while on-track starts immediately switch to race mode and begin from the nearest main-track route.
- Added a configurable race driving profile with higher waypoint-speed targets, later upshifts, and optional LFS automatic gears/clutch for track-spawned AI.

- Guarded `Console.ReadKey()` usage in the AI app so headless or redirected launches no longer crash while starting or handling fatal errors.

- Restricted AI control to bots spawned by this app by gating AI admission behind tracked `/ai` spawn requests, preventing takeover attempts on other admins' AI and startup errors from non-owned bots.

- Reset AI pit recovery to reinitialize driver/warmup and respawn on the spawn route so bots don’t loop pitting and immediately re-enter recovery.
- Held MCI-based engine-running detection during recovery so stalled bots finish the validation window and pit after repeated failures instead of looping restarts.
- Expanded AI list button ID ranges so remove buttons no longer overflow the reserved range when more than 40 bots are online.
- Added a spawn delay input above AI Count so `SpawnDelayMs` can be applied/persisted from config at runtime.
- Added MCI-first telemetry with optional AII (`UseAiiTelemetry`) so control scheduling and spawn AII requests no longer assume AII traffic.
- Added telemetry warmup gating and optional brake hold to prevent recovery/stall transitions during the first spawn seconds in MCI-only mode.
- Added MCI-only engine-running detection (motion + spawn window) and surfaced telemetry staleness/recovery rates in control scheduler logs.
- Hardened gearbox low-RPM clutch logic to treat RPM=0 as unknown telemetry.

- Retried track/layout discovery after connecting so AI spawns pick up the active layout routes instead of falling back to the default context when AXI/STA responses arrive late.
- Added a delayed route reload after layout data arrives so recorded pit/main routes apply reliably on startup without needing a manual refresh.

- Prevented AI spawn from looping in recovery without IS_AII data by marking the engine running once the restart cooldown completes, so MCI-only setups no longer re-enter the stall routine.

- Clicking the `X` next to an AI name now spectates the bot via `/spec` before removing it so it is not sent to the pits.
- Added a main UI label beneath the performance indicator to show the active track and layout.
- Prevented false stall recovery by ignoring zero-RPM AII updates unless the car is also moving slowly; engine state now uses speed-aware detection in the control loop.
- Stopped the AI recovery loop by treating positive RPM as a running engine even when the ignition flag is missing, preventing constant stall restarts while driving.
- AI reset after recovery failure now sends `/pitlane` to keep the bot in its chosen car instead of spectating and respawning.
- Added a prioritized outbound send queue with token-bucket pacing and runtime limit updates so handlers no longer block; high-priority control packets bypass normal traffic while measured send rate and queue depths surface in periodic logs.
- Shifted control to a fixed scheduler driven by cached MCI telemetry (with RPM/flag caches from AII only), computing per-AI control Hz from packet budgets and round-robining updates to keep large fleets responsive without join/leave broadcast spikes.
- Added control cadence/AII target settings (`ControlTickHz`, `MinControlHzPerAi`, `MaxControlHzPerAi`, `AIITargetHz`) plus per-AI AII interval batching to avoid O(n²) bursts on spawn/leave while keeping overall send rate under the configured budget.
- Added a packet-budget-aware AI info refresh rate that stretches the AII/AIC interval as more bots join, keeping total packets under the configured per-second cap so larger fleets stay connected.
- Lowered default packet budgets and tied the InSim send limiter to the configured budget/reserve so outbound traffic stays below the server cap by default.
- Start All AI button now shows the current AI count so you can see how many bots are active at a glance.
- Spawn lane merges now pick a heading-aligned point on the base `pit_entry` route so cars join mid-route instead of pivoting back toward the start of the recording.
- Stopped spawn approach curves from reactivating every frame; once an approach is finished the AI hands off to normal waypoints instead of looping into recovery near pit_entry.
- Re-enabled approach curves for spawn paths so cars can arc from their spawn point onto the nearest pit-entry lane using local lookahead instead of driving back to the route start.
- Hardened spawn approach-curve setup to skip missing path/index data so AIs no longer throw key lookup errors when spawning.
- Spawn routing now hops from a lane-specific pit_entry_lane onto the base pit_entry route before switching to main, keeping the correct merge sequence.
- Added `InSim.AdminPassword` to `config.ini` so the AI connects with the multiplayer admin password when required.
- Added configurable AI brake smoothing thresholds and ramp steps in `config.ini`.
- Smoothed AI braking by coasting on minor slowdowns and ramping brake pressure instead of tapping.
- Added configurable logging levels for the AI logger to reduce production log noise or enable verbose testing output.
- Added per-pass horn/flash reactions with a 1–2 minute cooldown so AI only occasionally respond when fast humans overtake them.
- Throttled InSim send calls with a rolling per-second cap to prevent buffer-size disconnects when many AIs are active.
- Layout visualization now uses the recorded node heading on the last point of non-loop routes so end-of-path rotations stick.
- Added an AI performance status indicator under Refresh AXM with green/yellow/red health and windowed load percent.
- Saved layout editor heading adjustments back into the route JSON when a node marker is rotated or moved.
- Added `DebugAI.LaneChangeDetailedLogging` to toggle lane-change detail logs in `config.ini`.
- Fixed Spawn Here so the player reset faces the route direction instead of backwards.
- Added collision logging that records AI/human identities, control inputs, and path projection details (route, distance along, lateral offset) for post-incident review.
- Added path-aware traffic snapshots so AI maintain time-headway spacing, brake earlier, and yield more to humans; lane-change and pit merges now check rear TTC and configurable gap targets before merging.
- Added new traffic/merge tuning knobs in `config.ini` for spacing, lookahead, TTC thresholds, and spawn-merge hold distances.

- Added full AI driving tuning coverage in `config.ini` (steering range, throttle/brake, clutch/gearbox, waypoint/recovery thresholds, collision scan, lane-change lookahead).
- Main assignment no longer forces an immediate return from main_alt, so the alternate lane can cycle via cooldown/chance; added throttled assignment decision logs for visibility.

- Stopped false reverse recoveries after path changes by resetting progress baselines on new routes and only reversing when moving away at low speed or after repeated regressions, preventing neutral-plus-throttle coasting.

- Seeded waypoint progress tracking with the first real distance reading so initial checks no longer log overflow-sized numbers from uninitialized state.
- Moved the AI debug HUD into the left column and added a Show/Hide Debug HUD toggle that remembers its state via `DebugAI.ShowDebugButtons` in `config.ini`.
- Lane-change transitions now aim at a target lane waypoint several nodes ahead (tunable with `LaneChange.TargetAheadWaypoints` in `config.ini`) to keep merges on the alternate lane from snapping to a point behind the car.
- Centered the layout selection status label ("No node selected") in the AI debug UI so it sits in the middle of the screen.
- Lane-change targeting now penalizes behind-car waypoints and only uses lane heading alignment when the target is ahead, stopping highway merges from snapping to a point behind and bouncing back to main.
- Added configurable pass-by reactions so AI can flash headlights and/or honk for ~2s when fast human drivers zip past, with chance/speed/mode/distance tuning in `config.ini`.
- Kept AI on the alternate main lane after a merge by skipping branch-end rejoins for looped inner lanes, relying on explicit lane-change checks to swap back.
- Narrowed the AI list remove “X” button width to 5 so the label column has more room while keeping the delete control aligned.
- Moved the Route Name input above the Add Route control in the AI debug UI so you can name a route immediately before adding it.
- Clamped spawn coordinate conversion to short when resetting the player at a selected route node so the layout reset packet uses valid ObjectInfo types.
- Selected route brackets now stay white while only the route name shows red/green status in the selector.
- Added a “Spawn Here” layout editor button that resets the tracked player car to the selected node for quick testing.
- Route selector now shows configured main/spawn/alternate/detour routes, coloring buttons green when a recording exists for the current track/layout and red when it does not.
- Added layout editor buttons to delete a selected node or extend a recording from that point so routes can be tweaked without restarting from scratch.
- Removed the AI debug overlay control buttons (spawn/stop/pit/set speed) so spectating views stay uncluttered; equivalent controls remain on the main UI.
- Publish now drops managed DLLs into a `dll/` subfolder during `dotnet publish` to keep the release output tidy without changing the build command.
- Lane changes between main and main_alt now require a speed-scaled parallel window (configurable lookahead distance/angle) before merging; the AI system manual documents the new detection and tuning knobs.
- Added an AI system manual describing configuration, routing, driving behaviors, lane changes, population management, and operational requirements derived from the current code.
- Enforced main_alt rejoin safety by gating branch-end merges with distance/heading/speed checks and indicator lead time before switching back to the main lane.
- Cleared nullable warnings across layout visualization, route loading, path validation, and debug UI by marking optional inputs nullable and guarding missing AI/route data.
- Relaxed branch validation so looped/alternate lanes can use the same attach/rejoin index without spurious warnings when both lanes form closed loops.
- Active waypoint marker now re-places the cone when switching routes even at the same coordinates, so merge points between main and alternate lanes show a pylon instead of being skipped as duplicates.
- Cleared nullable warnings by marking optional AI dependencies/config/loggers as nullable, guarding missing OutGauge/MCI data, and allowing layout/debug events to handle null payloads safely.
- Defaulted route metadata/branch models to safe values and relaxed path/route helpers to accept nulls so route loading/validation no longer emits nullability warnings.
- Renamed the ButtonIds main helper to avoid false entry point detection during builds and aligned config parsing defaults to stop null literal warnings.
- Braking clutch helper now only releases after a braking-engaged press, so mid-shift clutch cycles aren't cancelled and logs no longer show an upshift request followed by a release back in the previous gear.
- Allowed highway lane-change checks even when AI are assigned to the main route so they will merge onto the alternate lane again and emit the expected lane-change logs.
# Changelog

- Fixed lane-change state to read the new transition path index before evaluating progress so merges don’t complete immediately and bounce back to the original lane.
- Added throttled lane-change skip diagnostics (cooldown/interval/random/route missing) so highway merge attempts and their reasons surface in the logs.
- Rebuild lane-change transitions at execution time using the AI’s current position/heading and a fresh entry point on the target lane so highway merges generate smooth pure-pursuit Bezier paths instead of jumping/overshooting.
- Reduced false heading rejections for lane changes by comparing car heading to the lane’s forward vector (not just the waypoint position vector), so near-parallel lanes aren’t blocked by small lateral offsets.
- During lane-change execution, we now pick the closest current-path index, bias the target a few nodes ahead, rebuild the merge path, and cap speed to the merge limit so the car follows a smoother arc instead of snapping and hitting walls.
- Engine restart recovery now hard-resets gearbox state to 1st gear (clutch held) before validation, preventing immediate upshifts to 4th and repeated stalls after a restart.
- Clutch release now waits for the requested gear change to be confirmed (while holding the pedal) before starting the release, matching the press-shift-detect-then-let-out flow.
- While clutch-held during a shift, we now force-latch the requested gear before releasing so takeoff no longer bounces back to neutral/1 without actually selecting 1st.
- Gearbox state now starts aligned to the spawn inputs (clutch in, 1st gear selected) so the initial launch no longer loops a 1→2 shift while still reporting gear 1.
- When a shift starts we now stick to the original target gear until the cycle finishes, preventing rapid clutch in/out flapping when speed hovers near a threshold (e.g., 4↔5 dithering).
- Added gear hysteresis (up/down buffers) to suppress gear hunting near speed thresholds and stop needless clutch presses while cruising.
- Added min time-in-gear plus RPM-based downshift preference so the clutch only comes in for real gear changes or stall prevention, not constant hunting.
- Added a config build tag (`AI.BuildVersion`) announced on startup so we can verify the running build without digging logs.
- Made the post-shift clutch hold configurable via `AI.ClutchHoldAfterShiftMs` in `config.ini` so the shift dwell can be lengthened without code changes.
- Added a short post-shift clutch hold so the pedal stays down briefly before releasing, reducing missed or harsh gear engagements for the AI.
- Added configurable alternate-lane merging: detect the MainAlt lane, schedule indicator-led transitions with safety checks, smooth Bezier handoffs, cooldowns, and post-cooldown random merge attempts tuned via new `[LaneChange]` config.
- Routed hard-braking clutch engagement through the clutch state machine so the pedal releases after braking instead of staying fully pressed while coasting.
- Added rolling performance stats to AII/MCI handlers (gated by `DebugAI.PerformanceLogging`) so logs show active AI count, handler rates, and processing times when fleets grow large.
- Merged MCI telemetry across packets and clear stale PLIDs so AI control keeps receiving car data when running large fleets (15+ bots).
- Added config-driven throttling for active waypoint markers (`DebugAI.ActiveWaypointMarkers`, `DebugAI.ActiveWaypointIntervalMs`) to cut AXM spam and reduce InSim buffer overflows when many AIs are running.
- Added an optional `DebugAI.VerboseInSimLogging` toggle plus extra spawn/connect diagnostics so InSim disconnects during large AI spawns can be traced.
- Batched layout object placement into capped AXM packets to reduce InSim disconnects when visualizing large routes and waypoint cones.
- Layout toggle now shows/hides both route and waypoint cone visuals and writes the preference back to `config.ini` so it sticks across restarts, with the UI button label reflecting current state.
- Spectate commands now target AI by name (no quotes) and the AI list remove button is tighter so only the X glyph shows.
- Hardened AI debug UI updates: validate tracked AI data before rendering, reset on stale info, and guard against null waypoint data to stop spammy null-ref logs when spectating or running many AI.
- Further guarded AI debug data validation, clearing stale tracking without logging so AI removals/spectates stop throwing transient errors.
- Private message helper now uses /echo via MST instead of IS_MTC so non-host clients stop hitting “only for multiplayer hosts” errors.
- Added `CollisionDetectionHalfWidthM` AI config to tune how wide the forward collision scan should be when detecting cars ahead.
- Waypoint visualization now uses recorded Z heights for cones/markers so pylons spawn on the road surface without AXM position errors.
- Switched AI tool status messages to private host-only sends (or direct button responder UCIDs) so chat updates no longer broadcast under the admin name.
- Added an `AutoManagePopulation` switch (defaulted off in `config.ini`) so AI population management stays paused on startup until manually enabled.
- Added a "Start Auto AI" button in the main debug UI to enable population management and trigger the first reconcile when you're ready.
- Widened layout visualization coordinate bounds so recorded routes render fully without clipping long paths.
- Added an AI population manager that tracks human joins/leaves, keeps reserved slots free, and balances AI spawns/removals across routes using new metadata targets/weights with deterministic rounding.
- Extended route metadata defaults and config (`MaxPlayers`, `ReservedSlots`, `AiFillRatio`, `MinAIs`, `MaxAIs`, `AdjustIntervalMs`, spawn/remove batch sizes) so per-route AI targets are loaded from JSON and populated automatically from `config.ini`.
- Unified AI recovery into a single driver-managed state machine (stall restart + short/long reverse with cooldown validation) with new tuning knobs in `config.ini` for timings, success thresholds, and escalation limits before forcing a pit/spectate reset.
- Fixed AI reset flow to avoid invalid `/pit` command; we now spectate the AI before respawning.
- Simplified AI debug buttons: `AI_CTL` now only shows control inputs, `AI_ST` shows state only, and the state button is larger in a third column for readability.
- AI resets now spectate by AI name (e.g., "AI 23") and issue a /join to restart instead of using PLID.
- Clamped waypoint/cone placement to safe layout bounds to reduce “invalid position” errors when visualizing routes.
- Added a hide/show toggle that clears the debug UI and leaves a top-left Show UI button to restore the controls.
- Combined recording/visualization route buttons into one list with an “Add Route” action so routes aren’t accidentally overwritten; record toggle stays centered.
- Shifted visualization detail controls left to avoid overlapping the unified route buttons on the right column.
- Replaced InSim online/local reconnect buttons with read-only connection and host status labels in the AI debug UI.
- Recorded route visualization now places chalk markers at the recorded Z height with rounded coordinates so 1x detail renders stop failing with “invalid position” errors.
- Guard AI debug updates when car telemetry is missing so the debug UI stops throwing null reference errors.
- Offloaded AI spawning and recorded-route visualization to background tasks so large batches or heavy layouts no longer block InSim commands.
- Added low-RPM clutch protection so AIs press the clutch when RPM dips under ~500, let the engine recover, and then resume their previous action without stalling.
- While on the inner highway lane, AI now occasionally merges to the main (left) lane with heading/speed limits, distance checks, and cooldowns for a smooth random lane change.
- Fixed alternate-lane rejoin logic variable clash so the build succeeds while keeping smooth lane swaps.
- Added an official alternate main lane: configure `Routes.MainAlt` (e.g., the inner highway lane) to load alongside the main loop for smoother lane weaving.
- Inner highway branch changes are now opportunistic lane changes with a cooldown and random chance instead of firing immediately when the branch starts.
- Inner lane changes are blocked at higher speeds, use a tighter heading filter, and temporarily cap target speed on entry to stop the hard snap that caused stalls.
- Branch handoffs pick a heading-aligned entry waypoint so steering corrections stay smooth when swapping onto alternate routes.
- Introduced pure pursuit steering for AI with configurable lookahead, wheelbase, gain, and steering limits in `config.ini` to smooth waypoint following.
- Centralized InSim button ID ranges for the AI tools (using a shared ButtonIds registry) to mirror the Touge layout and avoid clashes with LFS/native buttons or debug overlays.
- Added configurable steering deadzone (`AI.SteeringDeadzoneDegrees`) alongside damping so small heading errors can be ignored to calm straight-line oscillations.
- Added configurable steering damping (`AI.SteeringDamping`) so heading corrections can be softened to reduce fishtailing on straights.
- AI now sends unrecoverable cars to the pits/spectate after failed recovery cycles so they don't block the track.
- Enabled wall recovery by default and removed dead AIDriver helpers (unused ignition command, stored headings) so recovery routines stay active and simpler.
- Improved AI stall recovery by turning toward waypoints, boosting throttle when steering, and backing up after repeated stalls.
- Moving a selected layout marker now writes the new position back into the corresponding route JSON so edits persist.
- Set a non-zero ReqI on TTC selection requests to stop LFS warning about IS_TTC packets with no request id.
- AI debug state button now surfaces real driving conditions (slow progress, moving away, stuck/low speed, reverse recovery) instead of a generic driving label.
- Blend steering toward multiple upcoming waypoints (weighted by proximity) so highway following uses a smoother lookahead path instead of chasing a single far point.
- Added a debug button to re-request AXM selection feed (TTC_SEL_START) plus online/local reconnect buttons to help single-player layouts stream Shift+U selections reliably.
- Added UI buttons to reconnect InSim to either the online host or local host so layout selection tracking can be pointed at the right server.
- Centered the layout node editing controls (attach, node speed, rejoin) along the bottom so the Shift+U editor buttons sit in the middle of the screen.
- Route visualization now falls back to the default player/AI when no camera view is selected, instead of requiring manual driver selection.
- Added a "Recording" label above the bottom selector row and moved visualization routes into a single right-hand column flush with the 200px edge topped with a "Visualizer" label so those controls are easier to understand at a glance.
- Added layout editor selection handling with UI controls to tag route nodes with speed limits and attach/rejoin indices while editing in Shift+U.
- Reset Layout now only clears layout visuals without removing AI.
- Added visualization route selection buttons with detail controls so recorded routes render as directional chalk arrows (main white, pit red via LYT flag) while staying under the layout object cap.
- Moved .NET Framework executable dependencies into a `lib` subfolder (with probing enabled) so the output folder highlights the runnable .exe.
- Added route validation that reports missing data, duplicate names, invalid branch indices, and non-loop main routes directly to the InSim user.
- Routes are now scoped per track/layout (with automatic migration of legacy files) so main/pit and detour recordings stay organized across circuits.
- Added an InSim route-name input and dynamic selector buttons that surface main/pit plus any custom recordings for the active track/layout.
- Discovered detour routes are auto-loaded as branches and connected to the main loop using recorded attach/rejoin waypoint metadata or nearest-point matching.
- Removed orphaned TougeBattle projects from the solution and added smoke tests so the test harness runs and validates core utilities.
- Tightened AI recovery logic: fixed reverse gear mapping, added a real ignition pause, removed redundant path init, and detect waypoint progress stalls so wall recovery kicks in instead of staying stuck.
- Added a debug button that tracks the viewed AI and shows whether it is driving normally, recovering, or restarting.
- Select the nearest waypoint that matches the car heading (reversing the path if needed) so AI handoffs onto recorded routes follow the loop instead of steering 180° or falling back to circles.
- Prevent AI cars from orbiting tight waypoints by dropping steering focus to the current node when close or facing extreme turns, even with a longer lookahead.
- Rebuild AI waypoint paths when an empty route is set so driving continues on a fallback path instead of spamming missing-path errors.
- Added an AI light controller so headlights, high beams, hazards, indicators, and horn can be driven per-car with helper methods.
- Lane/branch changes now trigger timed indicators (including manual branch switching support) so AIs signal before moving to alternate routes.
- Guard waypoint targeting when no valid path is available so AI control no longer crashes on divide-by-zero errors.
- Added a dedicated `Routes` folder (copied to build output) so recorded waypoint JSONs can be versioned and reviewed.
- Cleaned the AI debug panel by removing unused buttons, clarifying labels, and adding inline inputs for AI count and speed.
- Fixed type-in buttons so their labels are visible on-screen while also titling the dialog.
- Validated AI count/speed entries and send chat confirmations when spawning AIs or updating their target speed from the UI.
- Keep pit and main routes visible together when toggling layout visualization and render the pit route in red for clarity.
- Removed player-facing debug buttons from the in-game debug UI to focus on AI testing controls.
- Added a bottom-center Record toggle with route selector buttons so you can pick main, pit, or detour recordings before starting.
- Fixed record route selection wiring by routing it through the AI controller instead of the missing Program-level UI reference.
- Record button now shows live point count while recording to confirm data is being captured.
- Recording button turns green while active so it's obvious when capturing data.
- Synced `config.ini` with the AHPP Touge Racing template, including upgrade, economy, monitoring, and cop mission settings.
- Added config-driven host/port, debug flags, route names, AI spawn counts/delays, and OutGauge port to remove hardcoded values.
- Config file now marked as content and copied to the output so builds include the current `config.ini`.
- Prevent UI rendering before InSim connects by deferring record button drawing until the panel is shown.
- Reduced main UI buttons (including the Record control) to width 20 to match the compact debug buttons.
- Added configurable waypoint lookahead so AI target selection can be tuned via `LookaheadWaypoints` in config.
- Added configurable recording interval (meters between points) with both config support and an in-game type-in control.
- Skip sending /spec to invalid PLIDs when resetting or stopping AI fleets to avoid “parameter invalid” errors.
- Replaced “Spec All AI” with “Pit All AI” to send the `/pit_all` command from both UI and controller.
- Added per-AI “X” buttons to despawn individual bots directly from the list.
- Fixed AI removal buttons by routing click handling through the controller instead of a missing Program-level UI reference.
- Guard per-AI removal so /spec is only sent for valid PLIDs to avoid errors when clicking the X control.
- Steering now aims at a configurable lookahead waypoint while progress uses the current target, giving the car more time to react.
- Added reverse recovery: after repeated stalled progress, the AI backs up with a turn before retrying.
- Made waypoint proximity size configurable (`WaypointProximityMultiplier`) and set recording spacing to 5m by default.
- Record button now receives live updates (including green state and waypoint count) by constructing UI before the recorder.
- Add a route library and JSON templates for main loop, pit entry, and detour recordings with metadata.
- Record routes with typed presets, per-node speed limits, and saved metadata for editing later.
- Visualize recorded routes in LFS with color-coded cones so waypoints can be selected and tweaked in the layout editor.
- Make debug recording buttons clickable again so route recording can be started and stopped from the UI.
- Move debug UI button IDs up to the 240 range (descending) to avoid clashes with pit/menu UI elements.
- Add a Layout toggle button to load recorded routes and visualize their layout objects on demand.
- Switch recorded route visualization to chalk arrows with forward headings instead of cones for clearer flow.
- Avoid re-placing the active waypoint cone when the target hasn't changed to reduce layout spam.
- Honor recorded speeds from JSON when no explicit speed limit is set, and record live speeds into speed limits instead of forcing a default.
- Add a Reset button to clear layout visualizations and remove all AIs in one action.
- Replace Add AI with an inline type-in so clicking it immediately prompts for the number of AIs to spawn.
- When recovery attempts are exhausted, spectate the stuck AI and auto-spawn a replacement to keep the fleet moving.
- Swap route recording visualizer to Chalk Ahead arrows and align their heading with the driver's car so they follow movement direction.
- Raise Chalk Ahead markers to the car's Z position when recording so arrows no longer stick at a fixed 0.5m height.
- Flip Chalk Ahead marker heading 180° so route recording arrows point forward along the car's travel direction.
- Add recording button color cues so active recordings are obvious in the debug UI.
- Move the Add AI input box below the AI controls to stop it overlapping existing AI buttons.
- Wire the Load button to reload route JSON files and refresh active AI paths.
- Shift spawned AI list buttons further right to avoid overlapping other controls.
- Upgrade AHPP_AI and AHPP_AI_Tests to SDK-style .NET 6 projects with cross-platform dependencies (ImageSharp replacing System.Drawing).
- Configure AHPP_AI to produce a Windows x86 RID for publishing an .exe alongside cross-platform builds.
- Restore pits upgrade buttons to hide when leaving car selection and reappear beside the buy control once a car is chosen.
- Fix connection HUD substate display when IS_CIM reports byte-based submodes.
- Add debug HUD button to display the current UI substate beneath the connection state indicator.
- Remove the always-on-top configuration flag from GUI buttons so they behave as they did prior to the recent regression.
- Track LFS interface submodes so pit controls only show while the garage or pit setup screens display the selected car.
- Hide pit upgrade buttons while browsing the mod selection list so they only appear with the current car on screen.
- Clear the selected car when reopening the mod browser so pits buy and upgrade buttons immediately hide.
- Refresh pits buy/sell controls as soon as a car is selected so garage prices appear without reopening the menu.
- Keep buy and price buttons visible after selecting a car in the pits so drivers always see purchase options.
- Show drag races with a single opponent using both driver names and reserve "vs others" for larger lobbies in pending and active lists.
- Show pits buy/sell buttons when browsing the garage so new players can purchase cars without pitting first.
- Restore pit buttons when entering the garage immediately after joining so new players can manage cars.
- Keep upgrade controls visible in the garage even when arriving directly from the race screen.
- Show Tycoon missions as Undercover when players start them in cop cars.
- Add beta tester flag with red prestige badge support and a database column for tracking testers.
- Add developer badge support, including database storage and nameplate rendering alongside beta tester tags.
- Display GET VIDEO button for every campaign stage that has a configured YouTube link using the standard youtube{Stage} keys (R1, D1, etc.).
- Load campaign YouTube links from configuration instead of hardcoding them.
- Prevent completed battles from being aborted again so boss victories don't show failure dialogue.
- Spawn pit upgrade buttons only when entering pits to avoid flashing during other UI interactions.
- Hide mission leaderboard for Tycoon missions.
- Only display pit upgrade buttons when a car is selected and the driver is in the pits.
- Add button on How To menu to send configurable music stream link.
- Move accept battle button to the left side of the screen so challenges appear at x=0.
- Display remaining cop chase cooldown time when attempting a chase too soon.
- Clarify delay drag battle announcements and join prompts so players know all drivers are welcome.
- Fix compile error in pits dialog by removing stray closing brace.
- Hide upgrade buttons when no car is selected to avoid showing them before choosing a vehicle.
- Align How To and campaign dialogue text to the left to prevent random shifts.

## [0.8.5.4] - 2025-08-20
### Gameplay
- Hide damage button on connection HUD unless in Touge, Practice, Legend, or Attack battles.
- Hide HUD and pits buttons while the menu is open so menu clicks don't trigger underlying controls.
- Reset sell-car warning when switching vehicles so each selection prompts again.
- Log driver and nearby player positions during teleport safety checks to diagnose occupied warnings.
- Default buttons to use LFS always-on-top mode so closing the menu restores all UI elements.
- Link Prologue campaign GET VIDEO button to introductory YouTube video.
- Ensure pit upgrade and HUD buttons use the always-on flag so they remain visible over the LFS interface.

### Tech stuff
- Ensure logs are written to the application directory so Windows services produce log files as expected.
- Automatically create the `AHPP_TougeBattle` service during deployment if it is missing.
- Update deployment workflow to manage `AHPP_TougeBattle` service directly and remove watchdog script.
- Convert Touge racing program to run as a Windows service.
- Fix watchdog service start failure by locating executable relative to script
  when run from the service root.
- Move service binaries to `C:\AHPP_INSIM\bin\Release` and load configuration,
  database, and dialogue files from the `AHPP_INSIM` base directory.
- Install NuGet CLI in CI workflow to ensure `nuget` command is available.
- Restore NuGet packages in CI before building to ensure all dependencies are available.
- Updated GitHub Actions to build the AHPP_TougeBattle project instead of AHPP_Server.
- Use parent directory above executable for configuration, database, and dialogue files.
- Corrected GitHub Actions restart step to reference InSim DLL in Release folder.
- Load configuration, database and dialogue files from directory above Release.
- Updated restart script default config path.
- Ignore build output directories and runtime data.
- Ensure dialogue CSV is loaded from AHPP_INSIM base directory rather than Release.
- Resolve log directory from configuration and convert it to an absolute path so services write logs to the expected folder.

## [0.8.5.3] - 2025-08-18
### Added
- Battle payouts for drag, touge, and attack races are now configurable via `config.ini`.
### Changed
- Battle classes now reference program-level payout values loaded from the config file, standardizing `Attack` payout key.
- Reduced payback amount for Ange car to 250k credits.
### Fixed
- MissionInsertResult now logs SQL with parameter values, aiding debugging of mission activity inserts.
- Reuse existing value formatter in SQL logging helper to avoid duplicate conversion logic.

## [0.8.5.2] - 2025-08-17
### Fixed
- Ensures the main menu and menu button use the always-on flag so they display above LFS interface.

## [0.8.5.1] - 2025-08-16
### Fixed
- Mission payout decay now only counts successful missions so failed attempts no longer reduce rewards.
- Avoided running track payout scaling before the server reports its track, preventing startup warnings.

## [0.8.5] - 2025-08-14
- Hides battle accept button after acceptance and resolves dialog persistence issues.
- Clears drift and speed scores and health bars on race exit, CatMouse abort, or battle entry.
- Adds drag race delay configuration for faster cars, increases handicap scale, and improves false start handling and countdown behavior.
- Introduces mission payout decay configuration with updated scaling.
- Adds restart notification script with INI configuration, disconnect support, and reduced chat spam.
- Adds menu button above system buttons and logs SQL queries with parameter values.
- Adds unit test for CarManager progress and cleans up tracked artifacts.
- Refines CI workflows and deployment scripts for reliable service management.


## [0.8.1] - 2025-06-18
### Fixed
- When doing a campaign action like blackout and teleport with story, it should cancel current pending and active battles, or active mission, this avoids the menu from disappearing when doing a campaign sequence
- Fixed boss battle references to legend battles
- Fixed pending legend battles showing as practice with mission text
- Selling car now should sell the upgrades that you owned as well
- Did a few fixes for false starts to wait a second before tracking that. Also added logic to avoid doing the same state multiple times (potentially causing multiple actions like multiple pits when moving, etc, we should see no more duplicate messages now)
- Moved the legend text if you win, lose, or leave to the mission code to try and more reliability tell the player the message. Before it would only work sometimes since the mission is cleaned up before it can send the message (race condition)
- Ignore battle insim circles while not in battle (circles 0-10) This was causing the “unknown event”
- Close mission dialog or any menu actually when the player spectates or pits
- Fixed text formatting so we can import descriptions for mods into the database, was previously skipped but is now fixed. Needed this for reviewing not the game lol
- Adjusted the drifting speed to 30 from 20 kmh, making the missions a bit harder to just do donuts and get the mission done
- Fixed level progress when using !rm, it used to a bad ref to connection
- Added fix for cleaning up drivers when the player leaves the game, causing the lvl 1 glitch for missions
- Adjusted payback text for ange car
- Fixed bug to allow selling cars, but not the car if its anges and you didn't pay her back yet
- Fixed bug in !rm where it didn't set payback car to 0, and now removes dragbattles
- Updated text for selling anges car mods
- Added logic to prevent players from purchasing mods for ange car, that could potentially soft lock if they spent their money before removing the car, being stuck with the kei van

## [0.8.3] - 2025-07-15
- change seconds to time stamp on missions
- added freeroam drift points 2k culm to go towards drift missions (going towards that path)
- fixes for teleporting
- fixed crash when pitting while battling, and during countdown 
- added threshold to only track payouts for freeroam speeding or drifting if you get over 500 points
- added logging for when server is taking too long and slowing down
- fixed cop pull over cooldown bug
- added health system for touge
- added legend stars at the top
- added attack battle health system
- added brand deal bonus feature
- fixes for drag races
- added order to accept things, if you have a mission up, !a will accept that, otherwise touge challenge, otherwise, attack, last is drag race
- reduces attack battle damage to only 25% of touge values
- added drag stats in the menu, if you aren't in a car, it will show all times, if you are in a car, it will only show you 20% up or down od bhp/ton of the car you are in

## [0.8.4] - 2025-07-23
- Drift missions now accumulate drift score instead of one shot chain
- Added new command !dragdelay (temp name) so it will launch a drag race, but delay the start for the faster cars by tier. the stats should still individually track, but makes for hopefully a fun close finish
- fixed bug when damage happens in freeroam
- fix for drag to teleport all at once, added more time to the tree countdown
- fix for paying back ange car so you can upgrade it after
- only let the player to do the architect if they finished the previous legends first
- fixed button id issue

## [0.8.5] - 2025-07-25
- fixed bug in drag where if you left, the lights would stay up
- fixed more bugs with drag battles crashing when pitting at times before the battle starts
- now clear drift and speed score when starting any battle to prevent stale points giving lots of money
- Missions now will be worth less if you continue grinding the same one within an hour. Encouraging a variety of activities.
- Added real brand names for the brand deals!
- Routes: Branch/detour names are now discovered from recorded route files only; detour entries no longer come from config defaults to avoid duplicate-name warnings.
- Routes: Configured branch route names are ignored so only JSON files drive branch detection during route reloads.
- Stopped auto-creating placeholder route files (UnknownTrack/DefaultLayout templates) when routes are missing; missing routes now log a warning without writing default layouts.
