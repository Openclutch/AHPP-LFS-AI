# Changelog

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
