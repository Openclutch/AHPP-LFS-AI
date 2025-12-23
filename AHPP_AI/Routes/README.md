Routes are stored per track/layout under `Routes/{track}/{layout}`. Recordings for the current
track are copied to the build output so they can be loaded by the AI at runtime.

- Default `main_loop` and `pit_entry` templates are created automatically for each track/layout.
- Legacy routes at the root of this folder are migrated into the active track/layout on load.
- Custom route names are supported; each one saves a standalone JSON inside the current track/layout folder.
