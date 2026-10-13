# Historical validation evidence

On 2026-10-13, the user requested removal of `/home/orin/ros2_ws/.maintenance`
and functional local Git commits. The old directory is not used by runtime
code, installed symlinks, systemd services or the inspected running processes.

This directory preserves small original JSON, JUnit XML and text/log results,
including failed and preliminary runs. `manifest.json` records original paths,
byte sizes and SHA-256 hashes. Contents of these records were copied unchanged.
Old absolute paths inside records are historical provenance, not current paths.
An old `.maintenance/<group>/<file>` report maps to this directory's
`<group>/<file>`. Not every old file was retained; consult the manifest.

Source/install backup archives, downloaded wheel caches, copied runtimes,
images and duplicate source trees are not retained. Git records current source;
it does not replace those deleted historical backups or make them recoverable.
These records describe their original code versions, including superseded
map/MPC following and detection-gate interfaces. They do not validate all current
code, do not mean tests were rerun during this cleanup, and are not robot motion
acceptance. Keep original records unchanged; append new evidence separately.
