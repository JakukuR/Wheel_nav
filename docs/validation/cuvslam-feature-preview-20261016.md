# cuVSLAM feature observation preview — 2026-10-16

## Behavior

The navigation web page has a **VIO 特征匹配** entry at `/vio.html`.
Run the existing cuVSLAM navigation command, then open
`http://172.31.18.156:8080/vio.html`.

- Green marks: cuVSLAM's exported 2D observations.
- Yellow lines: the same SDK track ID in the two cameras; at most 100 stereo
  lines are drawn for readability. All valid observations are marked.
- Cyan lines: the same camera and track ID in successive displayed samples.
  Sampling is at most 5 Hz. History is cleared after a >1 s gap, timestamp
  reversal, image-size change, or process restart.
- These observations/IDs are not advertised as PnP inliers or accuracy scores.
- Empty, stale and disconnected states are visible; frozen images are dimmed.

## Source and integration

The existing cuVSLAM `GetState()` call already exports observations. This change
reads `State::observations` (`id`, `u`, `v`, `camera_index`) without adding a
feature detector, changing the SDK, or changing estimator health/motion policy.
Official API reference:
https://github.com/nvidia-isaac/cuVSLAM/blob/2a0882cfada6448632f37182a4b8a270adb2ec3d/libs/cuvslam/cuvslam2.h

The SDK implementation maps these IDs from `track_id` in its `tracks2d` output.
The exact two shared image messages supplied to `Track()` are retained with a
copy of that call's observations; a mismatched SDK timestamp gives an unannotated
image instead of incorrectly overlaying another frame's observations.

`FeatureVisualizer` has a separate rendering thread and one pending snapshot.
It publishes `sensor_msgs/Image` (`rgb8`, acquisition stamp from the left image)
on `/r680_nav/vio/features/image`, best effort, depth 1, at most 5 Hz. Rendering
exceptions do not change tracking health. `publish_feature_visualization=false`
disables this publisher/thread.

The web gateway subscribes while `/api/vio/status` is polled and releases its
subscription after 3 s without requests. With no subscribers, the frontend does
not copy or render previews. The page and JPEG encoding use the existing camera
owner and the existing web gateway. There is no second camera pipeline.

## Validation and deployment

- Built the actual Orin cuVSLAM executable successfully, no compiler warnings.
- All 7 CTests passed. The new renderer test checks exact track-ID association
  (including IDs above JavaScript's integer precision), invalid coordinates,
  padded images, stale history, and missing observations. The new ROS test checks
  asynchronous image delivery, acquisition headers and rejection of mismatched
  observation/image timestamps.
- `test_cuvslam_feature_gateway.py` passed in ROS domain 198 with synthetic images,
  no camera, VIO, Nav2 or chassis nodes. It checks the navigation entry/static
  assets, demand-based subscription, actual ROS image to JPEG HTTP delivery,
  source timestamp, frozen-frame status and subscription cleanup.
- Browser inspection of the synthetic fixture verified stereo image layout,
  colored points/lines, live age, waiting and stale presentation. The fixture is
  explicitly marked `SYNTHETIC SOFTWARE TEST - NOT CAMERA DATA`.
- Python syntax, JavaScript syntax and `git diff --check` passed.
- Installed the rebuilt executable into `.runtime/cuvslam_navigation/install`;
  updated `index.html`, `vio.html`, `vio.js` in the installed navigation web
  directory. The installed gateway script is a symlink to its source.
- Unrelated dirty launch/gateway/OpenVINS files were preserved.

Logs: `.runtime/cuvslam_navigation/feature-preview-{build,test,install,ui}.log`.
Integration artifacts: `.runtime/cuvslam_navigation/feature-preview-test/`.

No robot motion or live camera trial was performed. The tests establish the
rendering/transport/UI path with software fixtures, not the number or quality of
real SDK matches or the live performance overhead. Existing RTAB-Map covariance
crashes are outside this visualization change.
