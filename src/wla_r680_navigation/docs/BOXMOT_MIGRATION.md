# YOLOE-26s / native BoxMOT migration (2026-10-21)

This is the active replacement for GMC_RECOVERY.md and the separate CPU ReID sidecar. It is a local deployment candidate verified with real model inference and isolated ROS tests, not a moving-vehicle acceptance.

## Active pipeline

D455 RGB -> YOLOE-26s-seg (`predict`, never Ultralytics `track`) -> BoxMOT 25 BoT-SORT. All current person detections obtain OSNet x0.25 MSMT17 features through BoxMOT's own PyTorch backend, preprocessing and normalization. Native BoT-SORT uses these features, Kalman predictions, IoU and built-in SOF CMC. Other object classes retain a separate class-aware geometry tracker; the person ReID network is not applied to chairs or other objects.

The detector reuses exactly this frame's descriptors for `/r680/person_follow/reid_features`, limited to at most four requested identities. The follower retains its long-lived selected-person gallery and lost-time/predicted-position checks. The existing gallery-quality policy (confidence, size, clipping, overlap) remains separate from native tracking and is reported in feature diagnostics. No second encoder/process runs. Feature model id is `boxmot25-osnet-x025-msmt17:cf55163d78fc44c6`.

Native SOF is on by default (`use_cmc: true`, `cmc_method: sof`). Its own insufficient-keypoint/inlier behavior returns the identity transform. There is no custom GMC quality admission check and no public-ID change on compensation quality changes. Real-model tests force keypoint detection to return None for three frames; boxes and IDs continue. This is not a claim that blurred frames still contain a detectable person.

Public track integers are `2 * native_id` for people and `2 * native_id + 1` for other objects, inside the existing session/epoch namespace. These are transient track IDs, not permanent biometric identities. Only current, associated detections are published; no synthetic per-frame fallback IDs and no predicted-only boxes impersonate observations.

## Target-mask cloud exclusion

`/r680/perception/target_mask` is a bounded `wla-target-mask/v1` JSON message containing packed/compressed mask bits, RGB stamp, matched aligned-depth stamp, frame/size, session/epoch, task request and confirmed target ID. BoxMOT detection indices select the original YOLO instance mask; Kalman-smoothed boxes are not used for mask lookup.

The follower retains up to 16 aligned-depth acquisitions. During an active task it chooses the newest paired depth; after 120 ms it can process unpaired depth conservatively without removal. It never applies an old mask to a newer depth. RGB-depth matching remains bounded by the existing 35 ms policy; expiry remains 300 ms. Idle processing does not wait for masks.

Before native deprojection/ground/cluster filtering, the current confirmed target's segmentation pixels with valid depth within +/-0.35 m of the masked median surface become NaN. Points outside the silhouette and separated foreground/background depths remain. No whole-box deletion, target prediction mask, or clearing ray behind the target is generated. Missing/expired/changed-identity masks preserve all raw depth. Existing target-distance/collision constraints still apply.

The exact filtered cloud is passed to the local map and published on `/r680/person_follow/depth_obstacles`; the existing web display consumes that same output. Status includes `target_mask.applied`, `removed_pixels`, `reason`, and `compute_ms`. Existing historical map hits are not indiscriminately wiped: they clear by observed rays or the original 12-second TTL. Segmentation exclusion does not prove background free space or protect an entirely occluded object.

## Runtime and measurements

Pinned inference environment: `/home/orin/ros2_ws/.venv-boxmot`, Ultralytics 8.4.183, BoxMOT 25.0.0. It shares the existing GPU Torch 2.12.1+cu132/TorchVision stack rather than replacing it. The used runtime path was tested with vehicle NumPy 1.26.4, OpenCV 4.6.0 and SciPy 1.11.4; this is not a claim that every optional BoxMOT backend's dependency matrix is installed. Existing Torch emits its Orin CC8.7 support warning; the actual GPU tests below passed.

Models are official `yoloe-26s-seg.pt` and `mobileclip2_b.ts` from https://github.com/ultralytics/assets/releases/tag/v8.4.0 . Existing OSNet x0.25 MSMT17 weights are loaded by BoxMOT (565/565 required tensors); changing framework does not make these a newly trained stronger OSNet.

FP32 CUDA graphs cover YOLO inference and BoxMOT's ReID network forward. ReID uses fixed batches 1/4 with padding and bounded chunks. All capture occurs during discarded startup warmup, not on a new person entering the image. Descriptor replay matches the equivalent eager batch partition exactly in this sample (max absolute error 0). Different batch shapes can use different floating-point CUDA kernels; no bitwise cross-batch equivalence is claimed.

On the vehicle, 640x480 repeated bus image, 416 YOLO input, 27 configured classes, 4 person detections, 100 measured frames after warmup:

| Mode | Median | P95 |
| --- | ---: | ---: |
| YOLOE26 + native appearance tracking + SOF CMC | 63.87 ms | 64.58 ms |
| Same, CMC disabled | 58.14 ms | 58.77 ms |
| Native ReID only, 1 person | 5.22 ms | 5.33 ms |
| Native ReID only, 2 people | 11.10 ms | 11.32 ms |
| Native ReID only, 4 people | 13.50 ms | 13.78 ms |
| Native ReID only, 8 people | 25.90 ms | 26.09 ms |

GPU allocation after both graphs approximately 195.3 MiB (Torch allocated, not whole system memory). Warmup approximately 1.75 seconds and is discarded. Mask encoding/filtering is a separate CPU frontend stage, around 0.2/2 ms at 640x480 in the preliminary synthetic-depth benchmark. These are component timings; camera transport, ROS scheduling and local planning are not included. Full native tracking is around 15-17 compute FPS in this fixture, not stable 30 Hz or a measured live end-to-end rate. Repeated still images do not validate turning/occlusion/ReID accuracy.

## Installation and verification

Release overlay: `/home/orin/ros2_ws/releases/sim-follow-20261021-boxmot/install`. Both `r680_perception.sh` and `r680_person_follow.sh` source it last. The inference interpreter is `.venv-boxmot`; the follower remains system Python plus the existing C++ geometry backend. The follow launch no longer starts the old ReID sidecar.

Removed production source: custom `gmc_recovery.py`, unused Agent `yoloe_runtime.py`, old standalone `reid_node.py`/`reid_features.py` and vendored OSNet implementation. Replaced obsolete tests with native BoxMOT association, feature reuse, mask identity/time mismatch, pairer deadline and isolated ROS mask/depth tests. Historical release snapshots/measurement records remain rollback evidence, not active alternatives.

77 final targeted tests passed (68 component/ROS adapter checks plus 9 follower ROS tests, domain 198/localhost, no hardware command publishers). Real-model GPU test and native SOF weak-texture fallback passed separately. Source/installed parity and launcher dependency checks are required before switching the entrypoints. Raw evidence: `ros2_ws/.staging/boxmot-20261021/`.

Start preview: `cd ~/ros2_ws && ./r680_perception.sh debug-camera`; in a second terminal, `./r680_person_follow.sh run`. These commands keep the existing explicit-target/motion flow; this migration itself did not start camera/following or publish chassis speeds.

## TensorRT FP16 detector (2026-11-04)

`nav_yolo_detector` loads `~/ros2_ws/models/perception/yoloe/yoloe-26s-seg-fp16.engine` (not in git) when its sidecar matches the `--model` weights, text encoder and vocabulary. Otherwise it logs an error and runs the FP32 CUDA-graph path above; `/r680_nav/perception/timing` reports the backend in use. `WLA_YOLOE_ENGINE=` (empty) selects FP32 on purpose. The ReID network stays on its FP32 graph.

After changing the weights, the text encoder or `yoloe_classes.json`, rebuild (about 13 minutes):

```bash
~/ros2_ws/.venv-boxmot/bin/python ~/ros2_ws/install/wla_r680_navigation/lib/wla_r680_navigation/build_yoloe_engine
```

The engine has a fixed 320x416 input (what `imgsz=416` gives a 640x480 frame), so `--imgsz` does not apply to it. It needs `agnostic_nms=True` to match the `.pt` path, whose YOLOE class sets that by default; `TrackingRuntime` passes it.

Measured against FP32 on 113 held-out real frames with the live detector also running (one scene, FP32 as reference, no ground truth): detection p50 36.1 to 18.3 ms, forward 22.8 to 4.8 ms; person recall at conf >= .25 100% (30 boxes), at >= .10 98.4% (129 boxes). INT8 was rejected: 0.9 ms faster than FP16, person recall at >= .25 of 6.7% with 276 calibration frames from one scene.

## C++ BoT-SORT (2026-11-04)

`TrackingRuntime` runs BoxMOT's own native BoT-SORT for people and objects. OSNet still runs through `CachedBoxMOTReID`. `WLA_TRACKER_BACKEND=python` selects the Python tracker on purpose; if the library cannot load the runtime logs an error and uses Python. `/r680_nav/perception/timing` reports `tracker_backend`.

The stock library keeps every removed track until reset, so memory grew by about 2.7 KB a frame under track churn; live, the detector rose about 260 MB in 73 minutes. The runtime therefore loads `~/ros2_ws/models/perception/tracking/botsort_capi_bounded.so` (not in git), built from the installed BoxMOT source with removed tracks capped at 100, as in the Python tracker. If it is missing or was built for another BoxMOT version the runtime logs an error and uses the Python tracker. Build it after installing or upgrading BoxMOT (about 30 seconds; needs cmake, OpenCV and Eigen headers). The script stops if the source line it changes has moved:

```bash
~/ros2_ws/.venv-boxmot/bin/python ~/ros2_ws/install/wla_r680_navigation/lib/wla_r680_navigation/build_native_botsort
```

`NATIVE_ALIGNED` must stay: without it the adapter reads BoxMOT's tuned YAML defaults (`fuse_first_associate: true`, other thresholds), not the constructor defaults the Python tracker used. The C API has no `per_class`, so `NativeObjectTracks` keeps one tracker per class.

`track_buffer` counts frames in both backends, so it has to follow the detector rate.

Measured against Python on 8000 synthetic frames (16 seeds with 2-8 crossing people, missed and low-score boxes, distinct and similar appearance, panning camera on half; real pixels, no real people): person tracker p50 14.7 to 2.9 ms; identity switches 504 vs 504 and 514 vs 497. 99.9% of Python's tracked rows were also tracked by C++; 91-92% of frames were identical. The rest differ in association, with no more switches overall. Real multi-person footage has not been compared.

## OSNet crop upload (2026-11-04)

BoxMOT uploads each crop to the GPU on its own, which was most of the crop time. `batched_crops` uploads the batch once and produces identical values: for 8 boxes, crop time p50 8.05 to 3.37 ms. `WLA_REID_CROPS=boxmot` selects BoxMOT's path.

## TensorRT FP16 OSNet (2026-11-04)

`CachedBoxMOTReID` runs OSNet on two fixed-batch engines in `~/ros2_ws/models/perception/reid/` (not in git) when their sidecar matches the weights. Otherwise it logs an error and uses the FP32 graph; `WLA_REID_ENGINE=` (empty) selects that on purpose, and `/r680_nav/perception/timing` reports `reid_backend`. The model id stays the same because the network and weights do, so follow's `reid_model_id` does not change. Rebuild after changing the weights (about 6 minutes):

```bash
~/ros2_ws/.venv-boxmot/bin/python ~/ros2_ws/install/wla_r680_navigation/lib/wla_r680_navigation/build_osnet_engine
```

Measured against the FP32 graph on 418 real person boxes from two scenes (few identities): cosine at least 0.9998; among 40000 random pairs at most 0.27% changed side at 0.40, 0.45, 0.55 or 0.75. For 8 boxes with batched crops, get_features p50 25.4 to 11.7 ms and CPU per call 4.2 to 8.5 ms: FP16 costs some CPU and frees the GPU. A dedicated CUDA stream did not lower that CPU. Torch half precision on the graph gained only 1-2 ms.

## Detector rate (2026-11-04)

`nav_yolo_detector` runs at 20 Hz while follow's fresh request names the current epoch, and at 10 Hz otherwise. `WLA_DETECTOR_HZ` and `WLA_DETECTOR_IDLE_HZ` (or `--hz`, `--idle-hz`) change them; 30 and 30 is the old behaviour. Follow's control period is 50 ms. The pose producer takes the newest detection at 5 Hz, and consumers allow 0.3-0.5 s of age, from reading their code. Do not use 10 Hz during a task: in the synthetic test below, identity switches for fast walkers nearly doubled.

## Track buffer (2026-11-04)

17 frames keeps a lost track for 0.85 s at 20 Hz. Identity switches against ground truth, on 8 synthetic seeds of 40 s with people occluded for 0.2-1.2 s, slow / fast walkers: 30 Hz with 15 frames 219 / 254; 30 Hz with 26 frames 136 / 188; 20 Hz with 17 frames 122 / 222; 15 Hz with 15 frames 88 / 263; 10 Hz with 15 frames 63 / 450. Appearance is synthetic and distinct; real people have not been compared.
