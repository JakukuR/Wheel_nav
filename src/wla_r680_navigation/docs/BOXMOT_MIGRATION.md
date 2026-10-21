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
