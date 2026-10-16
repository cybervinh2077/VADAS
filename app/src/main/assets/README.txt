# VADAS Large Assets Guide

Due to GitHub's 100MB file limit, the following large files are excluded from Git:

1. `ultra_fast_lane.tflite` (~178MB):
   - Place in `app/src/main/assets/ultra_fast_lane.tflite` for Ultra-Fast-Lane-Detection (UFLD).
   - If absent, LaneAnalyzer will automatically use fallback lane projection without crashing.

2. `dashcam_test.mp4`:
   - Place in `app/src/main/assets/dashcam_test.mp4` for offline dashcam video playback test.
   - You can also record directly using CameraX live camera feed.

Note: `detect.tflite` (~4MB) and `labelmap.txt` are tracked directly in Git repository.
