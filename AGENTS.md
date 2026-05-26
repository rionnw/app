## Learned User Preferences
- Keep the `标注 ROI` control above the image area; the bottom action bar should only contain `打开/关闭相机`, `识别并解算`, and `直接运行`.
- ROI annotation should place a fixed 10x10 rectangle in original image coordinates at the click point, scaled to the displayed image, rather than using drag-to-draw variable rectangles.
- Treat `识别并解算` as recognize, solve, and convert without sending robot steps; treat `直接运行` as recognize, solve, convert, and send robot steps.
- Prefer device mocks to be enabled by development environment variables without adding a UI toggle.
- For image-display lag diagnostics, prefer lightweight throttled logs in the existing app log panel and terminal/stdout with stable prefixes rather than noisy per-frame console logging.

## Learned Workspace Facts
- The app is a Tauri project with a React frontend in `src/` and the main Rust backend in `robo-ui/src/lib.rs`.
- Camera streams are displayed in the frontend as one backend-composed grid image, not four separate images; ROI coordinates apply to that composed image.
- The backend supports both camera and file image sources: camera solving uses the latest frame path, while file solving uses a `solve_image_file` flow that decodes a data URL.
- Device mocks use `ROBO_UI_MOCK_CAMERA=1` and `ROBO_UI_MOCK_SERIAL=1`; default startup without those variables uses real hardware such as the Mac camera.
- Real camera diagnostics should not persist numeric camera indices blindly; AVFoundation/nokhwa can reorder Mac and USB camera indices between enumerations.
