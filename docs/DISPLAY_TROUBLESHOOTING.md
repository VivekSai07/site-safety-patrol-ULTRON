# Display and resource troubleshooting

From the team repository root, launch from the desktop terminal where
`xclock` works. It tests X11 access, not accelerated OpenGL. Keep its DISPLAY
and XAUTHORITY; do not change X server access controls.

```bash
bash scripts/launch-demo.sh
```

The launcher preserves display credentials, checks X access before loading MuJoCo, limits Mesa software rasterization to two threads, limits numerical-library threads and lowers process priority. The viewer is a fixed overview capped at 5 draws/second and 800 × 450 render pixels, including HiDPI displays. It has no passive background rendering loop, shadows or reflections. Motion and detection remain kinematic/oracle. Camera/depth evidence recording continues. Lower load does not guarantee desktop stability. Escape, window close or Ctrl+C stops the run, retaining partial evidence.

`ULTRON_VIEWER_FPS=3 bash scripts/launch-demo.sh` lowers the cap further. Allowed FPS: greater than zero through 30. The fixed overview replaces the MuJoCo inspector UI; no free-camera controls are provided.

Investigation on 27 November 2026: this execution session receives EPERM opening `/tmp/.X11-unix/X0` and `/mnt/wslg/.X11-unix/X0`, although DISPLAY is `:0`. This explains the agent's launch failure without disproving the user's working xclock. A small EGL probe reports Mesa `llvmpipe (LLVM 15.0.7, 256 bits)` here. WSLg logs also contain a software fallback. These establish rendering limitations here, not the exact cause of the user's near-crash or their desktop's OpenGL renderer.

The earlier investigation checked draw scheduling, HiDPI viewport cap, context restoration, shell syntax and refusal without display access. MuJoCo rendering was exercised under EGL with window transport stubbed. A visible GLFW window and desktop stability were not verified in that environment.
