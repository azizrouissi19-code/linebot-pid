# linebot-pid

PID-controlled line-following robot on ESP32 with TCRT5000 reflectance sensors and a
live web dashboard for tuning. Consolidated from many development iterations.

## Main sketch
`linebot_pid_main.ino` — the complete build: PID line following, junction/circle handling,
white-line detection, and a Wi-Fi dashboard for live speed/PID monitoring.

## Variants (`variants/`)
Notable alternate builds kept for reference:
- `LineBot_Dashboard_WhiteLine_Integrated_Fixed` — integrated white-line + dashboard.
- `LineBot_PID_TCRT5000_StableDashboard` — stability-focused dashboard build.
- `LineBot_White_Line_Detection_Full_FIXED` — white-line detection emphasis.

## Hardware
ESP32 · TCRT5000 IR sensor array · motor driver · DC gear motors.

## Build
Arduino IDE / PlatformIO with the ESP32 board package. The robot creates its own Wi-Fi
AP; open its IP in a browser for the tuning dashboard. (AP password is a demo default.)
