// #include "config.h"

// #ifdef USE_XBOX_CONTROLLER
// // ─────────────────────────────────────────────────────────────────────────────
// //  Xbox (BLE) controller teleop — asukiaaa/XboxSeriesXControllerESP32
// //
// //  Why this library (not Bluepad32): Bluepad32 on ESP32 is an ESP-IDF project
// //  template, NOT a drop-in Arduino lib_deps library, so it won't compile inside
// //  this Arduino-framework project. This library is plain Arduino + NimBLE and
// //  installs straight from lib_deps. The ESP32-S3 (BLE 5.0) is the recommended
// //  chip for it.
// //
// //  Hardware note: ESP32-S3 is BLE-only. Use a Series S/X pad (or an Xbox One BLE
// //  model 1708) on controller firmware v5.15+; older v3/v4 firmware is Bluetooth
// //  Classic and will NOT connect. Update via the Xbox Accessories app.
// //
// //  platformio.ini:
// //      lib_deps =
// //          asukiaaa/XboxSeriesXControllerESP32_asukiaaa
// //          asukiaaa/XboxControllerNotificationParser
// //          h2zero/NimBLE-Arduino
// //  (If the controller won't connect, it's almost always a NimBLE-Arduino version
// //   mismatch — pin a version compatible with the asukiaaa release per its README.)
// //
// //  All gamepad→robot mapping lives in applyController(): one place to read/edit.
// // ─────────────────────────────────────────────────────────────────────────────
// #include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// // Default ctor scans for any Xbox controller. To bind to ONE pad in a noisy demo
// // room, pass its MAC, e.g.  ...Core("xx:xx:xx:xx:xx:xx");
// XboxSeriesXControllerESP32_asukiaaa::Core xbox;

// // ── teleop feel ──
// static const float STICK_DEADZONE = 0.12f;   // 0..1
// static const float MAX_SPEED      = 0.8f;    // m/s
// static const float MAX_YAW_RATE   = 2.0f;    // rad/s
// static const float CTRL_DT        = 0.02f;   // 50 Hz

// // Raw sticks are uint16_t, 0..65535, centred at 32768. Map to -1..+1 + deadzone.
// static inline float normAxis(uint16_t raw) {
//     float v = ((int)raw - 32768) / 32768.0f;
//     return (fabs(v) < STICK_DEADZONE) ? 0.0f : v;
// }

// static void applyController() {
//     auto &n = xbox.xboxNotif;

//     // joyLVert: stick up usually reads LOW, so negate to make "up = forward".
//     float fwd  = -normAxis(n.joyLVert);
//     float turn =  normAxis(n.joyRHori);

//     // Forward: advance the position target while held; release → coast to stop.
//     target.velocity  = MAX_SPEED;                 // traversal speed magnitude
//     target.position += fwd * MAX_SPEED * CTRL_DT;

//     // Yaw: integrate right stick into heading reference (rad).
//     yaw_ref += -turn * MAX_YAW_RATE * CTRL_DT;    // flip this sign if it turns wrong

//     // A → reset position + heading to "here"
//     if (n.btnA) {
//         target.position = currentState.position;
//         position_offset = 0.0f;
//         yaw_ref         = currentState.yaw_rad;
//     }
//     // Bumpers → prev / next height profile (edge-triggered)
//     static bool lbPrev = false, rbPrev = false;
//     if (n.btnLB && !lbPrev) applyProfile(active_profile - 1);
//     if (n.btnRB && !rbPrev) applyProfile(active_profile + 1);
//     lbPrev = n.btnLB;
//     rbPrev = n.btnRB;
// }

// void TaskControllerCode(void *pv) {
//     xbox.begin();
//     for (;;) {
//         xbox.onLoop();
//         if (xbox.isConnected() && !xbox.isWaitingForFirstNotification())
//             applyController();
//         vTaskDelay(pdMS_TO_TICKS((int)(CTRL_DT * 1000)));   // 50 Hz
//     }
// }
// #endif // USE_XBOX_CONTROLLER