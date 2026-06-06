#include "config.h"

#ifdef USE_XBOX_CONTROLLER
// ─────────────────────────────────────────────────────────────────────────────
//  Xbox (BLE) controller teleop — Bluepad32
//
//  Hardware note: ESP32-S3 is BLE-only (no Bluetooth Classic). That is fine for
//  Xbox Series S/X pads and Xbox One BLE models (1708) on controller firmware
//  v5.15+, which speak BLE. Older controllers on v3/v4 firmware use Bluetooth
//  Classic and will NOT connect to an S3 — update the firmware via the Xbox
//  Accessories app first.
//
//  PlatformIO: requires arduino-esp32 core 3.x. Add to platformio.ini:
//      lib_deps = https://github.com/ricardoquesada/bluepad32
//  (Alternative, simpler Xbox-only lib: tbekas/BLE-Gamepad-Client.)
//
//  All gamepad→robot mapping lives in applyController() below — one place to
//  read or change for the demo.
// ─────────────────────────────────────────────────────────────────────────────
#include <Bluepad32.h>

static ControllerPtr gamepad = nullptr;

static void onConnect(ControllerPtr ctl)    { if (!gamepad) gamepad = ctl; }
static void onDisconnect(ControllerPtr ctl) { if (gamepad == ctl) gamepad = nullptr; }

// ── tuning constants for teleop feel ──
static const float STICK_DEADZONE = 0.12f;  // ignore small stick noise (0..1)
static const float MAX_SPEED      = 0.8f;   // m/s, forward/back top speed
static const float MAX_YAW_RATE   = 2.0f;   // rad/s, turn rate at full stick
static const float CTRL_DT        = 0.02f;  // 50 Hz task period (s)

static inline float deadzone(float v) {
    return (fabs(v) < STICK_DEADZONE) ? 0.0f : v;
}

// Map one frame of gamepad input onto the existing setpoints. The balance/yaw
// controllers do the rest; we only move target.position and yaw_ref.
static void applyController(ControllerPtr ctl) {
    // Bluepad32 stick range is roughly -512..511.
    float fwd  = deadzone(-ctl->axisY()  / 512.0f);   // left stick Y, up = forward
    float turn = deadzone( ctl->axisRX() / 512.0f);   // right stick X, right = CW

    // Forward: push the position target ahead while the stick is held; release
    // and the robot coasts to the last target (trapezoidal profile handles stop).
    target.velocity  = MAX_SPEED;                      // sets traversal speed magnitude
    target.position += fwd * MAX_SPEED * CTRL_DT;

    // Yaw: integrate the stick into the heading reference (radians).
    yaw_ref += -turn * MAX_YAW_RATE * CTRL_DT;         // flip sign here if it turns the wrong way

    // A  → reset position + heading to "here"
    if (ctl->a()) {
        target.position = currentState.position;
        position_offset = 0.0f;
        yaw_ref         = currentState.yaw_rad;
    }
    // Bumpers → step through height profiles (edge-triggered)
    static bool lbPrev = false, rbPrev = false;
    if (ctl->l1() && !lbPrev) applyProfile(active_profile - 1);
    if (ctl->r1() && !rbPrev) applyProfile(active_profile + 1);
    lbPrev = ctl->l1();
    rbPrev = ctl->r1();
}

void TaskControllerCode(void *pv) {
    BP32.setup(&onConnect, &onDisconnect);
    BP32.enableVirtualDevice(false);

    for (;;) {
        bool dataUpdated = BP32.update();
        if (dataUpdated && gamepad && gamepad->isConnected() && gamepad->hasData())
            applyController(gamepad);
        vTaskDelay(pdMS_TO_TICKS((int)(CTRL_DT * 1000)));   // 50 Hz
    }
}
#endif // USE_XBOX_CONTROLLER
