// ============================================================
//  LineBench.cpp — "Line Bench"  (TELEOP)
//  GLITCHED, Team 190, robot "Pixel".  NOT YET RUN ON THE ROBOT.
//
//  From the NRL programming guide (SDK 1.5.0), 3.4 "a bench tester with
//  buttons" and Part 8: "test each step on its own with a bench TELEOP from
//  the same start spot, 5 times. Find the first step that isn't repeatable."
//
//  One button = one LineActions step. Same tuning as "TerminalTests"
//  (GlitchTuning.h), so what works here works there.
//
//    X           follow to the next junction (corners turn by themselves)
//    D-pad UP    follow to the end of the line / onto the end bar
//    D-pad LEFT  left()  — use where a junction step stopped
//    D-pad RIGHT right() — use where a junction step stopped
//    D-pad DOWN  U-turn
//    RB          markHere()  — remember the junction the robot is stopped on
//    A           retraceTo() — back to the marked junction (after a U-turn)
//    LB          follow into the black box, then park(ms)
//    Y           STOP everything + fresh start (fault, counts, route cleared)
//                 -> press Y after picking the robot up and placing it again
//    LT          exits the OpMode (controller)
//
//  A new step only starts when the previous one has finished ("busy no").
//  TELEOP, so the drive sticks are NOT used here: the robot only moves when
//  a step runs.
// ============================================================
#include "NRL.h"
#include "GlitchTuning.h"

namespace {   // file-local: other OpModes have their own hardware objects

using namespace glitch;

// ── Hardware (same as TerminalTests / Start to Park) ───────────────────────
HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM,
                         .flipped = false, .minDrive = 34 }};
HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM,
                         .flipped = true,  .minDrive = 32 }};
TankDrive drive(leftMotor, rightMotor);

// Arm held stowed (110) so it can't hang in front of the sensor bar.
// Stow it by hand before INIT so INIT doesn't move it.
HexaServo arm{{ .signalPin = SERVO_1, .startAngle = 110.0f, .settleMs = 0 }};

HexaLineArray line{ hexaLineConfig() };
LineActions lineActions(drive, line, testTuning());

int benchMark = -1;                 // static: read later by retraceTo()
const char* lastStep = "-";         // the last button's step, for the screen

}  // namespace

class LineBench : public NRLOpMode {
public:
    void init() override {
        // Nothing here moves: motors begin stopped, arm holds its stowed pose.
        leftMotor.begin();
        rightMotor.begin();
        arm.begin();
        line.begin();
    }

    void start() override {
        freshStart();
    }

    void loop() override {
        if (gamepad1.justPressed(BTN_Y)) {          // STOP everything
            cancelActions();
            drive.stop();
            freshStart();
            lastStep = "STOP (Y)";
        }

        if (!isActionRunning()) {                   // one step at a time
            if (gamepad1.justPressed(BTN_X)) {
                lastStep = "to junction";
                runAction(lineActions.followToJunction());
            } else if (gamepad1.justPressed(BTN_DPAD_UP)) {
                lastStep = "to end";
                runAction(lineActions.followToEnd());
            } else if (gamepad1.justPressed(BTN_DPAD_LEFT)) {
                lastStep = "left";
                runAction(lineActions.left());
            } else if (gamepad1.justPressed(BTN_DPAD_RIGHT)) {
                lastStep = "right";
                runAction(lineActions.right());
            } else if (gamepad1.justPressed(BTN_DPAD_DOWN)) {
                lastStep = "U-turn";
                runAction(lineActions.uTurn());
            } else if (gamepad1.justPressed(BTN_RB)) {
                lastStep = "mark";
                runAction(lineActions.markHere(&benchMark));
            } else if (gamepad1.justPressed(BTN_A)) {
                lastStep = "retrace";
                runAction(lineActions.retraceTo(&benchMark));
            } else if (gamepad1.justPressed(BTN_LB)) {
                lastStep = "box+park";
                runAction(lineActions.auto_()
                    .followToBox()
                    .park(PARK_MS)
                    .build());
            }
        }

        // ── Telemetry: 8 lines max (guide 3.4 / 5.3) ──
        static char stepStr[32], counts[24], lastStr[16], maskStr[9];
        snprintf(stepStr, sizeof(stepStr), "%s %s", lastStep,
                 isActionRunning() ? "(busy)" : "(done)");
        snprintf(counts, sizeof(counts), "J%d C%d M%d",
                 lineActions.junctionCount(), lineActions.cornerCount(), benchMark);
        const LineJunction& j = lineActions.lastJunction();
        snprintf(lastStr, sizeof(lastStr), "%c%c%c d%d",
                 j.left ? 'L' : '-', j.right ? 'R' : '-', j.straight ? 'S' : '-',
                 lineActions.routeDepth());
        uint8_t m = line.mask();
        for (int i = 0; i < 8; i++) maskStr[i] = (m & (1 << i)) ? '#' : '.';
        maskStr[8] = 0;

        telemetry.addData("step", stepStr);
        telemetry.addData("fault", lineActions.faultName());
        telemetry.addData("count", counts);          // M = mark (-1 = none)
        telemetry.addData("last", lastStr);          // junction L/R/S + route depth
        telemetry.addData("turn deg", lineActions.lastTurnDegrees()); // 80-100 after left/right
        telemetry.addData("grid deg", lineActions.gridHeading());     // ~0/±90/180 on a line
        telemetry.addData("mask", maskStr);          // ch0 (LEFT-most) first
        telemetry.addData("err", (float)line.error());
    }

    void stop() override {
        cancelActions();
        drive.stop();
    }

private:
    // Fresh start (guide 4.6): clear fault, counts and route memory. The next
    // step sets grid 0 deg from the way the robot faces, so place it square.
    void freshStart() {
        lineActions.setTuning(testTuning());
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();
        benchMark = -1;
    }
};

REGISTER_OPMODE(LineBench, "Line Bench", TELEOP);
