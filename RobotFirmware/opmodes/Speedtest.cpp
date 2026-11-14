#include "NRL.h"

// Bench tool: measures how far the robot travels while line-following for a
// fixed time. Delete this file before competition if you like.
//   X       -> follow the line for 3 s, then stop
//   D-pad up -> follow the line for 5 s, then stop
//   Y       -> emergency stop
static HexaDCMotor leftMotor{{.dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true, .minDrive = 29}};
static HexaDCMotor rightMotor{{.dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .minDrive = 32}};
static TankDrive drive(leftMotor, rightMotor);
static HexaLineArray line{hexaLineConfig()};
static LineActions lineActions(drive, line);

class SpeedTest : public NRLOpMode
{
    float heldIn = 0.0f;   // IMU distance, frozen when a run ends
    bool wasBusy = false;

public:
    void init() override
    {
        leftMotor.begin();
        rightMotor.begin();
        line.begin();
    }

    void start() override
    {
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();
    }

    void loop() override
    {
        if (gamepad1.justPressed(BTN_Y))
        {
            cancelActions();
            drive.stop();
            lineActions.clearFault();
        }

        if (!isActionRunning())
        {
            if (gamepad1.justPressed(BTN_X))
            {
                NRLComms::resetDistance();
                runAction(lineActions.auto_().followFor(3000).stop().build());
            }
            if (gamepad1.justPressed(BTN_DPAD_UP))
            {
                NRLComms::resetDistance();
                runAction(lineActions.auto_().followFor(5000).stop().build());
            }
        }

        telemetry.addData("fault", lineActions.faultName());
        telemetry.addData("busy", isActionRunning() ? "yes" : "no");
        telemetry.addData("err", (float)line.error());

        // Live while running; frozen at the moment the run finishes.
        bool busy = isActionRunning();
        if (busy || !wasBusy)
        {
            if (busy) heldIn = NRLComms::getDistanceInches();
        }
        else
        {
            heldIn = NRLComms::getDistanceInches(); // run just ended: capture once
        }
        wasBusy = busy;
        telemetry.addData("imu in", heldIn);
    }

    void stop() override { drive.stop(); }
};

REGISTER_OPMODE(SpeedTest, "Speed Test", TELEOP);