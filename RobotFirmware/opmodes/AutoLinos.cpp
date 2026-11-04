#include "NRL.h"
static HexaDCMotor leftMotor{{.dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true, .minDrive = 29}};
static HexaDCMotor rightMotor{{.dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM,

                               .minDrive = 32}};
static TankDrve drive(leftMotor, rightMotor);
static HexaLineArray line{hexaLineConfig()};
static LineActions lineActions(drive, line);
class FirstJunction : public NRLOpMode
{
public:
    void init() override
    {
        leftMotor.begin();
        rightMotor.begin();
        line.begin();
    }
    void start() override
    {
        lineActions.clearFault();  // a fresh run: forget old faults,
        lineActions.resetCounts(); // counts,
        lineActions.clearRoute();  // and route memory
        runAction(lineActions.auto_()
                      .followToJunction() // follow; corners turn by themselves
                      .build());
    }
    void loop() override
    {
        const LineJunction &j = lineActions.lastJunction();
        char lrs[4] = {j.left ? 'L' : '-', j.right ? 'R' : '-', j.straight ? 'S' : '-', 0};
        telemetry.addData("fault", lineActions.faultName());
        telemetry.addData("jx", (float)lineActions.junctionCount());
        telemetry.addData("last", lrs);
        telemetry.addData("err", (float)line.error());
    }
    void stop() override { drive.stop(); }
};
REGISTER_OPMODE(FirstJunction, "First Junction", AUTO);
