#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
// static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor leftMotor{{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM}};
static HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM }};
static HexaServo arm{{.signalPin=SERVO_1, .startAngle = 90.0f, .settleMs = 0}};
static HexaServo gripper{{.signalPin=SERVO_2, .startAngle = 90.0f, .settleMs = 0}};
static TankDrive drive(leftMotor, rightMotor);

class Basic : public NRLOpMode {
public:
    float rightY, rightX, leftX, leftY;
    float angle_arm = 90;
    void init() override {
        // Runs once when INIT is pressed — begin() your hardware here.
        leftMotor.begin();
        rightMotor.begin();
        arm.begin();
        gripper.begin();
        arm.setPosition(90);
        gripper.setPosition(95);
    }

    void loop() override {
        rightY = gamepad1.rightY();
        leftX = gamepad1.leftX();
        rightX = -gamepad1.rightX();
        leftY = -gamepad1.leftY();


        // Runs at 100 Hz until STOP — read gamepad1, drive motor¨s here.
        //
        // Control each part in ONE place: loop() or an action, not
        // both. Actions run before loop() each tick, so loop() would
        // overwrite them. To share one part, step aside while a
        // sequence plays:  if (!isActionRunning()) { ... }
        drive.setScale(gamepad1.pressed(BTN_LB) ? 0.35f : 1.0f); // slow while LB held
        drive.drive(rightX, leftY);
        if (gamepad1.justPressed(BTN_RB)){
            gripper.setPosition(gripper.getPosition() == 95 ? 30 : 95);
        }
        angle_arm += rightY;
        if (angle_arm < 90){
            angle_arm = 90;
        }else if (angle_arm > 180){
            angle_arm = 180;
        }
        arm.setPosition(angle_arm);
        telemetry.addData("right X", rightX);
        telemetry.addData("left X", leftX);
        telemetry.addData("right Y", rightY);
        telemetry.addData("left Y", leftY);

    }

    void stop() override {
        rightMotor.stop();
        leftMotor.stop();
        arm.detach();
 }
};

REGISTER_OPMODE(Basic, "Basic", TELEOP);
