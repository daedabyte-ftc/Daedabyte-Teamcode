package org.firstinspires.ftc.teamcode;

// Import classes for LinearOpMode, TeleOp annotation, and motor control
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //===========  Initialize hardware ===========//
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
        Servo launcherServo = hardwareMap.servo.get("launcherServo");
        launcherServo.setPosition(0);

        // motor directions (per-motor, verified)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // ensure motors stop immediately when power is zero
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ==========================
        // 3. Wait for Driver to Start
        // ==========================
        waitForStart(); // Pauses the program until the play button is pressed

        if (isStopRequested()) return; // If stop is requested immediately, exit

        // ==========================
        // 4. Main TeleOp Loop
        // ==========================
        while (opModeIsActive()) {

            // --------------------------
            // a) Read joystick inputs
            // --------------------------
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x;  // Left/right strafing
            double rx = gamepad1.right_stick_x; // Rotation (left/right)

            // --------------------------
            // b) Calculate denominator for scaling
            // --------------------------
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // --------------------------
            // c) Calculate motor powers for mecanum drive
            // --------------------------
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // --------------------------
            // d) Apply power to motors
            // --------------------------
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --------------------------
            // e) Intake
            // --------------------------

            if (gamepad1.a) {
                intakeMotor.setPower(1);
            }
            else if (gamepad1.b) {
                intakeMotor.setPower(-1);
            }
            else {
                intakeMotor.setPower(0);
            }

            // --------------------------
            // f) Launcher Motor (R2)
            // --------------------------
            if (gamepad1.right_trigger > 0.1) {
                launcherMotor.setPower(0.75);
            }
            else if (gamepad1.right_bumper) {
                launcherMotor.setPower(0.5);
            }
            else if (gamepad1.left_trigger > 0.1) {
                launcherMotor.setPower(-0.75);
            }
            else if (gamepad1.left_bumper) {
                launcherMotor.setPower(-0.25);
            }
            else {
                launcherMotor.setPower(0);
            }

            // --------------------------
            // f) Launcher Servo (R2)
            // --------------------------

            if (gamepad1.x) {
                launcherServo.setPosition(0.2);
                }
            else if (gamepad1.y) {
                launcherServo.setPosition(0);
            }
            else {
                launcherServo.setPosition(0);
            }


        }
    }
}
