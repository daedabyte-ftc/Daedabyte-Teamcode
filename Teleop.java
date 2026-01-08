package org.firstinspires.ftc.teamcode;

// Import classes for LinearOpMode, TeleOp annotation, and motor control
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //===========  Initialize hardware ===========//
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor frontIntakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor rearIntakeMotor = hardwareMap.dcMotor.get("intakeMotor2");
        Servo servoTest = hardwareMap.servo.get("servoTest");

        //reverse right motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
            double y = -gamepad1.left_stick_y; // Forward/backward; negative because joystick Y is reversed
            double x = gamepad1.left_stick_x * 1.1; // Left/right strafing; multiplied by 1.1 to compensate for imperfect strafing
            double rx = gamepad1.right_stick_x; // Rotation (turning) input from the right joystick

            // --------------------------
            // b) Calculate denominator for scaling
            // --------------------------
            // Ensures that the motor powers stay within the [-1, 1] range while maintaining ratios
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // --------------------------
            // c) Calculate motor powers for mecanum drive
            // --------------------------
            // Formula based on standard mecanum wheel kinematics
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
            // e) Intake Servo
            // --------------------------



        }
    }
}
