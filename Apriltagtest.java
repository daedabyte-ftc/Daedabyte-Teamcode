package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;


@TeleOp
public class ConceptAprilTag extends LinearOpMode {
    private DcMotor launcherMotor;
    private boolean autoAimEnabled = true;
    boolean xPrev = false;

    //Declare april tag processer hardware/software
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
        initAprilTag();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                boolean xPressed = gamepad1.x;

                if (gamepad1.x && !xPrev) {
                    autoAimEnabled = !autoAimEnabled;
                }
                xPrev = xPressed;

                List<AprilTagDetection> detections = aprilTag.getDetections();

                if (autoAimEnabled && !detections.isEmpty()) {
                    AprilTagDetection tag = detections.get(0);
                    telemetry.addData("Auto-Aim Enabled", autoAimEnabled);

                    // ===== YAW AUTO AIM =====
                    double yawError = tag.ftcPose.yaw; // degrees
                    double turnPower = yawError * 0.015;

                    // TODO: replace with your drivetrain turn method
                    // drive.turn(turnPower);

                    // ===== DISTANCE → SHOOTER POWER (ms-style control) =====
                    double distanceInches = tag.ftcPose.range;

                    double launcherPower;
                    if (distanceInches < 40) {
                        launcherPower = 0.65;
                    } else if (distanceInches < 60) {
                        launcherPower = 0.75;
                    } else {
                        launcherPower = 0.85;
                    }

                    launcherMotor.setPower(launcherPower);

                } else {
                    launcherMotor.setPower(0);
                    telemetry.addLine("No Tag Detected");
                }

                telemetry.update();
                sleep(20); // give CPU a break

            }
        }
    }

    // Create the AprilTag processor
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }
}
