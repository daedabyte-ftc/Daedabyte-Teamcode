/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
