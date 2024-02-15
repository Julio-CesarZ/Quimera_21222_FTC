package org.firstinspires.ftc.teamcode.OpenCV;

import android.util.Size;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class Vision_Movimentacao extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    private boolean april1 = false;
    private boolean april2 = false;
    private boolean april3 = false;
    private boolean april4 = false;
    private boolean april5 = false;
    private boolean april6 = false;

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(509.98, 509.98, 312.107, 256.626)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        Timer cameraUpdateTimer = new Timer();
        cameraUpdateTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                updateCameraFeed();
            }
        }, 0, 1000);

        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0) {
                for (AprilTagDetection tag1 : tagProcessor.getDetections()) {
                    if (tag1.id == 1) {
                        april1 = true;
                    }
                }

                for (AprilTagDetection tag2 : tagProcessor.getDetections()) {
                    if (tag2.id == 2) {
                        april2 = true;
                    }
                }

                for (AprilTagDetection tag3 : tagProcessor.getDetections()) {
                    if (tag3.id == 3) {
                        april3 = true;
                    }
                }

                for (AprilTagDetection tag4 : tagProcessor.getDetections()) {
                    if (tag4.id == 4) {
                        april4 = true;
                    }
                }

                for (AprilTagDetection tag5 : tagProcessor.getDetections()) {
                    if (tag5.id == 5) {
                        april5 = true;
                    }
                }

                for (AprilTagDetection tag6 : tagProcessor.getDetections()) {
                    if (tag6.id == 6) {
                        april6 = true;
                    }
                }
            } else {
                april1 = false;
                april2 = false;
                april3 = false;
                april4 = false;
                april5 = false;
                april6 = false;
            }

            if (april1) {
                Walk(-0.5, 1000);
            } else {
                Walk(0, 1);
            }

            updateCameraFeed();

        }

    }

    public void Walk(double x, long z) {
        frontleft.setPower(x);
        frontright.setPower(x);
        backleft.setPower(-x);
        backright.setPower(-x);
        sleep(z);
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

    private void updateCameraFeed() {
        if (tagProcessor.getDetections().size() > 0) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            for (AprilTagDetection detection : detections) {
                if (detection.id == 1) {
                    telemetry.addLine(String.format("Tag1_XYZ %6.2f %6.2f %6.2f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                } else if (detection.id == 2) {
                    telemetry.addLine(String.format("Tag2_XYZ %6.2f %6.2f %6.2f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                } else if (detection.id == 3) {
                    telemetry.addLine(String.format("Tag3_XYZ %6.2f %6.2f %6.2f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                } else if (detection.id == 4) {
                    telemetry.addLine(String.format("Tag4_XYZ %6.2f %6.2f %6.2f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                } else if (detection.id == 5) {
                    telemetry.addLine(String.format("Tag5_XYZ %6.2f %6.2f %6.2f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                } else if (detection.id == 6) {
                    telemetry.addLine(String.format("Tag6_XYZ %6.2f %6.2f %6.2f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                }
                telemetry.addData("tag 1", april1);
                telemetry.addData("tag 2", april2);
                telemetry.addData("tag 3", april3);
                telemetry.addData("tag 4", april4);
                telemetry.addData("tag 5", april5);
                telemetry.addData("tag 6", april6);
                telemetry.update();
            }
        }
    }
}