package org.firstinspires.ftc.teamcode.Autonomous_Mode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.Objects;

@Autonomous(name = "Teste_de_Visao")
public class Teste_de_Visao extends LinearOpMode {

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    Servo s0;
    Servo s1;
    DcMotor motor_garra;
    private boolean april1 = false;
    private boolean april2 = false;
    private boolean april3 = false;
    private boolean april4 = false;
    private boolean april5 = false;
    private boolean april6 = false;
    private static final boolean USE_WEBCAM = true;

    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CenterStage.tflite";
    private static final String[] LABELS = {
            "ObjetoV",
    };

    private AprilTagProcessor aprilTag;

    private TfodProcessor tfod;

    private VisionPortal myVisionPortal;
    boolean viu = false;
    boolean viu_2 = false;
    int valor = 1;

    @Override
    public void runOpMode() throws InterruptedException{

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        s0 = hardwareMap.get(Servo.class, "s0");
        s1 = hardwareMap.get(Servo.class, "s1");
        motor_garra = hardwareMap.get(DcMotor.class, "motor_garra");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        motor_garra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initDoubleVision();

        CamAsync();

        waitForStart();

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        sleep(1000);

        if (currentRecognitions.size() == 1) {
            viu = true;
        } else {
            viu = false;
        }

        sleep(1000);

        walk("spinright",-240,0.5);

        sleep(1000);

        if (currentRecognitions.size() == 1) {
            viu_2 = true;
        } else {
            viu_2 = false;
        }

        if(viu && !viu_2){
            telemetry.addLine("Esquerda");
            sleep(10000);
            walk("spinright",240,0.5);
            walk("front", 30, 0.5);
            walk("spinright",480,0.5);
            walk("front", 20, 0.5);
            walk("back", 18, 0.5);
            walk("spinleft",480,0.5);
            walk("right", 80, 0.5);
            walk("spinright",960,0.5);
            garra("baixo", 300, 0.5);
            walk("front", 15, 0.5);
            walk("left", 16, 0.5);
            servo_garra("backdrop");
            walk("front", 7, 0.5);
            servo_pixel("soltar");
            garra("cima", 90, 0.5);
            walk("back", 17, 0.5);
            walk("spinleft",2000,0.5);
            walk("left", 50, 0.5);
            walk("back", 35, 0.5);
        }else if(viu && viu_2){
            telemetry.addLine("meio");
            sleep(10000);
        } else if(!viu && viu_2){
            telemetry.addLine("direita");
            sleep(10000);
        }

        sleep(1000);


    }

    private void initDoubleVision() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(509.98, 509.98, 312.107, 256.626)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

    }

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

        }

    }

    private void updateCameraFeed() {
        if (aprilTag.getDetections().size() > 0) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

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

    public void walk(String direcao, int cm, double power) {
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cm = cm*22;

        if(Objects.equals(direcao, "front")) {
            frontleft.setTargetPosition(-cm);
            frontright.setTargetPosition(-cm);
            backright.setTargetPosition(cm);
            backleft.setTargetPosition(cm);
        } else if(Objects.equals(direcao, "back")) {
            frontleft.setTargetPosition(cm);
            frontright.setTargetPosition(cm);
            backright.setTargetPosition(-cm);
            backleft.setTargetPosition(-cm);
        } else if(Objects.equals(direcao, "right")) {
            frontleft.setTargetPosition(-cm);
            frontright.setTargetPosition(cm);
            backright.setTargetPosition(cm);
            backleft.setTargetPosition(-cm);
        } else if(Objects.equals(direcao, "left")) {
            frontleft.setTargetPosition(cm);
            frontright.setTargetPosition(-cm);
            backright.setTargetPosition(-cm);
            backleft.setTargetPosition(cm);
        } else if(Objects.equals(direcao, "spinleft")) {
            cm = cm/22;
            frontleft.setTargetPosition(cm);
            frontright.setTargetPosition(-cm);
            backright.setTargetPosition(cm);
            backleft.setTargetPosition(-cm);
        } else if(Objects.equals(direcao, "spinright")) {
            cm = cm/22;
            frontleft.setTargetPosition(-cm);
            frontright.setTargetPosition(cm);
            backright.setTargetPosition(-cm);
            backleft.setTargetPosition(cm);
        }

        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setPower(power);
        frontright.setPower(power);
        backleft.setPower(power);
        backright.setPower(power);

        while (frontleft.isBusy() && frontright.isBusy() && backright.isBusy() && backleft.isBusy()) {
            // Aguarde
        }
    }

    public void garra(String direcao, int ticks, double power) {
        motor_garra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_garra.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(Objects.equals(direcao, "cima")) {
            motor_garra.setTargetPosition(ticks);
            motor_garra.setPower(0);
        } else if(Objects.equals(direcao, "baixo")) {
            motor_garra.setTargetPosition(-ticks);
            motor_garra.setPower(0);

        }

        motor_garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor_garra.setPower(power);

        while (motor_garra.isBusy()) {
            // Aguarde
        }
    }

    public void servo_garra(String mode) {
        if(Objects.equals(mode, "colocando")) {
            s0.setPosition(0.5);
            sleep(1000);
        } else if(Objects.equals(mode,"pegando")) {
            s0.setPosition(0.05);
            sleep(1000);
        } else if(Objects.equals(mode, "recolher")) {
            s0.setPosition(0.3);
            sleep(1000);
        } else if(Objects.equals(mode, "backdrop")) {
            s0.setPosition(0.15);
            sleep(1000);
        }
    }

    public void servo_pixel(String pixel) {
        if(Objects.equals(pixel, "pegar")) {
            s1.setPosition(1);
            sleep(1000);
        } else if(Objects.equals(pixel,"soltar")) {
            s1.setPosition(0.5);
            sleep(1000);
        }
    }

    public void CamAsync() {
        Thread garraThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!isStopRequested())  {

                    if (opModeInInit()) {
                        telemetry.addData("DS preview on/off","3 dots, Camera Stream");
                        telemetry.addLine();
                        telemetry.addLine("----------------------------------------");
                    }

                    if (myVisionPortal.getProcessorEnabled(aprilTag)) {
                        // User instructions: Dpad left or Dpad right.
                        telemetry.addLine("Dpad Left to disable AprilTag");
                        telemetry.addLine();
                        telemetryAprilTag();
                    } else {
                        telemetry.addLine("Dpad Right to enable AprilTag");
                    }
                    telemetry.addLine();
                    telemetry.addLine("----------------------------------------");
                    if (myVisionPortal.getProcessorEnabled(tfod)) {
                        telemetry.addLine("Dpad Down to disable TFOD");
                        telemetry.addLine();
                        telemetryTfod();
                    } else {
                        telemetry.addLine("Dpad Up to enable TFOD");
                    }

                    telemetry.update();

                    if (gamepad1.dpad_left) {
                        myVisionPortal.setProcessorEnabled(aprilTag, false);
                    } else if (gamepad1.dpad_right) {
                        myVisionPortal.setProcessorEnabled(aprilTag, true);
                    }
                    if (gamepad1.dpad_down) {
                        myVisionPortal.setProcessorEnabled(tfod, false);
                    } else if (gamepad1.dpad_up) {
                        myVisionPortal.setProcessorEnabled(tfod, true);
                    }

                    sleep(20);

                    if (aprilTag.getDetections().size() > 0) {
                        for (AprilTagDetection tag1 : aprilTag.getDetections()) {
                            if (tag1.id == 1) {
                                april1 = true;
                            }
                        }

                        for (AprilTagDetection tag2 : aprilTag.getDetections()) {
                            if (tag2.id == 2) {
                                april2 = true;
                            }
                        }

                        for (AprilTagDetection tag3 : aprilTag.getDetections()) {
                            if (tag3.id == 3) {
                                april3 = true;
                            }
                        }

                        for (AprilTagDetection tag4 : aprilTag.getDetections()) {
                            if (tag4.id == 4) {
                                april4 = true;
                            }
                        }

                        for (AprilTagDetection tag5 : aprilTag.getDetections()) {
                            if (tag5.id == 5) {
                                april5 = true;
                            }
                        }

                        for (AprilTagDetection tag6 : aprilTag.getDetections()) {
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

                    updateCameraFeed();

                }
            }
        });

        garraThread.start();
    }

}
