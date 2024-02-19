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

@Autonomous(name = "Right_Blue")
public class Autonomous_ULTRA_Poggers_Right_Blue extends LinearOpMode {

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    Servo s0;
    Servo s1;
    Servo s5;
    DcMotor motor_garra;
    private boolean april1 = false;
    private boolean april2 = false;
    private boolean april3 = false;
    private boolean april4 = false;
    private boolean april5 = false;
    private boolean april6 = false;
    int sair = 2;
    private static final boolean USE_WEBCAM = true;

    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CenterStage.tflite";
    private static final String[] LABELS = {
            "ObjetoV",
    };

    private AprilTagProcessor aprilTag;

    private TfodProcessor tfod;

    private VisionPortal myVisionPortal;

    float cameraWidth = 640;

    String teamSupportPosition = "desconhecida";

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        s0 = hardwareMap.get(Servo.class, "s0");
        s1 = hardwareMap.get(Servo.class, "s1");
        s5 = hardwareMap.get(Servo.class, "s5");
        motor_garra = hardwareMap.get(DcMotor.class, "motor_garra");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        motor_garra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initDoubleVision();

        waitForStart();

        sleep(1000);

        while (!isStopRequested() && opModeIsActive()) {

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

            List<Recognition> recognitions = tfod.getRecognitions();

            if (recognitions.size() > 0) {
                teamSupportPosition = determineTeamSupportPosition(recognitions);

                updateCameraFeed();

                // Tomar decisões com base na posição do objeto da equipe
                if (teamSupportPosition.equals("esquerda")) {
                    telemetry.addData(teamSupportPosition, " Detectado");
                    telemetry.update();

                    esquerda();
                    break;

                } else if (teamSupportPosition.equals("meio")) {
                    telemetry.addData(teamSupportPosition, " Detectado");
                    telemetry.update();

                    meio();
                    break;

                } else if (teamSupportPosition.equals("direita")) {
                    telemetry.addData(teamSupportPosition, " Detectado");
                    telemetry.update();

                    direita();
                    break;

                } else {
                    teamSupportPosition = "desconhecida";
                }

            } else {
                telemetry.addData(teamSupportPosition, "Detectado");
                telemetry.update();

                esquerda();
                break;

            }
        }
    }

    public void esquerda() {
        servo_porta("fechar");
        servo_porta("abrir");
        walk("front", 40, 0.5);
        walk("spinleft", 480, 0.5);
        walk("front", 25, 0.5);
        walk("back", 25, 0.5);
        walk("spinleft", 540, 0.5);
        walk("left", 18, 0.5);
        garra("baixo", 300, 0.5);
        servo_porta("fechar");
        walk("front", 80, 0.5);
        walk("right", 18, 0.5);
        walk("front", 25,0.5);
        servo_garra("backdrop");
        garra("cima", 70, 0.3);
        servo_pixel("soltar");
        walk("back", 20, 0.8);
        walk("spinright", 1920, 0.5);
        servo_porta("abrir");
        walk("right", 38, 0.5);
        walk("back", 30, 0.5);
    }
    public void meio() {
        servo_porta("fechar");
        servo_porta("abrir");
        walk("front", 70, 0.5);
        walk("back", 15, 0.5);
        walk("spinleft", 1000, 0.5);
        garra("baixo", 300, 0.5);
        servo_porta("fechar");
        walk("front", 80, 0.5);
        walk("right", 10, 0.5);
        walk("front", 15, 0.5);
        sleep(1000);
        leitura_x(2);
        servo_garra("backdrop");
        garra("cima", 85, 0.5);
        walk("front", 15, 0.5);
        servo_pixel("soltar");
        walk("back", 20, 0.5);
        walk("spinright", 1920, 0.5);
        servo_porta("abrir");
        walk("right", 75, 0.8);
        walk("back", 35, 0.5);
    }
    public void direita() {
        servo_porta("fechar");
        servo_porta("abrir");
        walk("front", 40, 0.5);
        walk("spinright", 480, 0.5);
        walk("front", 24, 0.5);
        walk("back", 24, 0.5);
        walk("spinleft", 1440, 0.5);
        garra("baixo", 300, 0.5);
        servo_porta("fechar");
        walk("front", 80, 0.5);
        walk("right", 45, 0.5);
        walk("front", 15, 0.5);
        sleep(1000);
        leitura_x(3);
        walk("front", 10, 0.5);
        servo_garra("backdrop");
        garra("cima", 70, 0.3);
        servo_pixel("soltar");
        walk("back", 20, 0.5);
        walk("spinright", 1920, 0.5);
        servo_porta("abrir");
        walk("right", 95, 0.5);
        walk("back", 35, 0.5);
    }
    public void leitura_x(int x){
        while (sair >= 1) {

            sair = sair - 1;

            updateCameraFeed();

            if (aprilTag.getDetections().size() > 0) {
                List<AprilTagDetection> detections = aprilTag.getDetections();

                for (AprilTagDetection detection : detections) {
                    if (detection.id == x) {
                         if (detection.ftcPose.x <= -2.2) {
                            while (detection.ftcPose.x <= -2.2) {
                                walk("left", 8, 0.5);
                                detections = aprilTag.getDetections();
                                for (AprilTagDetection updatedDetection : detections) {
                                    if (updatedDetection.id == x) {
                                        detection = updatedDetection;
                                        sleep(500);
                                        break;
                                    }
                                }
                            }

                        }
                    } else {
                        walk("left", 6, 0.5);
                        sleep(1000);
                    }
                }
            } else {
                walk("left", 6, 0.5);
                sleep(1000);
            }

        }
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
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
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

        cm = cm * 22;
        if (Objects.equals(direcao, "front")) {
            frontleft.setTargetPosition(-cm);
            frontright.setTargetPosition(-cm);
            backright.setTargetPosition(cm);
            backleft.setTargetPosition(cm);
        } else if (Objects.equals(direcao, "back")) {
            frontleft.setTargetPosition(cm);
            frontright.setTargetPosition(cm);
            backright.setTargetPosition(-cm);
            backleft.setTargetPosition(-cm);
        } else if (Objects.equals(direcao, "right")) {
            frontleft.setTargetPosition(-cm);
            frontright.setTargetPosition(cm);
            backright.setTargetPosition(cm);
            backleft.setTargetPosition(-cm);
        } else if (Objects.equals(direcao, "left")) {
            frontleft.setTargetPosition(cm);
            frontright.setTargetPosition(-cm);
            backright.setTargetPosition(-cm);
            backleft.setTargetPosition(cm);
        } else if (Objects.equals(direcao, "teste")) {
            frontleft.setTargetPosition(-cm);
            sleep(1000);
            frontright.setTargetPosition(-cm);
            sleep(1000);
            backright.setTargetPosition(cm);
            sleep(1000);
            backleft.setTargetPosition(cm);
            sleep(1000);
        } else if (Objects.equals(direcao, "spinleft")) {
            cm = cm / 22;
            frontleft.setTargetPosition(cm);
            frontright.setTargetPosition(-cm);
            backright.setTargetPosition(cm);
            backleft.setTargetPosition(-cm);
        } else if (Objects.equals(direcao, "spinright")) {
            cm = cm / 22;
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

        if (Objects.equals(direcao, "cima")) {
            motor_garra.setTargetPosition(ticks);
            motor_garra.setPower(0);
        } else if (Objects.equals(direcao, "baixo")) {
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
        if (Objects.equals(mode, "colocando")) {
            s0.setPosition(0.5);
            sleep(1000);
        } else if (Objects.equals(mode, "pegando")) {
            s0.setPosition(0.05);
            sleep(1000);
        } else if (Objects.equals(mode, "recolher")) {
            s0.setPosition(0.3);
            sleep(1000);
        } else if (Objects.equals(mode, "backdrop")) {
            s0.setPosition(0.15);
            sleep(1000);
        }
    }

    public void servo_porta(String mode) {
        if (Objects.equals(mode, "abrir")) {
            s5.setPosition(0.7);
            sleep(870);
            s5.setPosition(0.5);
        } else if (Objects.equals(mode, "fechar")) {
            s5.setPosition(0.3);
            sleep(900);
            s5.setPosition(0.5);
        }
    }


    public void servo_pixel(String pixel) {
        if (Objects.equals(pixel, "pegar")) {
            s1.setPosition(1);
            sleep(1000);
        } else if (Objects.equals(pixel, "soltar")) {
            s1.setPosition(0.5);
            sleep(1000);
        }
    }

    private String determineTeamSupportPosition(List<Recognition> recognitions) {
        // Se não detectar nada, retorna "desconhecido"
        if (recognitions.isEmpty()) {
            return "desconhecida";
        }

        // Primeira detecção
        Recognition firstRecognition = recognitions.get(0);

        // Coordenadas "x" da detecção em relação ao centro da imagem
        float centerX = firstRecognition.getLeft() + (firstRecognition.getWidth() / 2) - (cameraWidth / 2);

        // Valores no eixo "x"
        double leftLimit = -100;
        double rightLimit = 100;

        // Utiliza os valores do eixo "x" para ter definir a posição do objeto
        if (centerX < leftLimit) {
            return "esquerda";
        } else if (centerX > rightLimit) {
            return "direita";
        } else if (centerX < rightLimit && centerX > leftLimit) {
            return "meio";
        } else {
            return "desconhecida";
        }
    }

}
