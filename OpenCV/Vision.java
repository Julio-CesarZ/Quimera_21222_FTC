package org.firstinspires.ftc.teamcode.OpenCV;

// Importações necessárias
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
public class Vision extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    private boolean april1 = false;
    private boolean april2 = false;
    private boolean april3 = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Inicialização do processador e portal de visão
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


        // Aguardar o início da partida
        waitForStart();

        // Configurar um Timer para atualizar o feed da câmera a cada X milissegundos
        Timer cameraUpdateTimer = new Timer();
        cameraUpdateTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                updateCameraFeed();
            }
        }, 0, 1000); // Atualizar a cada 1 segundo (ajuste conforme necessário)

        while (!isStopRequested() && opModeIsActive()) {

            // Reinicialize as variáveis como falsas antes de verificar as tags
            april1 = false;
            april2 = false;
            april3 = false;

            // Verificar se há tags detectadas
            if (tagProcessor.getDetections().size() > 0) {
                // Iterar por todas as tags encontradas
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
            } else {
                april1 = false;
                april2 = false;
                april3 = false;
            }

            updateCameraFeed();

        }

    }

    // Método para atualizar o feed da câmera
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
                }
            }
            telemetry.addData("tag 1", april1);
            telemetry.addData("tag 2", april2);
            telemetry.addData("tag 3", april3);
            telemetry.update();
        }
    }
}