package org.firstinspires.ftc.teamcode.TeleOp_Mode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.CompletableFuture;

@TeleOp(name = "TeleOp_ULTRA_Poggers")
public class TeleOp_ULTRA_Poggers extends LinearOpMode {

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor motor_garra;
    private Servo s0;
    private Servo s1;
    private Servo s2;
    private Servo s5;

    private boolean leftTriggerPressed = false;

    private boolean rightTriggerPressed = false;

    private float lastStickValue;

    private String mode = "txt";
    private boolean pixel;
    private double startTime;
    boolean portao_aberto = false;
    boolean aviao = false;
    int exe = 1;

    @Override
    public void runOpMode() {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        motor_garra = hardwareMap.get(DcMotor.class, "motor_garra");
        s0 = hardwareMap.get(Servo.class, "s0");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s5 = hardwareMap.get(Servo.class, "s5");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        motor_garra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        float Y;
        float X;
        float Z;
        float Y2;
        boolean garra_lenta = true;
        double maxPower = 1;
        boolean rightTrigger2Pressed = false;

        waitForStart();
        while(opModeIsActive()) {
            Y = gamepad1.left_stick_y;
            X = gamepad1.left_stick_x;
            Z = gamepad1.right_stick_x;
            Y2 = gamepad2.left_stick_y;

            while(exe == 1) {
                exe = 0;
                s5.setPosition(1);
                portao_aberto = true;
                sleep(450);
                s5.setPosition(0.5);
            }

            if (gamepad1.left_trigger > 0.1 && !leftTriggerPressed && maxPower > 0.0) {
                maxPower = 0.5;
                mode = "Peguei! ";
            }
            leftTriggerPressed = gamepad1.left_trigger > 0.1;

            // Ajusta o maxPower se o gatilho direito for pressionado e não estava pressionado anteriormente
            if (gamepad1.right_trigger > 0.1 && !rightTriggerPressed && maxPower < 1.0) {
                maxPower = 2;
                mode = "Veloz! ";
            }
            rightTriggerPressed = gamepad1.right_trigger > 0.1;

            // Garante que maxPower esteja no intervalo [0, 1]
            maxPower = Math.max(0.1, Math.min(1.0, maxPower));

            X *= maxPower;
            Y *= maxPower;
            Z *= maxPower;

            if (gamepad2.left_bumper) {
                s0.setPosition(0.35);
            } else if(gamepad2.x) {
                s0.setPosition(0.06);
            }

            if(gamepad2.y && pixel) {
                pixel = false;
                s1.setPosition(0);
                sleep(200);
            } else if (gamepad2.y && !pixel) {
                pixel = true;
                s1.setPosition(0.5);
                sleep(200);
            }

            if(gamepad1.x && aviao) {
                aviao = false;
                s2.setPosition(-0.1);
                sleep(200);
            } else if (gamepad1.x && !aviao) {
                aviao = true;
                s2.setPosition(0.5);
                sleep(200);
            }

            if (gamepad2.right_trigger > 0.1 && !rightTrigger2Pressed) {
                startTime = getRuntime();
                rightTrigger2Pressed = true;
            } else if (gamepad2.right_trigger <= 0.1 && rightTrigger2Pressed) {
                rightTrigger2Pressed = false;
            }

            if (gamepad1.right_bumper && !portao_aberto) {
                s5.setPosition(1);
                portao_aberto = true;
                sleep(450);
                s5.setPosition(0.5);
            } else if (gamepad1.right_bumper && portao_aberto) {
                s5.setPosition(0);
                portao_aberto = false;
                sleep(450);
                s5.setPosition(0.5);
            } else if (gamepad1.y) {
                portao_aberto = false;
            }

            frontleft.setPower(-X);
            frontright.setPower(X);
            backleft.setPower(-X);
            backright.setPower(X);

            frontleft.setPower(Y);
            frontright.setPower(Y);
            backleft.setPower(-Y);
            backright.setPower(-Y);

            frontleft.setPower(-Z);
            frontright.setPower(Z);
            backleft.setPower(Z);
            backright.setPower(-Z);

            if (gamepad2.right_bumper){
                garra_lenta = !garra_lenta;
                sleep(300);
            }


            if (garra_lenta) {
                Y2 *= 0.3;
                motor_garra.setPower(Y2);
            } else {
                Y2 *= 0.7;
                motor_garra.setPower(Y2);
            }

            telemetry.addData("Power: ", maxPower);
            telemetry.addData("Garra rápida? ", garra_lenta);
            telemetry.addData("Mode: ", mode);
            telemetry.addData("pixel", pixel);
            telemetry.addData("portão: ", portao_aberto);
            telemetry.update();
        }
    }
}
