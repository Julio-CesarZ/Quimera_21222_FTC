package org.firstinspires.ftc.teamcode.TeleOp_Mode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_Atualizado")
public class TeleOp_Atualizado extends LinearOpMode {

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor motor_garra;
    private Servo s0;
    private Servo s1;
    private Servo s5;

    private boolean leftTriggerPressed = false;

    private boolean rightTriggerPressed = false;

    private boolean buttonR;

    private float lastStickValue;

    private String mode = "txt";
    private boolean pixel;
    private double startTime;
    private double servoSpeed = 0.1;
    boolean portao_aberto = false;

    @Override
    public void runOpMode() {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        motor_garra = hardwareMap.get(DcMotor.class, "motor_garra");
        s0 = hardwareMap.get(Servo.class, "s0");
        s1 = hardwareMap.get(Servo.class, "s1");
        s5 = hardwareMap.get(Servo.class, "s5");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        motor_garra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        float Y;
        float X;
        float Z;
        float Y2;
        float X2;
        float Z2;
        boolean garra_lenta = true;
        double maxPower = 1;
        boolean rightTrigger2Pressed = false;

        waitForStart();
        while(opModeIsActive()) {
            Y = gamepad1.left_stick_y;
            X = gamepad1.left_stick_x;
            Z = gamepad1.right_stick_x;
            Y2 = gamepad2.left_stick_y;
            Z2 = gamepad2.right_stick_y;


            if (gamepad1.left_trigger > 0.1 && !leftTriggerPressed && maxPower > 0.0) {
                maxPower = 0.5;
                mode = "Peguei! ";
            }
            leftTriggerPressed = gamepad1.left_trigger > 0.1;

            // Ajusta o maxPower se o gatilho direito for pressionado e não estava pressionado anteriormente
            if (gamepad1.right_trigger > 0.1 && !rightTriggerPressed && maxPower < 1.0) {
                maxPower = 1.0;
                mode = "Veloz! ";
            }
            rightTriggerPressed = gamepad1.right_trigger > 0.1;

            // Garante que maxPower esteja no intervalo [0, 1]
            maxPower = Math.max(0.1, Math.min(1.0, maxPower));

            X *= maxPower;
            Y *= maxPower;
            Z *= maxPower;

            if (buttonR) {
                s0.setPosition(0.5);
                if(gamepad2.right_stick_y > 0) {
                    s0.setDirection(Servo.Direction.FORWARD);
                    s0.setPosition(Z2);
                    lastStickValue = Z2;
                } else if(gamepad2.right_stick_y < 0) {
                    s0.setDirection(Servo.Direction.REVERSE);
                    s0.setPosition(-Z2);
                    lastStickValue = -Z2;
                } else if(gamepad2.x) {
                    s0.setPosition(0.05);
                    buttonR = false;
                }
            }

            if(gamepad2.y && pixel) {
                pixel = false;
                s1.setPosition(-0.5);
                sleep(200);
            } else if (gamepad2.y && !pixel) {
                pixel = true;
                s1.setPosition(1);
                sleep(200);
            }

            if(gamepad2.right_trigger > 0.2 && !portao_aberto) {
                s5.setPosition(0.7);
                sleep(1350);
                s5.setPosition(0.5);
                portao_aberto = !portao_aberto;
            } else if (gamepad2.right_trigger > 0.2 && portao_aberto) {
                s5.setPosition(0.3);
                sleep(900);
                s5.setPosition(0.5);
                portao_aberto = !portao_aberto;
            }

            if (gamepad2.right_trigger > 0.1 && !rightTrigger2Pressed) {
                startTime = getRuntime();
                rightTrigger2Pressed = true;
            } else if (gamepad2.right_trigger <= 0.1 && rightTrigger2Pressed) {
                rightTrigger2Pressed = false;
            }

            if (gamepad2.left_bumper && !buttonR) {
                buttonR = true;
                sleep(200);
            } else if (gamepad2.left_bumper && buttonR) {
                buttonR = false;
                sleep(200);
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
            telemetry.addData("Garra: ", garra_lenta);
            telemetry.addData("Mode: ", mode);
            telemetry.addData("Servo: ", buttonR);
            telemetry.addData("pixel", pixel);
            telemetry.update();
        }
    }
}
