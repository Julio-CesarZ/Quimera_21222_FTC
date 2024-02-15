package org.firstinspires.ftc.teamcode.TeleOp_Mode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_Alternativo")
public class TeleOp_Alternativo extends LinearOpMode {

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor motor_garra;
    private Servo s0;
    private Servo s1;

    private boolean leftTriggerPressed = false;

    private boolean rightTriggerPressed = false;

    private boolean buttonR;

    private float lastStickValue;

    private String mode = "txt";
    private boolean pixel;

    @Override
    public void runOpMode() {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        motor_garra = hardwareMap.get(DcMotor.class, "motor_garra");
        s0 = hardwareMap.get(Servo.class, "s0");
        s1 = hardwareMap.get(Servo.class, "s1");
        DcMotor elevador = hardwareMap.get(DcMotor.class, "elevador");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        motor_garra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        float Y;
        float X;
        float Z;
        float Y2;
        float X2;
        float Z2;

        double maxPower = 1;

        waitForStart();
        while(opModeIsActive()) {
            Y = gamepad1.left_stick_y;
            X = gamepad1.right_stick_x;
            Z = gamepad1.left_stick_x;
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
            Y2 *= 0.3;

            if (buttonR) {
                s0.setPosition(0.5);
                if(gamepad2.right_stick_y > 0) {
                    s0.setDirection(Servo.Direction.FORWARD);
                    s0.setPosition(Z2);
                    lastStickValue = Z2;  // Atualiza o último valor do analógico
                } else if(gamepad2.right_stick_y < 0) {
                    s0.setDirection(Servo.Direction.REVERSE);
                    s0.setPosition(-Z2);
                    lastStickValue = -Z2;
                } else if(gamepad2.x) {
                    s0.setPosition(0.07);
                    buttonR = false;
                }
            }

            if(gamepad2.y && pixel) {
                pixel = false;
                s1.setPosition(0.5);
                sleep(200);
            } else if (gamepad2.y && !pixel) {
                pixel = true;
                s1.setPosition(1);
                sleep(200);
            }

            if (gamepad2.b) {
                elevador.setPower(1);  // Adjust power as needed
            } else {
                elevador.setPower(0);  // Stop the elevator motor if the 'B' button is not pressed
            }

            // Read the 'A' button on gamepad2 to move the elevator motor backward
            if (gamepad2.a) {
                elevador.setPower(-1);  // Adjust power as needed
            } else {
                elevador.setPower(0);  // Stop the elevator motor if the 'A' button is not pressed
            }

            if (gamepad2.left_bumper && !buttonR) {
                buttonR = true;
                sleep(200);
            } else if (gamepad2.right_bumper && buttonR) {
                s0.setPosition(lastStickValue);  // Configura a posição do servo com o último valor do analógico
                buttonR = false;  // Libera o botão R para permitir movimento do analógico novamente
                sleep(200);
            } else if (gamepad2.left_bumper && buttonR) {
                buttonR = false;
                sleep(200);
            }

            // Ajuste os motores para mover para a esquerda ou direita com base em X
            frontleft.setPower(-X); // Motor m0 para a esquerda
            frontright.setPower(X);  // Motor m1 para a direita
            backleft.setPower(-X); // Motor m2 para a esquerda
            backright.setPower(X);  // Motor m3 para a direita

            // Ajuste os motores para mover para cima ou para baixo com base em Y
            frontleft.setPower(Y);  // Motor m0 para cima
            frontright.setPower(Y);  // Motor m1 para cima
            backleft.setPower(-Y);  // Motor m2 para cima
            backright.setPower(-Y);  // Motor m3 para cima

            // Adicione a rotação controlada por Z
            frontleft.setPower(-Z);
            frontright.setPower(Z);
            backleft.setPower(Z);
            backright.setPower(-Z);

            motor_garra.setPower(Y2);

            telemetry.addData("Power: ", maxPower);
            telemetry.addData("Mode: ", mode);
            telemetry.addData("Servo: ", buttonR);
            telemetry.addData("pixel", pixel);
            telemetry.update();
        }
    }
}
