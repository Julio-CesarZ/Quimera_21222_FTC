package org.firstinspires.ftc.teamcode.Testes_Unicos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo_TEST")
public class Servo_TEST extends LinearOpMode {

    private Servo s0;
    private boolean buttonR;
    private float lastStickValue;  // Adicionada variável para armazenar o último valor do analógico

    @Override
    public void runOpMode() {

        s0 = hardwareMap.get(Servo.class, "s0");

        waitForStart();
        while (opModeIsActive()) {

            if (buttonR) {
                s0.setPosition(gamepad1.left_stick_y);
                lastStickValue = gamepad1.left_stick_y;  // Atualiza o último valor do analógico
            }

            if (gamepad1.left_bumper && !buttonR) {
                buttonR = true;
                sleep(200);
            } else if (gamepad1.a && buttonR) {
                s0.setPosition(lastStickValue);  // Configura a posição do servo com o último valor do analógico
                buttonR = false;  // Libera o botão R para permitir movimento do analógico novamente
                sleep(200);
            } else if (gamepad1.left_bumper && buttonR) {
                buttonR = false;
                sleep(200);
            }

            telemetry.addData("Servo: ", buttonR);
            telemetry.addLine("Para usar o Servo, aperte A");
            telemetry.update();
        }
    }
}
