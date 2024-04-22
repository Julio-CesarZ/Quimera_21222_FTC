package org.firstinspires.ftc.teamcode.CodigosDoControl2023;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Melissaa")
public class Melissaa extends LinearOpMode {

    private DcMotor m0;
    private DcMotor m1;

    @Override
    public void runOpMode() {
        m0 = hardwareMap.get(DcMotor.class, "m0");
        m1 = hardwareMap.get(DcMotor.class, "m1");

        // Definindo o modo de operação dos motores
        m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            float math;
            float sheila;
            float power = 0.0f;
            final float MAX_POWER = 0.5f;
            final float ACCELERATION = 0.02f;
            final float BRAKE_POWER = 0.02f;

            while (opModeIsActive()) {
                math = gamepad1.right_stick_y;
                sheila = gamepad1.left_stick_y;

                // Aplicar aceleração/desaceleração
                if (gamepad1.right_stick_y != 0 || gamepad1.left_stick_y != 0) {
                    if (power < MAX_POWER) {
                        power += ACCELERATION;
                    }
                } else {
                    // Desacelerar quando não há entrada
                    if (power > BRAKE_POWER) {
                        power -= BRAKE_POWER; // Aplicar frenagem suave
                    } else {
                        power = 0.0f; // Garantir que a potência seja zero quando totalmente desacelerada
                    }
                }

                m0.setPower(sheila * power);
                m1.setPower(math * power);

                telemetry.addData("Power", power);
                telemetry.update();
            }
        }
    }
}
