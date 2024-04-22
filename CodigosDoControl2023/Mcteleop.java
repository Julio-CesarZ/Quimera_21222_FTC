package org.firstinspires.ftc.teamcode.CodigosDoControl2023;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mcteleop")
public class Mcteleop extends OpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    private double maxPower = 1.0; // Valor inicial de maxPower
    private boolean leftTriggerPressed = false;
    private boolean rightTriggerPressed = false;

    @Override
    public void init() {
        // Inicialização dos motores aqui (usando o REV Hardware Client)
        motorFrontLeft = hardwareMap.get(DcMotor.class, "m0");
        motorFrontRight = hardwareMap.get(DcMotor.class, "m1");
        motorBackLeft = hardwareMap.get(DcMotor.class, "m2");
        motorBackRight = hardwareMap.get(DcMotor.class, "m3");
    }

    @Override
    public void loop() {
        // Controle de teleop aqui
        double drive = -gamepad1.left_stick_x;
        double strafe = gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Ajusta o maxPower se o gatilho esquerdo for pressionado e não estava pressionado anteriormente
        if (gamepad1.left_trigger > 0.1 && !leftTriggerPressed && maxPower > 0.0) {
            maxPower -= 0.1;
        }
        leftTriggerPressed = gamepad1.left_trigger > 0.1;

        // Ajusta o maxPower se o gatilho direito for pressionado e não estava pressionado anteriormente
        if (gamepad1.right_trigger > 0.1 && !rightTriggerPressed && maxPower < 1.0) {
            maxPower += 0.1;
        }
        rightTriggerPressed = gamepad1.right_trigger > 0.1;

        // Garante que maxPower esteja no intervalo [0, 1]
        maxPower = Math.max(0.0, Math.min(1.0, maxPower));

        // Aplica o maxPower aos motores
        frontLeftPower *= maxPower;
        frontRightPower *= maxPower;
        backLeftPower *= maxPower;
        backRightPower *= maxPower;

        // Configuração da potência dos motores
        motorFrontLeft.setPower(frontLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackLeft.setPower(backLeftPower);
        motorBackRight.setPower(backRightPower);

        telemetry.addData("Max Power: ", maxPower);
        telemetry.update();
    }
}
