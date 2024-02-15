package org.firstinspires.ftc.teamcode.Testes_Unicos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor_TEST_Reduzido")
public class Motor_TEST_Reduzido extends LinearOpMode {


    private DcMotor Motor;
    double maxPower = 1;
    private boolean leftTriggerPressed = false;
    private boolean rightTriggerPressed = false;



    @Override
    public void runOpMode() {

        Motor = hardwareMap.get(DcMotor.class, "Motor");
        Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        float Y;
        float X;
        float Z;

        waitForStart();
        while(opModeIsActive()) {
            Y = gamepad1.left_stick_y;
            X = gamepad1.left_stick_x;
            Z = gamepad1.right_stick_x;

            if (gamepad1.left_trigger > 0.1 && !leftTriggerPressed && maxPower > 0.0) {
                maxPower -= 0.1;
            }
            leftTriggerPressed = gamepad1.left_trigger > 0.1;

            // Ajusta o maxPower se o gatilho direito for pressionado e não estava pressionado anteriormente
            if (gamepad1.right_trigger > 0.1 && !rightTriggerPressed && maxPower < 1.0) {
                maxPower += 0.1;
            }
            rightTriggerPressed = gamepad1.right_trigger > 0.1;

            X *= maxPower;
            Y *= maxPower;
            Z *= maxPower;

            Motor.setPower(X);
            Motor.setPower(Y);
            Motor.setPower(Z);

            telemetry.addData("Power: ", maxPower);
            telemetry.update();

        }
    }
}
