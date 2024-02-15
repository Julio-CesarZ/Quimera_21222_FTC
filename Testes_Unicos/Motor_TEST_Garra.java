package org.firstinspires.ftc.teamcode.Testes_Unicos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Motor_TEST_Garra")
public class Motor_TEST_Garra extends LinearOpMode {


    private DcMotor garra;
    private Servo s0;
    private Servo sAzul;
    private boolean buttonR;

    @Override
    public void runOpMode() {

        garra = hardwareMap.get(DcMotor.class, "garra");
        s0 = hardwareMap.get(Servo.class, "s0");
        sAzul = hardwareMap.get(Servo.class, "sAzul");
        garra.setDirection(DcMotor.Direction.REVERSE);
        garra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_bumper && !buttonR) {
                buttonR = true;
                sleep(200);
            }
            else if(buttonR) {
                garra.setPower(gamepad1.right_stick_y);
            }
            else if(gamepad1.right_stick_y == 0) {
                garra.setPower(0);
            }
            else if(gamepad1.a && buttonR) {
                s0.setPosition(0.5);
                sleep(1000);
                s0.setPosition(0);
            }
            else if(gamepad1.left_bumper && buttonR) {
                buttonR = false;
                sleep(200);
            }
            else if(gamepad1.right_trigger == 0.5 && buttonR) {
                sAzul.setPosition(0.5);
            }

            telemetry.addData("Garra: ", buttonR);
            telemetry.addLine("Para ativar a garra, aperte o LB");
            telemetry.addLine("Para usar o Hex, aperte Y");
            telemetry.addLine("Para usar o Servo braço, aperte A");
            telemetry.addLine("Para usar o Servo garra azul, aperte RT");
            telemetry.update();

        }
    }
}
