package org.firstinspires.ftc.teamcode.Autonomous_Mode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomus_TEST_1")

public class Autonomous_TEST_1 extends LinearOpMode {

    DcMotor m0;
    DcMotor m1;
    DcMotor m2;
    DcMotor m3;
    Servo servo1;
    Servo servo2;
    ColorSensor color1;

    @Override
    public void runOpMode() {
        m0 = hardwareMap.get(DcMotor.class, "m0");
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        color1 = hardwareMap.get(ColorSensor.class, "color1");

        waitForStart();
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);

        Servo(90);

    }

    public void Walk(double x, long z) {
        m0.setPower(x);
        m1.setPower(x);
        m2.setPower(x);
        m3.setPower(x);
        sleep(z);
        m0.setPower(0);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
    }

    public void Spin90L() {
        m0.setPower(0.7);
        m1.setPower(-0.7);
        m2.setPower(-0.7);
        m3.setPower(0.7);

        sleep(1300);

        m0.setPower(0);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
    }

    public void Spin90R() {
        m0.setPower(-0.7);
        m1.setPower(0.7);
        m2.setPower(0.7);
        m3.setPower(-0.7);

        sleep(1300);

        m0.setPower(0);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
    }

    public void Servo(double grausS1) {
        double posicaoServo1 = grausS1 / 180.0;
        double posicaoServo2 = grausS1 / 180.0;

        servo1.setPosition(posicaoServo1);
        servo2.setPosition(posicaoServo2);
    }

}