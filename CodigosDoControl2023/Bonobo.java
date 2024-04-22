package org.firstinspires.ftc.teamcode.CodigosDoControl2023;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Bonobo")
public class Bonobo extends LinearOpMode {

    private DcMotor m0;
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;

    private int m0Pos;
    private int m1Pos;
    private int m2Pos;
    private int m3Pos;


    @Override
    public void runOpMode() {

        m0 = hardwareMap.get(DcMotor.class, "frontleft");
        m1 = hardwareMap.get(DcMotor.class, "frontright");
        m2 = hardwareMap.get(DcMotor.class, "backleft");
        m3 = hardwareMap.get(DcMotor.class, "backright");

        m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m0.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        m0Pos = 0;
        m1Pos = 0;
        m2Pos = 0;
        m3Pos = 0;

        waitForStart();

        drive(10000, 0.5);
        sleep(1000);




    }

    private void turn(int m0Ta, int m1Ta, int m2Ta, int m3Ta, double speed, double speed2, double speed3, double speed4) {

        m0Pos += m0Ta;
        m1Pos += m1Ta;
        m2Pos += m2Ta;
        m3Pos += m3Ta;

        m0.setTargetPosition(m0Pos);
        m1.setTargetPosition(m1Pos);
        m2.setTargetPosition(m2Pos);
        m3.setTargetPosition(m3Pos);

        m0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m0.setPower(speed);
        m1.setPower(speed2);
        m2.setPower(speed3);
        m3.setPower(speed4);


        while(opModeIsActive() && m0.isBusy() && m1.isBusy() && m2.isBusy() && m3.isBusy()) {
            idle();
        }

    }

    private void drive(int direction, double speed) {

        m0Pos += direction;
        m1Pos += direction;
        m2Pos += direction;
        m3Pos += direction;

        m0.setTargetPosition(m0Pos);
        m1.setTargetPosition(m1Pos);
        m2.setTargetPosition(m2Pos);
        m3.setTargetPosition(m3Pos);

        m0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m0.setPower(speed);
        m1.setPower(speed);
        m2.setPower(speed);
        m3.setPower(speed);


        while(opModeIsActive() && m0.isBusy() && m1.isBusy() && m2.isBusy() && m3.isBusy()) {
            idle();
        }

    }



}