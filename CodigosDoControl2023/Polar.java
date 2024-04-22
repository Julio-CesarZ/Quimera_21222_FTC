package org.firstinspires.ftc.teamcode.CodigosDoControl2023;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp (name = "Polar")

public class Polar extends LinearOpMode {
    private DcMotor c1;
    private DcMotor c2;

    @Override
    public void runOpMode() {
        float like;
        float dislike;

        c1 = hardwareMap.get(DcMotor.class, "c1");
        c2 = hardwareMap.get(DcMotor.class, "c2");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()){
                like = gamepad1.right_stick_y;
                dislike = gamepad1.left_stick_y;

                c1.setPower(like);
                c2.setPower(dislike);

                telemetry.update();
            }
        }
    }
}