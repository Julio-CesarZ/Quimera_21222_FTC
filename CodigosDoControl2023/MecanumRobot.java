package org.firstinspires.ftc.teamcode.CodigosDoControl2023;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "MecanumRobot")
public class MecanumRobot extends LinearOpMode {

    // Define your hardware components
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    Servo s0;
    Servo s1;
    Servo s5;
    DcMotor motor_garra;

    // Define IMU and PID constants
    BNO055IMU imu;
    double kP = 0.1;
    double kI = 0.01;
    double kD = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your hardware components

        // Initialize IMU
        initializeIMU();

        waitForStart();

        // Autonomous code
        while (opModeIsActive()) {
            // Your existing code here

            // Example of using IMU and PID control for rotation
            double targetAngle = 90; // Example target angle (rotate 90 degrees)
            double currentAngle = getYawAngle(); // Get current yaw angle from IMU

            // Calculate PID output
            double correction = calculatePID(targetAngle, currentAngle);

            // Apply correction to robot movement
            rotateRobot(correction);

            // Your existing code here
        }
    }

    // Initialize IMU
    public void initializeIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    // Get current yaw angle from IMU
    public double getYawAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
    }

    // Calculate PID output for rotation
    public double calculatePID(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        // Implement PID calculation here
        return kP * error;
    }

    // Rotate the robot using PID-controlled motor power
    public void rotateRobot(double power) {
        // Adjust motor powers accordingly
        frontleft.setPower(power);
        frontright.setPower(-power);
        backleft.setPower(power);
        backright.setPower(-power);
    }

    // Your existing methods (esquerda, meio, direita, etc.)
}
