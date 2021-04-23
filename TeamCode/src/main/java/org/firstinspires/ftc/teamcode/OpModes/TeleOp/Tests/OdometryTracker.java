package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp (name = "Odometry Tracker", group = "Test")
public class OdometryTracker extends LinearOpMode {
    DcMotor leftOdometry;
    DcMotor rightOdometry;
    DcMotor middleOdometry;

    @Override
    public void runOpMode() {
        MorganConstants robot = new MorganConstants(hardwareMap);

        leftOdometry = hardwareMap.get(DcMotor.class, robot.LEFT_ODOMETRY_WHEEL);
        rightOdometry = hardwareMap.get(DcMotor.class, robot.RIGHT_ODOMETRY_WHEEL);
        middleOdometry = hardwareMap.get(DcMotor.class, robot.MIDDLE_ODOMETRY_WHEEL);

        waitForStart();

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                telemetry.addData("Left Odometry", leftOdometry.getCurrentPosition());
                telemetry.addData("Right Odometry", rightOdometry.getCurrentPosition());
                telemetry.addData("Middle Odometry", middleOdometry.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}