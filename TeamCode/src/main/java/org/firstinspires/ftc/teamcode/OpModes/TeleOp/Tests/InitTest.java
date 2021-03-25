package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

@TeleOp (name = "Initialization Test", group = "Test")
@Disabled
public class InitTest extends LinearOpMode {
    MorganConstants robot;

    DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;
    DcMotor mainIntake = null;
    CRServo hopperIntake = null;
    DcMotorEx frontFlywheel = null, backFlywheel = null;
    Servo ringPusher = null;
    DcMotor wobbleGoalArm = null;
    CRServo wobbleGoalFinger = null;

    @Override
    public void runOpMode() {
        robot = new MorganConstants(hardwareMap);
        robot.initializeRobot(
                leftFront, rightFront, leftBack, rightBack, mainIntake, hopperIntake,
                frontFlywheel, backFlywheel, ringPusher, wobbleGoalArm, wobbleGoalFinger
        );

        telemetry.addData("Robot: ", "READY");
        telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            while(opModeIsActive()) {

            }
        }
    }
}
