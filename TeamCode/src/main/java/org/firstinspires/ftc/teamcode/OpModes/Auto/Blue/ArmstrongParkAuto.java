package org.firstinspires.ftc.teamcode.OpModes.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.ArmstrongConstants;
import org.firstinspires.ftc.teamcode.Libs.MechanumDriveClass;

@Disabled
@Autonomous (name = "Armstrong Park Auto", group = "Blue")
public class ArmstrongParkAuto extends LinearOpMode {
    ArmstrongConstants robot;

    DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;

    MechanumDriveClass drive;

    @Override
    public void runOpMode() {
        robot = new ArmstrongConstants();

        leftFront = hardwareMap.get(DcMotor.class, robot.LEFT_FRONT);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront = hardwareMap.get(DcMotor.class, robot.RIGHT_FRONT);
        leftBack = hardwareMap.get(DcMotor.class, robot.LEFT_BACK);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack = hardwareMap.get(DcMotor.class, robot.RIGHT_BACK);

        drive = new MechanumDriveClass(leftFront, rightFront, leftBack, rightBack, null);

        waitForStart();

        drive.driveForwardsEnocoder(0.5, 2000);
    }
}
