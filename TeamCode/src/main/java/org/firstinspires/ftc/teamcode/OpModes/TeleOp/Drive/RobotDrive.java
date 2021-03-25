package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Libs.MechanumDriveClass;
import org.firstinspires.ftc.teamcode.Libs.MovementClass;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterClass;
import org.firstinspires.ftc.teamcode.Libs.TwoPartIntakeClass;
import org.firstinspires.ftc.teamcode.Libs.ArmClass;

import java.io.File;

@TeleOp (name = "Robot Drive 2.0", group = "Drive")
public class RobotDrive extends LinearOpMode {
    MorganConstants robot;

    DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;

    DcMotor mainIntake = null;
    CRServo hopperIntake = null;

    DcMotor flywheel = null;
    Servo ringPusher = null;

    DcMotor arm = null;
    CRServo finger = null;
    DigitalChannel extendedLimit = null, retractedLimit = null;

    BNO055IMU imu;

    MechanumDriveClass drive;
    ArmClass wobbleGoal;

    @Override
    public void runOpMode() {
        robot = new MorganConstants(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, robot.LEFT_FRONT);
        rightFront = hardwareMap.get(DcMotor.class, robot.RIGHT_FRONT);
        leftBack = hardwareMap.get(DcMotor.class, robot.LEFT_BACK);
        rightBack = hardwareMap.get(DcMotor.class, robot.RIGHT_BACK);
        if(robot.LEFT_FRONT_REVERSED) {
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(robot.RIGHT_FRONT_REVERSED) {
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(robot.LEFT_BACK_REVERSED) {
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(robot.RIGHT_BACK_REVERSED) {
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        arm = hardwareMap.get(DcMotor.class, robot.WOBBLE_GOAL_ARM);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        finger = hardwareMap.get(CRServo.class, robot.WOBBLE_GOAL_FINGER);
        extendedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_EXTENDED_LIMIT_SWITCH);
        retractedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_RETRACTED_LIMIT_SWITCH);

        imu = hardwareMap.get(BNO055IMU.class, robot.IMU);

        drive = new MechanumDriveClass(leftFront, rightFront, leftBack, rightBack, imu);
        wobbleGoal = new ArmClass(arm, finger, retractedLimit, extendedLimit);

        waitForStart();

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                drive.mechanumDriveControl(gamepad1);
                wobbleGoal.armDriveControl(gamepad2, 0, false, false);
            }
        }
    }
}