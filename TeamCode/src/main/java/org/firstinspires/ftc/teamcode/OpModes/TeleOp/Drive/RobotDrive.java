package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Hardware.PoseSettings;
import org.firstinspires.ftc.teamcode.Libs.MechanumDriveClass;
import org.firstinspires.ftc.teamcode.Libs.MovementClass;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterControlThread;
import org.firstinspires.ftc.teamcode.Libs.ToggleClass;
import org.firstinspires.ftc.teamcode.Libs.TwoPartIntakeClass;
import org.firstinspires.ftc.teamcode.Libs.ArmClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;

@TeleOp (name = "Robot Drive 2.0", group = "Drive")
public class RobotDrive extends LinearOpMode {


    SampleMecanumDrive localization;
    MorganConstants robot;
    PoseSettings poses;

    DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;

    DcMotor mainIntake = null;
    CRServo hopperIntake = null;

    DcMotorEx flywheel = null;
    Servo ringPusher = null;

    DcMotor arm = null;
    CRServo finger = null;
    DigitalChannel extendedLimit = null, retractedLimit = null;

    BNO055IMU imu;

    MechanumDriveClass drive;
    TwoPartIntakeClass intake;
    ShooterControlThread shooterControl;
    Thread shooterThread;
    RingPusherClass ringPusherClass;
    ArmClass wobbleGoal;
    ToggleClass shooterRPMToggle;

    private int currentShooterRPM = 0;
    private int currentRPMSetting = 0;
    private boolean currentShooterSetting = false; //is high goal RPM

    @Override
    public void runOpMode() {
        //localization = new SampleMecanumDrive(hardwareMap);
        robot = new MorganConstants(hardwareMap);
        poses = new PoseSettings();

        //localization.setPoseEstimate(poses.autoEndPosition);

        telemetry.addData("Localization: ", "Complete");
        telemetry.update();

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

        mainIntake = hardwareMap.get(DcMotor.class, robot.MAIN_INTAKE);
        hopperIntake = hardwareMap.get(CRServo.class, robot.HOPPER_INTAKE);
        if(robot.MAIN_INTAKE_REVERSED) {
            mainIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(robot.HOPPER_INTAKE_REVERSED) {
            hopperIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        flywheel = hardwareMap.get(DcMotorEx.class, robot.FRONT_FLYWHEEL);
        ringPusher = hardwareMap.get(Servo.class, robot.RING_PUSHER);
        if(robot.FLYWHEELS_REVERSED) {
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        arm = hardwareMap.get(DcMotor.class, robot.WOBBLE_GOAL_ARM);
        if(robot.WOBBLE_GOAL_ARM_REVERSED) {
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        finger = hardwareMap.get(CRServo.class, robot.WOBBLE_GOAL_FINGER);
        extendedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_EXTENDED_LIMIT_SWITCH);
        retractedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_RETRACTED_LIMIT_SWITCH);

        imu = hardwareMap.get(BNO055IMU.class, robot.IMU);

        drive = new MechanumDriveClass(leftFront, rightFront, leftBack, rightBack, imu);
        intake =  new TwoPartIntakeClass(mainIntake, hopperIntake);
        shooterControl = new ShooterControlThread(flywheel);
        shooterThread = new Thread(shooterControl);
        ringPusherClass = new RingPusherClass(ringPusher);
        wobbleGoal = new ArmClass(arm, finger, retractedLimit, extendedLimit);
        shooterRPMToggle = new ToggleClass();

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();

        shooterThread.start();
        currentShooterRPM = robot.IDLE_SHOOTER_RPM;

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                drive.mechanumDriveControl(gamepad1);
                intake.intakeDriveControl(gamepad2);
                if(gamepad1.left_trigger != 0) {
                    currentShooterRPM = currentRPMSetting;
                } else {
                    currentShooterRPM = robot.IDLE_SHOOTER_RPM;
                }
                shooterControl.setTargetShooterRPM(currentShooterRPM);
                ringPusherClass.ringPusherControl(gamepad1);
                wobbleGoal.armDriveControl(gamepad2, 0, true);

                currentShooterSetting = shooterRPMToggle.buttonReleaseToggle(gamepad1.dpad_left, false);

                if(currentShooterSetting == false) {
                    telemetry.addData("Current Shooter RPM: ", "High Goal");
                    currentRPMSetting = robot.HIGH_GOAL_SHOOTER_RPM;
                } else if (currentShooterSetting == true) {
                    telemetry.addData("Current Shooter RPM: ", "Power Shot");
                    currentRPMSetting = robot.POWER_SHOT_SHOOTER_RPM;
                }

                telemetry.update();
            }

            shooterControl.setTargetShooterRPM(0);
            shooterControl.stopShooterThread();
        }
    }
}