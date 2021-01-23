package org.firstinspires.ftc.teamcode.morgan.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.morgan.DriveTrain;
import org.firstinspires.ftc.teamcode.morgan.Intake;
import org.firstinspires.ftc.teamcode.morgan.RobotConfig;
import org.firstinspires.ftc.teamcode.morgan.Shooter;
import org.firstinspires.ftc.teamcode.morgan.WobbleArm;

//@Disabled
@Autonomous (name = "Odometery Autonomous", group = "Red")
public class OdoAuto extends LinearOpMode {

    RobotConfig robotConfig;

    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotor leftOdometer, rightOdometer, frontOdometer;

    DcMotor intake;

    DcMotor rotation;
    DcMotor flywheel;
    Servo ringPusher;
    DistanceSensor distanceSensor;

    DcMotor wobbleGoal;
    CRServo finger;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    DriveTrain drive;
    Intake intakeClass;
    Shooter shooter;
    WobbleArm wobbleArm;

    ElapsedTime timer;

    @Override
    public void runOpMode() {

        robotConfig = new RobotConfig(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, robotConfig.leftFront);
        rightFront = hardwareMap.get(DcMotor.class, robotConfig.rightFront);
        leftBack = hardwareMap.get(DcMotor.class, robotConfig.leftBack);
        rightBack = hardwareMap.get(DcMotor.class, robotConfig.rightBack);

        leftOdometer = hardwareMap.get(DcMotor.class, robotConfig.leftOdometer);
        rightOdometer = hardwareMap.get(DcMotor.class, robotConfig.rightOdometer);
        frontOdometer = hardwareMap.get(DcMotor.class, robotConfig.frontOdometer);

        intake = hardwareMap.get(DcMotor.class, robotConfig.intake);

        rotation = hardwareMap.get(DcMotor.class, robotConfig.rotation);
        flywheel = hardwareMap.get(DcMotor.class, robotConfig.flywheel);
        ringPusher = hardwareMap.get(Servo.class, robotConfig.ringPusher);

        wobbleGoal = hardwareMap.get(DcMotor.class, robotConfig.wobbleArm);
        finger = hardwareMap.get(CRServo.class, robotConfig.finger);

        imu = hardwareMap.get(BNO055IMU.class, robotConfig.imu);

        drive = new DriveTrain(leftFront, rightFront, leftBack, rightBack, leftOdometer, rightOdometer, frontOdometer);
        intakeClass = new Intake(intake);
        shooter = new Shooter(rotation, flywheel, ringPusher);
        wobbleArm = new WobbleArm(wobbleGoal, finger);

        timer = new ElapsedTime();

        waitForStart();

        int breakTime = 2000;

        drive.forwardsOdometry(0.5, 1000);

        timer.reset();
        while(timer.milliseconds() < breakTime);

        drive.backwardsOdometery(0.5, 1000);

        timer.reset();
        while(timer.milliseconds() < breakTime);

        drive.rightOdometery(0.5, 1000);

        timer.reset();
        while(timer.milliseconds() < breakTime);

        drive.leftOdometery(0.5, 1000);

        timer.reset();
        while(timer.milliseconds() < breakTime);

        drive.turnRightOdometery(0.5, 1000);

        timer.reset();
        while(timer.milliseconds() < breakTime);

        drive.turnLeftOdometery(0.5, 1000);

    }

}