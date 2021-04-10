package org.firstinspires.ftc.teamcode.OpModes.Auto.Blue;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Libs.ArmClass;
import org.firstinspires.ftc.teamcode.Libs.MechanumDriveClass;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterClass;
import org.firstinspires.ftc.teamcode.Libs.TwoPartIntakeClass;

@Disabled
@Autonomous (name = "Blue Right Line", group = "Blue")
public class BlueRightLine extends LinearOpMode {
    MorganConstants robot;

    private DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;

    private DcMotor mainIntake = null;
    private CRServo hopperIntake = null;

    private DcMotorEx frontFlywheel = null, backFlywheel = null;
    private Servo ringPusher = null;

    private DcMotor wobbleArm = null;
    private CRServo finger = null;

    private BNO055IMU imu = null;

    MechanumDriveClass drive;
    TwoPartIntakeClass intake;
    /*DualShooterControlThread shooterControl;
    Thread shooterThread;*/
    ShooterClass shooter;
    RingPusherClass ringPusherClass;
    ArmClass wobbleGoal;

    ElapsedTime timer;

    @Override
    public void runOpMode() {
        robot = new MorganConstants(hardwareMap);

        //robotConfig.initializeRobot();

        leftFront = hardwareMap.get(DcMotor.class, robot.LEFT_FRONT);
        rightFront = hardwareMap.get(DcMotor.class, robot.RIGHT_FRONT);
        leftBack = hardwareMap.get(DcMotor.class, robot.LEFT_BACK);
        rightBack = hardwareMap.get(DcMotor.class, robot.RIGHT_BACK);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        mainIntake = hardwareMap.get(DcMotor.class, robot.MAIN_INTAKE);
        hopperIntake = hardwareMap.get(CRServo.class, robot.HOPPER_INTAKE);

        mainIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontFlywheel = hardwareMap.get(DcMotorEx.class, robot.FRONT_FLYWHEEL);
        backFlywheel = hardwareMap.get(DcMotorEx.class, robot.BACK_FLYWHEEL);
        ringPusher = hardwareMap.get(Servo.class, robot.RING_PUSHER);

        wobbleArm = hardwareMap.get(DcMotor.class, robot.WOBBLE_GOAL_ARM);
        finger = hardwareMap.get(CRServo.class, robot.WOBBLE_GOAL_FINGER);

        imu = hardwareMap.get(BNO055IMU.class, robot.IMU);

        drive = new MechanumDriveClass(leftFront, rightFront, leftBack, rightBack, imu);
        intake = new TwoPartIntakeClass(mainIntake, hopperIntake);
        /*shooterControl = new DualShooterControlThread(frontFlywheel, backFlywheel);
        shooterThread = new Thread(shooterControl);*/
        shooter = new ShooterClass(robot.RUN_DUAL_SHOOTER, frontFlywheel, backFlywheel);
        ringPusherClass = new RingPusherClass(ringPusher);
        wobbleGoal = new ArmClass(wobbleArm, finger, null, null);
        timer = new ElapsedTime();

        waitForStart();

        drive.driveRightEncoder(25, 750);

        drive.driveForwardsEnocoder(25, 3000);

        drive.driveRightEncoder(25, 500);
    }
}
