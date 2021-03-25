package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Libs.MechanumDriveClass;
import org.firstinspires.ftc.teamcode.Libs.OnePartIntakeClass;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterControlThread;
import org.firstinspires.ftc.teamcode.Libs.ArmClass;

@Autonomous (name = "Autonomous Testing", group = "Test")
public class AutoTest extends LinearOpMode {
    MorganConstants robot;

    private DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;

    private DcMotor mainIntake = null;

    private DcMotor flywheel = null;
    private Servo ringPusher = null;

    private DcMotor wobbleArm = null;
    private CRServo finger = null;

    private BNO055IMU imu = null;

    MechanumDriveClass drive;
    OnePartIntakeClass intake;
    ShooterControlThread shooter;
    Thread shooterThread;
    RingPusherClass ringPusherClass;
    ArmClass wobbleGoal;

    @Override
    public void runOpMode() {
        robot = new MorganConstants(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, robot.LEFT_FRONT);
        rightFront = hardwareMap.get(DcMotor.class, robot.RIGHT_FRONT);
        leftBack = hardwareMap.get(DcMotor.class, robot.LEFT_BACK);
        rightBack = hardwareMap.get(DcMotor.class, robot.RIGHT_BACK);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        mainIntake = hardwareMap.get(DcMotor.class, robot.MAIN_INTAKE);

        flywheel = hardwareMap.get(DcMotor.class, robot.FRONT_FLYWHEEL);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        ringPusher = hardwareMap.get(Servo.class, robot.RING_PUSHER);

        wobbleArm = hardwareMap.get(DcMotor.class, robot.WOBBLE_GOAL_ARM);
        finger = hardwareMap.get(CRServo.class, robot.WOBBLE_GOAL_FINGER);

        imu = hardwareMap.get(BNO055IMU.class, robot.IMU);

        drive = new MechanumDriveClass(leftFront, rightFront, leftBack, rightBack, imu);
        intake = new OnePartIntakeClass(mainIntake);
        shooter = new ShooterControlThread(flywheel);
        shooterThread = new Thread(shooter);
        ringPusherClass = new RingPusherClass(ringPusher);
        wobbleGoal =new ArmClass(wobbleArm, finger, null, null);

        waitForStart();

        drive.driveForwardsRightEncoder(25, 2000);

        drive.driveForwardsPIDEncoder(25, 2000);
    }
}
