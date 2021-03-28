package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Libs.MechanumDriveClass;
import org.firstinspires.ftc.teamcode.Libs.MovementClass;
import org.firstinspires.ftc.teamcode.Libs.OnePartIntakeClass;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterControlThread;
import org.firstinspires.ftc.teamcode.Libs.ArmClass;

@TeleOp (name = "Robot Drive", group = "Drive")
@Disabled
public class RobotDriveOld extends LinearOpMode {
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
    MovementClass movement;

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
        wobbleGoal = new ArmClass(wobbleArm, finger, null, null);
        movement = new MovementClass();

        waitForStart();

        shooterThread.start();

        if(opModeIsActive()) {
            while (opModeIsActive()) {
                drive.mechanumDriveControl(gamepad1);
                intake.intakeDriveControl(gamepad1);
                ringPusherClass.ringPusherControl(gamepad2);
                wobbleGoal.armDriveControl(gamepad2, 0, false);

                if(gamepad2.left_trigger != 0) {
                    shooter.setTargetShooterRPM(robot.HIGH_GOAL_SHOOTER_RPM);
                } else {
                    shooter.setTargetShooterRPM(robot.IDLE_SHOOTER_RPM);
                }

                telemetry.addData("Movement", movement.movement(gamepad2));
            }
            shooter.setTargetShooterRPM(0);
            shooter.stopShooterThread();
        }
    }
}
