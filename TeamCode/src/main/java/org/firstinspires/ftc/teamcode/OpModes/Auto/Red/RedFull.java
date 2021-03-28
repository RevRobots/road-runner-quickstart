package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Hardware.PoseSettings;
import org.firstinspires.ftc.teamcode.Libs.ArmClass;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "Red Full", group = "Red")
public class RedFull extends LinearOpMode {
    DcMotorEx flywheel = null;
    Servo ringPusher = null;

    DcMotor arm = null;
    CRServo finger = null;
    DigitalChannel extendedLimit = null, retractedLimit = null;

    @Override
    public void runOpMode() {
        MorganConstants robot = new MorganConstants(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PoseSettings poses = new PoseSettings();

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
        if(robot.WOBBLE_GOAL_FINGER_REVERSED) {
            finger.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        extendedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_EXTENDED_LIMIT_SWITCH);
        retractedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_RETRACTED_LIMIT_SWITCH);

        ShooterClass shooter = new ShooterClass(false, flywheel, null);
        RingPusherClass ringPusherClass = new RingPusherClass(ringPusher);
        ArmClass wobbleGoal = new ArmClass(arm, finger, retractedLimit, extendedLimit);

        drive.setPoseEstimate(poses.redFullStartPoint);

        Trajectory ringDetectionMovement = drive.trajectoryBuilder(poses.redFullStartPoint, Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-24.5, -17.5), Math.toRadians(0))
                .build();

        Trajectory boxCMovements = drive.trajectoryBuilder(new Pose2d(-24, -17.5, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(65, -40, Math.toRadians(-75)), Math.toRadians(-90))
                .build();

        Trajectory wobbleGoalGrab = drive.trajectoryBuilder(new Pose2d(65, -40, Math.toRadians(-75)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-26, -60, Math.toRadians(180)), Math.toRadians(180))
                .build();

        Trajectory depositGoal = drive.trajectoryBuilder(new Pose2d(-26, -60, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(5, -55, Math.toRadians(90)), 0)
                .splineToSplineHeading(new Pose2d(50, -55, Math.toRadians(-5)), 0)
                .build();

        Trajectory powerShotsMovement = drive.trajectoryBuilder(new Pose2d(50, -55, Math.toRadians(0)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(10, -7.5), Math.toRadians(90))
                .build();

        Trajectory secondPowerBoi = drive.trajectoryBuilder(new Pose2d(10, -7.5, Math.toRadians(5)), Math.toRadians(90))
                .strafeLeft(5.5)
                .build();

        Trajectory thirdPowerLad = drive.trajectoryBuilder(new Pose2d(10, -2.5, Math.toRadians(5)), Math.toRadians(90))
                .strafeLeft(5.5)
                .build();

        Trajectory park = drive.trajectoryBuilder(new Pose2d(10, 3, Math.toRadians(5)))
                .forward(10)
                .build();

        waitForStart();

        finger.setPower(1);
        drive.followTrajectory(ringDetectionMovement);
        drive.followTrajectory(boxCMovements);
        while(extendedLimit.getState() == true) {
            arm.setPower(0.5);
        }
        arm.setPower(0);
        finger.setPower(-1);
        sleep(500);
        finger.setPower(0);
        drive.followTrajectory(wobbleGoalGrab);
        finger.setPower(1);
        sleep(1000);
        while(retractedLimit.getState() == true) {
            arm.setPower(-0.5);
        }
        arm.setPower(0);
        drive.followTrajectory(depositGoal);
        while(extendedLimit.getState() == true) {
            arm.setPower(0.5);
        }
        arm.setPower(0);
        finger.setPower(-1);
        sleep(500);
        while(retractedLimit.getState() == true) {
            arm.setPower(-0.5);
        }
        arm.setPower(0);
        shooter.setFlywheelVelocity(robot.HIGH_GOAL_SHOOTER_RPM);
        drive.followTrajectory(powerShotsMovement);
        ringPusherClass.trigger(750);
        drive.followTrajectory(secondPowerBoi);
        ringPusherClass.trigger(750);
        drive.followTrajectory(thirdPowerLad);
        ringPusherClass.trigger(750);
        ringPusherClass.trigger(750);
        drive.followTrajectory(park);
    }
}