package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RoadRunnerTrajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "RoadRunner Testing", group = "Red")
public class RR extends LinearOpMode {
    SampleMecanumDrive driveTrain;
    RoadRunnerTrajectories rrTraj;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new SampleMecanumDrive(hardwareMap);
        driveTrain.setPoseEstimate(new Pose2d(-63, -18, Math.toRadians(0)));
        waitForStart();
        /*Trajectory ringDetection = driveTrain.trajectoryBuilder(new Pose2d(-63, -18, 0))
                .splineToSplineHeading(new Pose2d(-36, -18, Math.toRadians(-45)), 0)
                .build();
        driveTrain.followTrajectory(ringDetection);
        Thread.sleep(1000);
        Trajectory powerShotOne = driveTrain.trajectoryBuilder(new Pose2d(-36, -18, Math.toRadians(-45)))
                .splineToSplineHeading(new Pose2d(0, -15, Math.toRadians(0)), 30)
                .build();
        driveTrain.followTrajectory(powerShotOne);
        Thread.sleep(1000);
        Trajectory powerShotTwo = driveTrain.trajectoryBuilder(new Pose2d(0, -15, 0))
                .splineToConstantHeading(new Vector2d(0, -5), 90)
                .build();
        driveTrain.followTrajectory(powerShotTwo);
        Thread.sleep(1000);
        Trajectory powerShotThree = driveTrain.trajectoryBuilder(new Pose2d(0, -5, 0))
                .splineToConstantHeading(new Vector2d(0, 5), 0)
                .build();
        driveTrain.followTrajectory(powerShotThree);
        Thread.sleep(1000);
        Trajectory wobbleGoalDeposit = driveTrain.trajectoryBuilder(new Pose2d(0, 5, 0))
                .splineToLinearHeading(new Pose2d(54, -60, 0), 0)
                .build();
        driveTrain.followTrajectory(wobbleGoalDeposit);
        Trajectory parkPartOne = driveTrain.trajectoryBuilder(new Pose2d(54, -60, 0))
                .splineToLinearHeading(new Pose2d(30, -60, 0), 180)
                .build();
        driveTrain.followTrajectory(parkPartOne);
        Trajectory parkPartTwo = driveTrain.trajectoryBuilder(new Pose2d(30, -60, 0))
                .splineToLinearHeading(new Pose2d(11, -12, 0), 180)
                .build();
        driveTrain.followTrajectory(parkPartTwo);*/
    }
}
