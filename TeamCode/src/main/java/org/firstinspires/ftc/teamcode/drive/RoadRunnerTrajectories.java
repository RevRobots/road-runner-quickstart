package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class RoadRunnerTrajectories {

    SampleMecanumDrive driveTrain;

    public RoadRunnerTrajectories (SampleMecanumDrive dT) {
        driveTrain = dT;
    }

    public Trajectory forwardsByInch (double x, double y, double heading, double inches) {

        Trajectory forwards = driveTrain.trajectoryBuilder(new Pose2d(x, y, heading))
                .forward(inches)
                .build();

        return forwards;

    }

    public Trajectory backwardsByInch (double x, double y, double heading, double inches) {

        Trajectory backwards = driveTrain.trajectoryBuilder(new Pose2d(x, y, heading))
                .back(inches)
                .build();

        return backwards;

    }

    public Trajectory strafeRightByInch (double x, double y, double heading, double inches) {

        Trajectory strafeRight = driveTrain.trajectoryBuilder(new Pose2d(x, y, heading))
                .strafeRight(inches)
                .build();

        return strafeRight;

    }

    public Trajectory strafeLeftByInch (double x, double y, double heading, double inches) {

        Trajectory strafeLeft = driveTrain.trajectoryBuilder(new Pose2d(x, y, heading))
                .strafeLeft(inches)
                .build();

        return strafeLeft;
    }

    public Trajectory lineToPoint (double currentX, double currentY, double heading, double goalX, double goalY) {

        Trajectory lineTo = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, heading))
                .lineTo(new Vector2d(goalX, goalY))
                .build();

        return  lineTo;

    }

    public Trajectory strafeToPoint (double currentX, double currentY, double heading, double goalX, double goalY) {

        Trajectory strafeTo = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, heading))
                .strafeTo(new Vector2d(goalX, goalY))
                .build();

        return strafeTo;

    }

    public Trajectory splineToPoint (double currentX, double currentY, double currentHeading, double goalX, double goalY, double goalHeading) {

        Trajectory splineTo = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, currentHeading))
                .splineTo(new Vector2d(goalX, goalY), goalHeading)
                .build();

        return splineTo;

    }

}
