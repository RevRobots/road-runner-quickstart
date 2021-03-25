package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class RoadRunnerTrajectories {
    SampleMecanumDrive driveTrain;

    public RoadRunnerTrajectories(SampleMecanumDrive dT) {
        this.driveTrain = dT;
    }

    public Trajectory forwardsByInch(double currentX, double currentY, double heading, double inches) {
        Trajectory trajectory = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, heading))
                .forward(inches)
                .build();

        return trajectory;
    }

    public Trajectory backwardsByInch(double currentX, double currentY, double heading, double inches) {
        Trajectory trajectory = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, heading))
                .back(inches)
                .build();

        return trajectory;
    }

    public Trajectory strafeRightByInch(double currentX, double currentY, double heading, double inches) {
        Trajectory trajectory = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, heading))
                .strafeRight(inches)
                .build();

        return trajectory;
    }

    public Trajectory strafeLeftByInch(double currentX, double currentY, double heading, double inches) {
        Trajectory trajectory = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, heading))
                .strafeLeft(inches)
                .build();

        return trajectory;
    }

    public Trajectory lineToPoint(double currentX, double currentY, double heading, double goalX, double goalY) {
        Trajectory trajectory = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, heading))
                .lineTo(new Vector2d(goalX, goalY))
                .build();

        return trajectory;
    }

    public Trajectory strafeToPoint(double currentX, double currentY, double heading, double goalX, double goalY) {
        Trajectory trajectory = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, heading))
                .strafeTo(new Vector2d(goalX, goalY))
                .build();

        return trajectory;
    }

    public Trajectory splineToPoint(double currentX, double currentY, double currentHeading, double goalX, double goalY, double goalHeading) {
        Trajectory trajectory = driveTrain.trajectoryBuilder(new Pose2d(currentX, currentY, currentHeading))
                .splineTo(new Vector2d(goalX, goalY), goalHeading)
                .build();

        return trajectory;
    }
}
