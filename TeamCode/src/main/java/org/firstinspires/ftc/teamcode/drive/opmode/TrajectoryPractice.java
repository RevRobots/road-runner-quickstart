package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RoadRunnerTrajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name="Trajectories Practice", group="Trajectories")
public class TrajectoryPractice extends LinearOpMode {

    SampleMecanumDrive driveTrain;
    RoadRunnerTrajectories roadRunner;

    @Override
    public void runOpMode () {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        roadRunner = new RoadRunnerTrajectories(driveTrain);

        waitForStart();

        driveTrain.followTrajectory(roadRunner.splineToPoint(0, 0, 0, 500, -300, 90));

    }

}
