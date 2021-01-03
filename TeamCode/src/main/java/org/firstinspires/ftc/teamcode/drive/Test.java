package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Testing", group = "tests")
public class Test extends LinearOpMode {

    SampleMecanumDrive driveTrain;
    RoadRunnerTrajectories rrTrajectories;

    @Override
    public void runOpMode () {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        rrTrajectories = new RoadRunnerTrajectories(driveTrain);

        waitForStart();

        driveTrain.followTrajectory(rrTrajectories.splineToPoint(0, 0, 0, 10, 25, 90));

    }

}
