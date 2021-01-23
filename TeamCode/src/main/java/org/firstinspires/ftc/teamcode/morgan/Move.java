package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Move extends LinearOpMode {

    DcMotor leftFront, rightFront, leftRear, rightRear;
    DcMotor leftOdometer, rightOdometer, frontOdometer;

    RevBlinkinLedDriver led;

    Localization localization;

    int leftOdometerMultiplier;
    int rightOdometerMultiplier;
    int frontOdometerMultiplier;

    double correctedLeftOdometerValue;
    double correctedRightOdometerValue;
    double correctedFrontOdometerValue;

    double previouLeftOdometerValue;
    double previousRightOdometerValue;
    double previousFrontOdometerValue;

    double changeInRobotOreintation;
    double robotOrientation;

    double globalXPosition;
    double globalYPostion;

    @Override
    public void runOpMode () throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftBack");
        rightRear = hardwareMap.get(DcMotor.class, "rightBack");

        leftOdometer = hardwareMap.get(DcMotor.class, "leftBack");
        rightOdometer = hardwareMap.get(DcMotor.class, "rightBack");
        frontOdometer = hardwareMap.get(DcMotor.class, "leftFront");

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        localization = new Localization(hardwareMap, leftOdometer, rightOdometer, frontOdometer, 75);
        Thread positionThread = new Thread(localization);
        positionThread.start();

        waitForStart();

        if (opModeIsActive()) {

            while(opModeIsActive()) {

                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);

                correctedLeftOdometerValue = (leftOdometer.getCurrentPosition() * leftOdometerMultiplier);
                correctedRightOdometerValue = (rightOdometer.getCurrentPosition() * rightOdometerMultiplier);

                double leftChange = correctedLeftOdometerValue - previouLeftOdometerValue;
                double rightChange = correctedRightOdometerValue = previousRightOdometerValue;

                changeInRobotOreintation = (leftChange - rightChange) / 15;
                robotOrientation = robotOrientation + changeInRobotOreintation;

                correctedFrontOdometerValue = (frontOdometer.getCurrentPosition() * frontOdometerMultiplier);
                double rawFrontChange = correctedFrontOdometerValue - previousFrontOdometerValue;
                double frontChange = rawFrontChange - (changeInRobotOreintation * 2);

                double p = ((rightChange + leftChange) / 2);
                double n = frontChange;

                globalXPosition = globalXPosition + (p * Math.sin(robotOrientation)) + (n * Math.cos(robotOrientation));
                globalYPostion = globalYPostion + (p * Math.cos(robotOrientation)) + (n * Math.sin(robotOrientation));

                previouLeftOdometerValue = correctedLeftOdometerValue;
                previousRightOdometerValue = correctedRightOdometerValue;
                previousFrontOdometerValue = correctedFrontOdometerValue;

                telemetry.addData("X Pos", globalXPosition);
                telemetry.addData("Y Pos", globalYPostion);
                telemetry.addData("Angle", robotOrientation);

                telemetry.addData("Left Odo", correctedLeftOdometerValue);
                telemetry.addData("Right Odo", correctedRightOdometerValue);
                telemetry.addData("Front Odo", correctedFrontOdometerValue);

                telemetry.addData("Thread Active", positionThread.isAlive());

                telemetry.update();

            }

            localization.stop();

        }

    }

}
