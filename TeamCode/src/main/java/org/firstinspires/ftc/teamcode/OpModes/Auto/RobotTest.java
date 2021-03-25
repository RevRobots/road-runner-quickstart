package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Libs.ActuatorTestClass;

@Autonomous (name="Robot Test", group="Testing")
public class RobotTest extends LinearOpMode {

    /**
     * The variables for the values we want to output
     */
    public DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;

    public DcMotor mainIntake = null;
    public CRServo hopperIntake = null;

    public DcMotor frontFlywheel = null, backFlywheel = null;
    public Servo ringPusher = null;

    public DcMotor wobbleGoalArm = null;
    public CRServo wobbleGoalFinger = null;

    public BNO055IMU imu = null;

    /**
     * The classes we want to use during these program
     */
    MorganConstants robot;
    MorganConstants robotConstants;
    Orientation angles;

    ActuatorTestClass testRobot;

    /**
     * The program runs through here when you press init on the DS
     */
    @Override
    public void runOpMode() {
        /**
         * initializes the robot hardware
         */
        robot = new MorganConstants(hardwareMap);

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

        wobbleGoalArm = hardwareMap.get(DcMotor.class, robot.WOBBLE_GOAL_ARM);
        wobbleGoalFinger = hardwareMap.get(CRServo.class, robot.WOBBLE_GOAL_FINGER);

        imu = hardwareMap.get(BNO055IMU.class, robot.IMU);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        testRobot = new ActuatorTestClass();

        telemetry.addData("Test Is Ready To Run | ", "Gamepad 1 Required");
        telemetry.update();

        /**
         * waits for the user to press the play button on the DS
         */
        waitForStart();

        /**
         * when the program is running it goes through this
         */
        if(opModeIsActive()) {
            telemetry.addData("Running: ", "Front Left");
            telemetry.update();

            testRobot.motorEncoderTest(leftFront, 0.25, 2000);

            telemetry.addData("Running: ", "Front Right");
            telemetry.update();

            testRobot.motorEncoderTest(rightFront, 0.25, 2000);

            telemetry.addData("Running: ", "Back Left");
            telemetry.update();

            testRobot.motorEncoderTest(leftBack, 0.25, 2000);

            telemetry.addData("Running: ", "Back Right");
            telemetry.update();

            testRobot.motorEncoderTest(rightBack, 0.25, 2000);

            telemetry.addData("Continue To Intake?", "Press A");
            telemetry.update();

            while(!gamepad1.a);

            telemetry.addData("Running: ", "Main Intake");
            telemetry.update();

            //testRobot.motorEncoderTest(mainIntake, 0.5, 2000);

            telemetry.addData("Running: ", "Hopper Intake");
            telemetry.update();

            testRobot.continuousRotationServoTest(hopperIntake, 0.5, 2000, false);

            telemetry.addData("Continue To Shooter?", "Press B");
            telemetry.update();

            while(!gamepad1.b);

            telemetry.addData("Running: ", "Front Flywheel");
            telemetry.update();

            testRobot.motorEncoderTest(frontFlywheel, 0.125, 2000);

            telemetry.addData("Running: ", "Back Flywheel");
            telemetry.update();

            testRobot.motorEncoderTest(backFlywheel, 0.125, 2000);

            telemetry.addData("Running: ", "Ring Pusher");
            telemetry.update();

            testRobot.servoTest(ringPusher, 0, 0.5, 1000);

            telemetry.addData("Continue To Wobble Goal Arm?", "Press X");
            telemetry.update();

            while(!gamepad1.x);

            telemetry.addData("Running: ", "Wobble Goal Arm");
            telemetry.update();

            testRobot.motorEncoderTest(wobbleGoalArm, 0.25, 500);

            telemetry.addData("Running: ", "Finger");
            telemetry.update();

            testRobot.continuousRotationServoTest(wobbleGoalFinger, 0.25, 1000, false);

            telemetry.addData("Test", "Finished!");
            telemetry.update();
        }   //end of while(...)
    }   //end of if(...)
}   //end of RobotTest
