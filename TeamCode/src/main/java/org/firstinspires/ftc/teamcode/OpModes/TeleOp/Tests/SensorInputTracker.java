package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

@TeleOp (name="Sensor Input Tracker", group = "Testing")
public class SensorInputTracker extends LinearOpMode {

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

    public DigitalChannel limitSwitch = null;
    public DigitalChannel limitSwitchRetracted = null;

    public BNO055IMU imu = null;

    /**
     * The classes we want to use during these program
     */
    MorganConstants robot;
    Orientation angles;

    /**
     * The program runs through here when you press init on the DS
     */
    @Override
    public void runOpMode() {
        try {
            robot = new MorganConstants(hardwareMap);

            leftFront = hardwareMap.get(DcMotor.class, robot.LEFT_FRONT);
            rightFront = hardwareMap.get(DcMotor.class, robot.RIGHT_FRONT);
            leftBack = hardwareMap.get(DcMotor.class, robot.LEFT_BACK);
            rightBack = hardwareMap.get(DcMotor.class, robot.RIGHT_BACK);

            mainIntake = hardwareMap.get(DcMotor.class, robot.MAIN_INTAKE);
            hopperIntake = hardwareMap.get(CRServo.class, robot.HOPPER_INTAKE);

            frontFlywheel = hardwareMap.get(DcMotor.class, robot.FRONT_FLYWHEEL);
            backFlywheel = hardwareMap.get(DcMotor.class, robot.BACK_FLYWHEEL);
            ringPusher = hardwareMap.get(Servo.class, robot.RING_PUSHER);

            wobbleGoalArm = hardwareMap.get(DcMotor.class, robot.WOBBLE_GOAL_ARM);
            wobbleGoalFinger = hardwareMap.get(CRServo.class, robot.WOBBLE_GOAL_FINGER);

            limitSwitch = hardwareMap.get(DigitalChannel.class, robot.ARM_EXTENDED_LIMIT_SWITCH);
            limitSwitchRetracted = hardwareMap.get(DigitalChannel.class, robot.ARM_RETRACTED_LIMIT_SWITCH);

            imu = hardwareMap.get(BNO055IMU.class, robot.IMU);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);
        } catch(IllegalArgumentException e) {
            telemetry.addData("Error: ", e);
            telemetry.update();
        }   //end of try - catch(...)

        telemetry.addData("Robot: ", "Ready to start");
        telemetry.update();

        /**
         * waits for the user to press the play button on the DS
         */
        waitForStart();

        /**
         * when the program is running it goes through this
         */
        if(opModeIsActive()) {
            /**
             * when the program is running it loops through here
             */
            while(opModeIsActive()) {
                telemetry.addData("Gamepad 1", "Values");
                telemetry.addData("Gamepad 1 | Left Stick Y: ", gamepad1.left_stick_y);
                telemetry.addData("Gamepad 1 | Left Stick X: ", gamepad1.left_stick_x);
                telemetry.addData("Gamepad 1 | Right Stick Y: ", gamepad1.right_stick_y);
                telemetry.addData("Gamepad 1 | Right Stick X: ", gamepad1.right_stick_x);

                telemetry.addData("Gamepad 1 | D-Pad Up: ", gamepad1.dpad_up);
                telemetry.addData("Gamepad 1 | D-Pad Down: ", gamepad1.dpad_down);
                telemetry.addData("Gamepad 1 | D-Pad Left: ", gamepad1.dpad_left);
                telemetry.addData("Gamepad 1 | D-Pad Right: ", gamepad1.dpad_right);

                telemetry.addData("Gamepad 1 | A: ", gamepad1.a);
                telemetry.addData("Gamepad 1 | B: ", gamepad1.b);
                telemetry.addData("Gamepad 1 | X: ", gamepad1.x);
                telemetry.addData("Gamepad 1 | Y: ", gamepad1.y);

                telemetry.addData("Gamepad 1 | Left Bumper: ", gamepad1.left_bumper);
                telemetry.addData("Gamepad 1 | Right Bumper: ", gamepad1.right_bumper);
                telemetry.addData("Gamepad 1 | Left Trigger: ", gamepad1.left_trigger);
                telemetry.addData("Gamepad 1 | Right Trigger: ", gamepad1.right_trigger);


                telemetry.addData("Gamepad 2", "Values");
                telemetry.addData("Gamepad 2 | Left Stick Y: ", gamepad2.left_stick_y);
                telemetry.addData("Gamepad 2 | Left Stick X: ", gamepad2.left_stick_x);
                telemetry.addData("Gamepad 2 | Right Stick Y: ", gamepad2.right_stick_y);
                telemetry.addData("Gamepad 2 | Right Stick X: ", gamepad2.right_stick_x);

                telemetry.addData("Gamepad 2 | D-Pad Up: ", gamepad2.dpad_up);
                telemetry.addData("Gamepad 2 | D-Pad Down: ", gamepad2.dpad_down);
                telemetry.addData("Gamepad 2 | D-Pad Left: ", gamepad2.dpad_left);
                telemetry.addData("Gamepad 2 | D-Pad Right: ", gamepad2.dpad_right);

                telemetry.addData("Gamepad 2 | A: ", gamepad2.a);
                telemetry.addData("Gamepad 2 | B: ", gamepad2.b);
                telemetry.addData("Gamepad 2 | X: ", gamepad2.x);
                telemetry.addData("Gamepad 2 | Y: ", gamepad2.y);

                telemetry.addData("Gamepad 2 | Left Bumper: ", gamepad2.left_bumper);
                telemetry.addData("Gamepad 2 | Right Bumper: ", gamepad2.right_bumper);
                telemetry.addData("Gamepad 2 | Left Trigger: ", gamepad2.left_trigger);
                telemetry.addData("Gamepad 2 | Right Trigger: ", gamepad2.right_trigger);

                telemetry.addData("Wheel Base ", "Encoders");
                telemetry.addData("Left Front Value: ", leftFront.getCurrentPosition());
                telemetry.addData("Right Front Value: ", rightFront.getCurrentPosition());
                telemetry.addData("Left Back Value: ", leftBack.getCurrentPosition());
                telemetry.addData("Right Back Value: ", rightBack.getCurrentPosition());

                telemetry.addData("Intake ", "Encoders");
                telemetry.addData("Main Intake Value: ", mainIntake.getCurrentPosition());

                telemetry.addData("Shooter ", "Encoders");
                telemetry.addData("Flywheel Value: ", frontFlywheel.getCurrentPosition());

                telemetry.addData("Wobble Goal Arm ", "Encoders");
                telemetry.addData("Wobble Goal Arm Value: ", wobbleGoalArm.getCurrentPosition());

                telemetry.addData("Limit Switch", limitSwitch.getState());
                telemetry.addData("Limit 2", limitSwitchRetracted.getState());

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Other Sensor ", "Values");
                telemetry.addData("IMU First Angle: ", angles.firstAngle);
                telemetry.addData("IMU Second Angle: ", angles.secondAngle);
                telemetry.addData("IMU Third Angle: ", angles.thirdAngle);

                telemetry.update();
            }   //end of while(opModeIsActive())
        }   //end of if(opModeIsActive())
    }   //end of runOpMode()

}
