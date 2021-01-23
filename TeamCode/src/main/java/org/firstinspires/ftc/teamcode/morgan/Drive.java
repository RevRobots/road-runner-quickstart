package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@TeleOp (name = "Right Hand Man", group = "Tele-Op")
public class Drive extends OpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;

    DcMotor leftOdometer, rightOdometer, frontOdometer;

    DcMotor intake;

    DcMotor rotation;
    DcMotor flywheel;

    DcMotor wobbleArm;

    CRServo intakeWheels;

    Servo ringPusher;

    CRServo finger;

    DistanceSensor distanceSensor;

    DriveTrain driveTrain;
    Intake intakeClass;
    Shooter shooter;
    WobbleArm wobbleArmClass;

    RobotConfig robotConfig;
    Localization localization;

    @Override
    public void init () {

        robotConfig = new RobotConfig(hardwareMap);

        leftFront = hardwareMap.dcMotor.get(robotConfig.leftFront);
        rightFront = hardwareMap.dcMotor.get(robotConfig.rightFront);
        leftBack = hardwareMap.dcMotor.get(robotConfig.leftBack);
        rightBack = hardwareMap.dcMotor.get(robotConfig.rightBack);

        leftOdometer = hardwareMap.dcMotor.get(robotConfig.leftOdometer);
        rightOdometer = hardwareMap.dcMotor.get(robotConfig.rightOdometer);
        frontOdometer = hardwareMap.dcMotor.get(robotConfig.frontOdometer);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.dcMotor.get(robotConfig.intake);

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rotation = hardwareMap.dcMotor.get(robotConfig.rotation);
        flywheel = hardwareMap.dcMotor.get(robotConfig.flywheel);

        //rotation.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        wobbleArm = hardwareMap.dcMotor.get(robotConfig.wobbleArm);

        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeWheels = hardwareMap.crservo.get(robotConfig.intakeWheels);
        intakeWheels.setDirection(DcMotorSimple.Direction.REVERSE);

        ringPusher = hardwareMap.servo.get(robotConfig.ringPusher);

        finger = hardwareMap.crservo.get(robotConfig.finger);

        distanceSensor = hardwareMap.get(DistanceSensor.class, robotConfig.distanceSensor);

        driveTrain = new DriveTrain(leftFront, rightFront, leftBack, rightBack, leftOdometer, rightOdometer, frontOdometer);
        intakeClass = new Intake(intake);
        shooter = new Shooter (rotation, flywheel, ringPusher);
        wobbleArmClass = new WobbleArm(wobbleArm, finger);

        localization = new Localization(hardwareMap, leftOdometer, rightOdometer, frontOdometer, 75);

    }

    @Override
    public void start () {

    }

    @Override
    public void loop () {

        driveTrain.driveTrainControl(gamepad1);
        intakeClass.intakeControl(gamepad1);
        shooter.shooterControl(gamepad2);
        wobbleArmClass.wobbleArmControl(gamepad2);

        telemetry.addData("Speed Divider | 1 A |", driveTrain.driveSpeedLimiter);
        telemetry.addData("Rotation Speed Divider |  2 Y |", shooter.rotationLimiter);
        telemetry.addData("isLoading Toggle | 2 A |", shooter.isLoading);
        telemetry.addData("Arm Speed Divider | 2 U |", wobbleArmClass.wobbleArmLimiter);

        telemetry.addData("Distance Sensor Output: ", distanceSensor.getDistance(DistanceUnit.MM));

        if (distanceSensor.getDistance(DistanceUnit.MM) <= 175) {
            telemetry.addData("Ring Number: ", "0");
        } else if (distanceSensor.getDistance(DistanceUnit.MM) <= 140) {
            telemetry.addData("Ring Number: ", "1");
        } else if (distanceSensor.getDistance(DistanceUnit.MM) <= 120) {
            telemetry.addData("Ring Number: ", 2);
        } else {
            telemetry.addData("Ring Number: ", "3");
        }

        telemetry.addData("Ring Pusher Position: ", ringPusher.getPosition());

        telemetry.update();

    }

    @Override
    public void stop () {

    }

}
