package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

        intakeWheels = hardwareMap.crservo.get(robotConfig.intakeWheels);
        intakeWheels.setDirection(DcMotorSimple.Direction.REVERSE);

        ringPusher = hardwareMap.servo.get(robotConfig.ringPusher);

        finger = hardwareMap.crservo.get(robotConfig.finger);

        driveTrain = new DriveTrain(leftFront, rightFront, leftBack, rightBack);
        intakeClass = new Intake(intake, intakeWheels);
        shooter = new Shooter (rotation, flywheel, ringPusher);
        wobbleArmClass = new WobbleArm(wobbleArm, finger);

        localization = new Localization(hardwareMap, leftOdometer, rightOdometer, frontOdometer);

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

        localization.localize();

        telemetry.addData("X Position: ", localization.currentX);
        telemetry.addData("Y Position: ", localization.currentY);
        telemetry.addData("Angle: ", localization.angles.firstAngle);

        telemetry.update();

    }

    @Override
    public void stop () {

    }

}
