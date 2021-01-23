package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp (name = "Encoder Tracker", group = "Tests")
public class EncoderTracker extends OpMode {

    DcMotor leftEncoder;
    DcMotor rightEncoder;
    DcMotor middleEncoder;

    DcMotor intake;
    CRServo intakeWheels;

    DcMotor rotation;
    DcMotor flywheel;
    Servo ringPusher;
    DistanceSensor distanceSensor;

    DcMotor wobbleGoal;
    CRServo finger;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void init () {

        leftEncoder = hardwareMap.dcMotor.get("leftFront");
        rightEncoder = hardwareMap.dcMotor.get("rightFront");
        middleEncoder = hardwareMap.dcMotor.get("leftBack");

        intake = hardwareMap.dcMotor.get("intake");
        intakeWheels = hardwareMap.crservo.get("intakeWheels");

        rotation = hardwareMap.dcMotor.get("rotation");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        ringPusher = hardwareMap.servo.get("ringPusher");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        wobbleGoal = hardwareMap.dcMotor.get("wobbleArm");
        finger = hardwareMap.crservo.get("finger");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }

    @Override
    public void start () {

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    }

    @Override
    public void loop () {

        telemetry.addData("Wheelbase", "Encoders");

        telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
        telemetry.addData("Middle Encoder", middleEncoder.getCurrentPosition());

        telemetry.addData("Intake", "Encoders");

        telemetry.addData("Intake", intake.getCurrentPosition());

        telemetry.addData("Shooter", "Encoders");

        telemetry.addData("Rotation", rotation.getCurrentPosition());
        telemetry.addData("Flywheel", flywheel.getCurrentPosition());
        telemetry.addData("Ring Pusher", ringPusher.getPosition());

        telemetry.addData("Wobble Arm", "Encoders");

        telemetry.addData("Wobble Arm", wobbleGoal.getCurrentPosition());

        telemetry.addData("Extra", "Data");

        telemetry.addData("IMU First Angle", angles.firstAngle);
        telemetry.addData("IMU Second Angle", angles.secondAngle);
        telemetry.addData("IMU Third Angle", angles.thirdAngle);
        telemetry.addData("IMU getAngularOrientation", imu.getAngularOrientation());

        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.MM));

        telemetry.update();

    }

    @Override
    public void stop () {

    }

}
