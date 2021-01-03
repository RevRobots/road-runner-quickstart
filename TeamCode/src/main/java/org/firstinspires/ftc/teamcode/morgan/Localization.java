package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Localization {

    HardwareMap hardwareMap;

    RobotConfig robotConfig;

    DcMotor leftOdometer;
    DcMotor rightOdometer;
    DcMotor frontOdometer;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    double leftPosition;
    double rightPosition;
    double frontPosition;

    double previousX;
    double previousY;

    double newX;
    double newY;

    double currentX;
    double currentY;

    double theta;

    public Localization (HardwareMap hwMap, DcMotor lO, DcMotor rO, DcMotor fO) {

        hardwareMap = hwMap;

        leftOdometer = lO;
        rightOdometer = rO;
        frontOdometer = fO;

        robotConfig = new RobotConfig(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

    }

    public void localize () {

        theta = angles.firstAngle + 90;

        leftPosition = leftOdometer.getCurrentPosition();
        rightPosition = rightOdometer.getCurrentPosition();
        frontPosition = frontOdometer.getCurrentPosition();

        double yWheels = (leftPosition + rightPosition)/2;
        double xWheel = frontPosition;

        newX = (yWheels * Math.cos(theta)) + (xWheel * Math.cos(theta));
        newY = (yWheels * Math.sin(theta)) + (xWheel * Math.sin(theta));

        currentX = newX + previousX;
        currentY = newY + previousY;

        previousX = currentX;
        previousY = currentY;

        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
