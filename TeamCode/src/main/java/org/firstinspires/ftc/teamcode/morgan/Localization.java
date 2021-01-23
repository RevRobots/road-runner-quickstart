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

public class Localization implements Runnable {

    HardwareMap hardwareMap;

    RobotConfig robotConfig;

    DcMotor leftOdometer;
    DcMotor rightOdometer;
    DcMotor frontOdometer;

    BNO055IMU imu;
    Orientation angles;

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

    private boolean isRunning = true;
    int sleepTime;

    public Localization (HardwareMap hwMap, DcMotor lO, DcMotor rO, DcMotor fO, int sT) {

        hardwareMap = hwMap;

        leftOdometer = lO;
        rightOdometer = rO;
        frontOdometer = fO;

        sleepTime = sT;

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

    private void localize () {

        correctedLeftOdometerValue = (leftOdometer.getCurrentPosition() * leftOdometerMultiplier);
        correctedRightOdometerValue = (rightOdometer.getCurrentPosition() * rightOdometerMultiplier);

        double leftChange = correctedLeftOdometerValue - previouLeftOdometerValue;
        double rightChange = correctedRightOdometerValue = previousRightOdometerValue;

        changeInRobotOreintation = (leftChange - rightChange) / robotConfig.odometerTrackWidth;
        robotOrientation = robotOrientation + changeInRobotOreintation;

        correctedFrontOdometerValue = (frontOdometer.getCurrentPosition() * frontOdometerMultiplier);
        double rawFrontChange = correctedFrontOdometerValue - previousFrontOdometerValue;
        double frontChange = rawFrontChange - (changeInRobotOreintation * robotConfig.frontOdometerOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = frontChange;

        globalXPosition = globalXPosition + (p * Math.sin(robotOrientation)) + (n * Math.cos(robotOrientation));
        globalYPostion = globalYPostion + (p * Math.cos(robotOrientation)) + (n * Math.sin(robotOrientation));

        previouLeftOdometerValue = correctedLeftOdometerValue;
        previousRightOdometerValue = correctedRightOdometerValue;
        previousFrontOdometerValue = correctedFrontOdometerValue;

    }

    public double getGlobalXPosition () {
        return globalXPosition;
    }

    public double getGlobalYPostion () {
        return globalYPostion;
    }

    public double getRobotOrientation () {
        return (Math.toDegrees(robotOrientation) % 360);
    }

    public double getCorrectedLeftOdometerValue () {
        return correctedLeftOdometerValue;
    }

    public double getCorrectedRightOdometerValue () {
        return correctedRightOdometerValue;
    }

    public double getCorrectedFrontOdometerValue () {
        return correctedFrontOdometerValue;
    }

    public void stop() {
        isRunning = false;
    }

    @Override
    public void run() {
        while(isRunning) {
            localize();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}
