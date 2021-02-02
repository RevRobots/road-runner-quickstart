package org.firstinspires.ftc.teamcode.morgan.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.morgan.DriveTrain;
import org.firstinspires.ftc.teamcode.morgan.Intake;
import org.firstinspires.ftc.teamcode.morgan.RobotConfig;
import org.firstinspires.ftc.teamcode.morgan.Shooter;
import org.firstinspires.ftc.teamcode.morgan.WobbleArm;

import java.util.List;

//@Disabled
@Autonomous (name = "Odometery Autonomous", group = "Red")
public class OdoAuto extends LinearOpMode {

    RobotConfig robotConfig;

    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotor leftOdometer, rightOdometer, frontOdometer;

    DcMotor intake;

    DcMotor rotation;
    DcMotor flywheel;
    Servo ringPusher;
    DistanceSensor distanceSensor;

    DcMotor wobbleGoal;
    CRServo finger;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    DriveTrain drive;
    Intake intakeClass;
    Shooter shooter;
    WobbleArm wobbleArm;

    ElapsedTime timer;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AQUWr4X/////AAABme38EPssRkvls9+q/BGPYgxKXBXELWHMdkTcCqUqHeyDpyXGWFLCTABgDXEMGe1EmsnDQxmJ7WQ069J3YSv+kOcfq3g2EnwZr2O3DujsIU1nT0aXgLlAtQU2r7wWAgHvR9ADO5pe/q7MzCyhjSTQLCgizGFLgmqfre0A9rjYcXYbYw11R3P7VRHnL3QHn3QH2oFVQfMb+dIzmZkfv0cd5qWvdhjovYF8hpZ/HT7veIa8ZQ9CIQ0541pxplXVud80z1xWpjFGJPaoQGO+xKWZ8E+Zlu7z5umiaV1+ChGeJ9pPyIJn0LsnoIHumZoYb4di4tFygMPVmH8ChsTlGJjaPBSCRBFjxzBqsXmBZY7eCa6S";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    boolean reset = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robotConfig = new RobotConfig(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, robotConfig.leftFront);
        rightFront = hardwareMap.get(DcMotor.class, robotConfig.rightFront);
        leftBack = hardwareMap.get(DcMotor.class, robotConfig.leftBack);
        rightBack = hardwareMap.get(DcMotor.class, robotConfig.rightBack);

        leftOdometer = hardwareMap.get(DcMotor.class, robotConfig.leftOdometer);
        rightOdometer = hardwareMap.get(DcMotor.class, robotConfig.rightOdometer);
        frontOdometer = hardwareMap.get(DcMotor.class, robotConfig.frontOdometer);

        intake = hardwareMap.get(DcMotor.class, robotConfig.intake);

        rotation = hardwareMap.get(DcMotor.class, robotConfig.rotation);
        flywheel = hardwareMap.get(DcMotor.class, robotConfig.flywheel);
        ringPusher = hardwareMap.get(Servo.class, robotConfig.ringPusher);

        wobbleGoal = hardwareMap.get(DcMotor.class, robotConfig.wobbleArm);
        finger = hardwareMap.get(CRServo.class, robotConfig.finger);

        imu = hardwareMap.get(BNO055IMU.class, robotConfig.imu);

        drive = new DriveTrain(leftFront, rightFront, leftBack, rightBack, leftOdometer, rightOdometer, frontOdometer);
        intakeClass = new Intake(intake);
        shooter = new Shooter(rotation, flywheel, ringPusher);
        wobbleArm = new WobbleArm(wobbleGoal, finger);

        timer = new ElapsedTime();

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        finger.setPower(-1);

        wobbleGoal.setPower(0.1);

        drive.forwardsEncoder(0.25, 600);

        timer.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            if (recognition.getLabel() == "Single") {
                                drive.rightEncoder(0.25, 600);

                                drive.forwardsEncoder(0.375, 2125);

                                drive.turnLeftEncoder(0.25, 400);

                                drive.forwardsEncoder(0.25, 800);

                                drive.moveMotor(wobbleGoal, 0.25, 400);

                                Thread.sleep(1000);

                                finger.setPower(1);

                                Thread.sleep(1000);

                                finger.setPower(0);

                                drive.backwardsEncoder(0.25, 800);

                                Thread.sleep(25000);
                            } else if (recognition.getLabel() == "Quad") {
                                drive.rightEncoder(0.25, 600);

                                drive.forwardsEncoder(0.375, 4000);

                                drive.turnLeftEncoder(0.25, 100);

                                drive.moveMotor(wobbleGoal, 0.25, 400);

                                Thread.sleep(1000);

                                finger.setPower(1);

                                Thread.sleep(1000);

                                finger.setPower(0);

                                drive.turnRightEncoder(0.25, 100);

                                drive.backwardsEncoder(0.25, 1750);

                                Thread.sleep(25000);
                            }
                        }

                        telemetry.update();

                        if (reset == false) {
                            drive.forwardsEncoder(0.25, 100);
                            reset = true;
                        }

                        if(timer.milliseconds() >= 3000) {
                            if(updatedRecognitions.size() == 0) {
                                drive.rightEncoder(0.25, 600);

                                drive.forwardsEncoder(0.375, 2125);

                                drive.turnLeftEncoder(0.25, 100);

                                drive.moveMotor(wobbleGoal, 0.25, 400);

                                Thread.sleep(1000);

                                finger.setPower(1);

                                Thread.sleep(1000);

                                finger.setPower(0);

                                drive.backwardsEncoder(0.25, 100);

                                Thread.sleep(25000);
                            }
                        }
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}