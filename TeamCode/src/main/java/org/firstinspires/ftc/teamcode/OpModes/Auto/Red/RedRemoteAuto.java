package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Libs.MechanumDriveClass;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterClass;
import org.firstinspires.ftc.teamcode.Libs.TwoPartIntakeClass;
import org.firstinspires.ftc.teamcode.Libs.ArmClass;

import java.io.File;
import java.util.List;

@Autonomous (name = "Red Remote Match Auto", group = "Red")
public class RedRemoteAuto extends LinearOpMode {
    MorganConstants robot;

    private DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;

    private DcMotor mainIntake = null;
    private CRServo hopperIntake = null;

    private DcMotorEx frontFlywheel = null, backFlywheel = null;
    private Servo ringPusher = null;

    private DcMotor wobbleArm = null;
    private CRServo finger = null;

    private BNO055IMU imu = null;

    MechanumDriveClass drive;
    TwoPartIntakeClass intake;
    /*DualShooterControlThread shooterControl;
    Thread shooterThread;*/
    ShooterClass shooter;
    RingPusherClass ringPusherClass;
    ArmClass wobbleGoal;

    ElapsedTime timer;

    File armPositionFile = AppUtil.getInstance().getSettingsFile("armPosition.txt");

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AQUWr4X/////AAABme38EPssRkvls9+q/BGPYgxKXBXELWHMdkTcCqUqHeyDpyXGWFLCTABgDXEMGe1EmsnDQxmJ7WQ069J3YSv+kOcfq3g2EnwZr2O3DujsIU1nT0aXgLlAtQU2r7wWAgHvR9ADO5pe/q7MzCyhjSTQLCgizGFLgmqfre0A9rjYcXYbYw11R3P7VRHnL3QHn3QH2oFVQfMb+dIzmZkfv0cd5qWvdhjovYF8hpZ/HT7veIa8ZQ9CIQ0541pxplXVud80z1xWpjFGJPaoQGO+xKWZ8E+Zlu7z5umiaV1+ChGeJ9pPyIJn0LsnoIHumZoYb4di4tFygMPVmH8ChsTlGJjaPBSCRBFjxzBqsXmBZY7eCa6S";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    boolean reset = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MorganConstants(hardwareMap);

        //robotConfig.initializeRobot();

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

        wobbleArm = hardwareMap.get(DcMotor.class, robot.WOBBLE_GOAL_ARM);
        finger = hardwareMap.get(CRServo.class, robot.WOBBLE_GOAL_FINGER);

        imu = hardwareMap.get(BNO055IMU.class, robot.IMU);

        drive = new MechanumDriveClass(leftFront, rightFront, leftBack, rightBack, imu);
        intake = new TwoPartIntakeClass(mainIntake, hopperIntake);
        /*shooterControl = new DualShooterControlThread(frontFlywheel, backFlywheel);
        shooterThread = new Thread(shooterControl);*/
        shooter = new ShooterClass(robot.RUN_DUAL_SHOOTER, frontFlywheel, backFlywheel);
        ringPusherClass = new RingPusherClass(ringPusher);
        wobbleGoal = new ArmClass(wobbleArm, finger, null, null);
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

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        finger.setPower(-1);

        wobbleArm.setPower(0.1);

        drive.driveForwardsEnocoder(25, 600);

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
                                drive.driveRightEncoder(25, 600);

                                drive.driveForwardsEnocoder(25, 2125);

                                drive.turnLeftEncoder(25, 350);

                                drive.driveForwardsEnocoder(25, 800);

                                wobbleGoal.extendArmEncoder(25, 400);

                                Thread.sleep(1000);

                                finger.setPower(1);

                                Thread.sleep(1000);

                                finger.setPower(0);

                                drive.driveBackwardsEncoder(25, 1100);

                                wobbleGoal.retractArmEncoder(0.25, 750);

                                shooter.setFlywheelPower(1);

                                drive.turnRightEncoder(25, 115);

                                Thread.sleep(1500);

                                ringPusherClass.trigger(250);

                                Thread.sleep(1000);

                                ringPusherClass.trigger(250);

                                Thread.sleep(1000);

                                ringPusherClass.trigger(250);

                                Thread.sleep(1500);

                                ringPusherClass.trigger(250);

                                Thread.sleep(1000);

                                ringPusherClass.trigger(250);

                                drive.driveForwardsEnocoder(25, 300);

                                ReadWriteFile.writeFile(armPositionFile, String.valueOf(wobbleGoal.currentArmTicks));

                                Thread.sleep(25000);
                            } else if (recognition.getLabel() == "Quad") {
                                drive.driveRightEncoder(25, 600);

                                drive.driveForwardsEnocoder(37.5, 4000);

                                wobbleGoal.extendArmEncoder(25, 400);

                                Thread.sleep(1000);

                                finger.setPower(1);

                                Thread.sleep(1000);

                                finger.setPower(0);

                                drive.driveBackwardsEncoder(25, 2100);

                                wobbleGoal.retractArmEncoder(0.25, 700);

                                shooter.setFlywheelPower(1);

                                drive.driveLeftEncoder(25, 1100);

                                Thread.sleep(1500);

                                ringPusherClass.trigger(250);

                                Thread.sleep(1000);

                                ringPusherClass.trigger(250);

                                Thread.sleep(1000);

                                ringPusherClass.trigger(250);

                                Thread.sleep(1500);

                                ringPusherClass.trigger(250);

                                drive.driveForwardsEnocoder(25, 500);

                                ReadWriteFile.writeFile(armPositionFile, String.valueOf(wobbleGoal.currentArmTicks));

                                Thread.sleep(25000);
                            }
                        }

                        telemetry.update();

                        if (reset == false) {
                            drive.driveForwardsEnocoder(25, 100);
                            reset = true;
                        }

                        if(timer.milliseconds() >= 2000) {
                            if(updatedRecognitions.size() == 0) {

                                drive.driveRightEncoder(25, 600);

                                shooter.setFlywheelVelocity(robot.IDLE_SHOOTER_RPM, robot.IDLE_SHOOTER_RPM);

                                drive.driveForwardsEnocoder(37.5, 1800);

                                wobbleGoal.extendArmEncoder(25, 300);

                                Thread.sleep(1000);

                                finger.setPower(1);

                                Thread.sleep(1000);

                                finger.setPower(0);

                                wobbleGoal.retractArmEncoder(0.25, 600);

                                drive.driveBackwardsEncoder(25, 100);

                                drive.driveLeftEncoder(25, 1100);

                                shooter.setFlywheelVelocity(robot.POWER_SHOT_SHOOTER_RPM, robot.POWER_SHOT_SHOOTER_RPM);

                                Thread.sleep(1000);

                                ringPusherClass.trigger(100);

                                Thread.sleep(500);

                                ringPusherClass.trigger(100);

                                Thread.sleep(500);

                                ringPusherClass.trigger(100);

                                Thread.sleep(500);

                                ringPusherClass.trigger(100);

                                Thread.sleep(500);

                                shooter.setFlywheelVelocity(0, 0l);

                                drive.driveForwardsEnocoder(25, 400);

                                ReadWriteFile.writeFile(armPositionFile, String.valueOf(wobbleGoal.currentArmTicks));

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
