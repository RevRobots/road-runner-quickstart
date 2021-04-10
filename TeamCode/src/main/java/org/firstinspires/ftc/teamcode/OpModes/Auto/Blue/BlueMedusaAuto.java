package org.firstinspires.ftc.teamcode.OpModes.Auto.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Hardware.PoseSettings;
import org.firstinspires.ftc.teamcode.Libs.ArmClass;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterControlThread;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous (name = "Blue Medusa", group = "Blue")
public class BlueMedusaAuto extends LinearOpMode {
    DcMotorEx flywheel = null;
    Servo ringPusher = null;

    DcMotor arm = null;
    CRServo finger = null;
    DigitalChannel extendedLimit = null, retractedLimit = null;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private boolean stackDetected = false;
    private int ringCount = 0;

    private static final String VUFORIA_KEY = "AQUWr4X/////AAABme38EPssRkvls9+q/BGPYgxKXBXELWHMdkTcCqUqHeyDpyXGWFLCTABgDXEMGe1EmsnDQxmJ7WQ069J3YSv+kOcfq3g2EnwZr2O3DujsIU1nT0aXgLlAtQU2r7wWAgHvR9ADO5pe/q7MzCyhjSTQLCgizGFLgmqfre0A9rjYcXYbYw11R3P7VRHnL3QHn3QH2oFVQfMb+dIzmZkfv0cd5qWvdhjovYF8hpZ/HT7veIa8ZQ9CIQ0541pxplXVud80z1xWpjFGJPaoQGO+xKWZ8E+Zlu7z5umiaV1+ChGeJ9pPyIJn0LsnoIHumZoYb4di4tFygMPVmH8ChsTlGJjaPBSCRBFjxzBqsXmBZY7eCa6S";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        MorganConstants robot = new MorganConstants(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PoseSettings poses = new PoseSettings();

        flywheel = hardwareMap.get(DcMotorEx.class, robot.FRONT_FLYWHEEL);
        ringPusher = hardwareMap.get(Servo.class, robot.RING_PUSHER);
        if(robot.FLYWHEELS_REVERSED) {
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        ringPusher.setPosition(robot.RING_PUSHER_RETRACTED);

        arm = hardwareMap.get(DcMotor.class, robot.WOBBLE_GOAL_ARM);
        if(robot.WOBBLE_GOAL_ARM_REVERSED) {
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        finger = hardwareMap.get(CRServo.class, robot.WOBBLE_GOAL_FINGER);
        if(robot.WOBBLE_GOAL_FINGER_REVERSED) {
            finger.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        extendedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_EXTENDED_LIMIT_SWITCH);
        retractedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_RETRACTED_LIMIT_SWITCH);

        ShooterControlThread shooterControl = new ShooterControlThread(flywheel);
        Thread shooterThread = new Thread(shooterControl);
        RingPusherClass ringPusherClass = new RingPusherClass(ringPusher);
        ArmClass wobbleGoal = new ArmClass(arm, finger, retractedLimit, extendedLimit);

        //TODO: CHANGE THIS FOR EVERY PROGRAM BUDDY
        drive.setPoseEstimate(poses.blueMedusaStartingPoint);

        Trajectory ringDetectionMovement = drive.trajectoryBuilder(poses.blueMedusaStartingPoint, Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20.5, 17.5), Math.toRadians(0))
                .build();

        Trajectory boxAMovements = drive.trajectoryBuilder(new Pose2d(-20.5, 17.5, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(12, 37), Math.toRadians(90))
                .build();

        Trajectory powerShootAMovement = drive.trajectoryBuilder(new Pose2d(20, 30, Math.toRadians(90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, 6, Math.toRadians(0)), Math.toRadians(-90))
                .build();

        Trajectory boxBMovements = drive.trajectoryBuilder(new Pose2d(-20.5, -17.5, Math.toRadians(-90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -12), Math.toRadians(0))
                .build();

        Trajectory powerShotBMovement = drive.trajectoryBuilder(new Pose2d(35, -16, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(4, -8, Math.toRadians(0)), Math.toRadians(90))
                .build();

        Trajectory boxCMovements = drive.trajectoryBuilder(new Pose2d(-20, -17.5, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(poses.redBoxCLeftDrop, Math.toRadians(-90))
                .build();

        Trajectory powerShotCMovement = drive.trajectoryBuilder(poses.redBoxCLeftDrop, Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -5, Math.toRadians(0)), Math.toRadians(90))
                .build();

        Trajectory secondPowerBoi = drive.trajectoryBuilder(new Pose2d(0, -5.5, Math.toRadians(0)), Math.toRadians(0))
                .strafeLeft(6)
                .build();

        Trajectory thirdPowerLad = drive.trajectoryBuilder(new Pose2d(0, 0.5, Math.toRadians(0)), Math.toRadians(0))
                .strafeLeft(6)
                .build();

        Trajectory park = drive.trajectoryBuilder(new Pose2d(0, -5, Math.toRadians(7.5)), Math.toRadians(0))
                .forward(10)
                .build();

        initVuforia();
        initTfod();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

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

        ElapsedTime detectionTimer = new ElapsedTime();
        ElapsedTime revTimer = new ElapsedTime();

        waitForStart();

        shooterThread.start();
        shooterControl.setTargetShooterRPM(robot.IDLE_SHOOTER_RPM);

        if(opModeIsActive()) {
            finger.setPower(1);
            //sleep(6500);
            drive.followTrajectory(ringDetectionMovement);
            detectionTimer.reset();
            while (stackDetected == false) {
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
                            if(recognition.getLabel() == "Single") {
                                stackDetected = true;
                                ringCount = 1;
                            } else if(recognition.getLabel() == "Quad") {
                                stackDetected = true;
                                ringCount = 4;
                            }
                        }
                        telemetry.update();
                        if(detectionTimer.milliseconds() >= robot.DETECTION_REST_TIME) {
                            stackDetected = true;
                            ringCount = 0;
                        }
                    }
                }
            }

            if(ringCount == 0) {
                drive.followTrajectory(boxAMovements);
                while(extendedLimit.getState() == true) {
                    arm.setPower(0.5);
                }
                arm.setPower(0);
                finger.setPower(-1);
                sleep(500);
                finger.setPower(0);
                while(retractedLimit.getState() == true) {
                    arm.setPower(-0.5);
                }
                arm.setPower(0);
                drive.followTrajectory(powerShootAMovement);
            } else if(ringCount == 1) {
                drive.followTrajectory(boxBMovements);
                while(extendedLimit.getState() == true) {
                    arm.setPower(0.5);
                }
                arm.setPower(0);
                finger.setPower(-1);
                sleep(500);
                finger.setPower(0);
                while(retractedLimit.getState() == true) {
                    arm.setPower(-0.5);
                }
                arm.setPower(0);
                drive.followTrajectory(powerShotBMovement);
            } else if(ringCount == 4) {
                drive.followTrajectory(boxCMovements);
                while(extendedLimit.getState() == true) {
                    arm.setPower(0.5);
                }
                arm.setPower(0);
                finger.setPower(-1);
                sleep(500);
                finger.setPower(0);
                while(retractedLimit.getState() == true) {
                    arm.setPower(-0.5);
                }
                arm.setPower(0);
                drive.followTrajectory(powerShotCMovement);
            }

            revTimer.reset();
            while(revTimer.milliseconds() < 1000) {
                shooterControl.setTargetShooterRPM(robot.POWER_SHOT_SHOOTER_RPM);
            }

            ringPusherClass.trigger(750);
            drive.turn(Math.toRadians(5));

            revTimer.reset();
            while(revTimer.milliseconds() < 1000) {
                shooterControl.setTargetShooterRPM(robot.POWER_SHOT_SHOOTER_RPM);
            }

            ringPusherClass.trigger(750);
            drive.turn(Math.toRadians(7));

            revTimer.reset();
            while(revTimer.milliseconds() < 1000) {
                shooterControl.setTargetShooterRPM(robot.POWER_SHOT_SHOOTER_RPM);
            }

            ringPusherClass.trigger(750);
            ringPusherClass.trigger(750);

            shooterControl.setTargetShooterRPM(0);
            shooterControl.stopShooterThread();

            drive.followTrajectory(park);
        }

        shooterControl.setTargetShooterRPM(0);
        shooterControl.stopShooterThread();

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