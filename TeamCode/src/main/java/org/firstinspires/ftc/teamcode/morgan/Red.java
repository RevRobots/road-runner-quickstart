package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.morgan.DriveTrain;
import org.firstinspires.ftc.teamcode.morgan.Intake;
import org.firstinspires.ftc.teamcode.morgan.RobotConfig;
import org.firstinspires.ftc.teamcode.morgan.Shooter;
import org.firstinspires.ftc.teamcode.morgan.WobbleArm;

import java.util.List;

@Autonomous (name = "Red", group = "red")
//@Disabled
public class Red extends LinearOpMode {

    RobotConfig robotConfig;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    DcMotor intake;
    CRServo intakeWheel;

    DcMotor rotation, flywheel;
    Servo ringPusher;

    DcMotor wobbleGoalArm;
    CRServo finger;

    DriveTrain driveTrain;
    Intake intakeClass;
    Shooter shooter;
    WobbleArm wobbleArm;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AQUWr4X/////AAABme38EPssRkvls9+q/BGPYgxKXBXELWHMdkTcCqUqHeyDpyXGWFLCTABgDXEMGe1EmsnDQxmJ7WQ069J3YSv+kOcfq3g2EnwZr2O3DujsIU1nT0aXgLlAtQU2r7wWAgHvR9ADO5pe/q7MzCyhjSTQLCgizGFLgmqfre0A9rjYcXYbYw11R3P7VRHnL3QHn3QH2oFVQfMb+dIzmZkfv0cd5qWvdhjovYF8hpZ/HT7veIa8ZQ9CIQ0541pxplXVud80z1xWpjFGJPaoQGO+xKWZ8E+Zlu7z5umiaV1+ChGeJ9pPyIJn0LsnoIHumZoYb4di4tFygMPVmH8ChsTlGJjaPBSCRBFjxzBqsXmBZY7eCa6S";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode () {

        robotConfig = new RobotConfig(hardwareMap);

        leftFront = hardwareMap.dcMotor.get(robotConfig.leftFront);
        rightFront = hardwareMap.dcMotor.get(robotConfig.rightFront);
        leftBack = hardwareMap.dcMotor.get(robotConfig.leftBack);
        rightBack = hardwareMap.dcMotor.get(robotConfig.rightBack);

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

        wobbleGoalArm = hardwareMap.dcMotor.get(robotConfig.wobbleArm);

        intakeWheel = hardwareMap.crservo.get(robotConfig.intakeWheels);
        intakeWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        ringPusher = hardwareMap.servo.get(robotConfig.ringPusher);

        finger = hardwareMap.crservo.get(robotConfig.finger);

        driveTrain = new DriveTrain(leftFront, rightFront, leftBack, rightBack);
        intakeClass = new Intake(intake, intakeWheel);
        shooter = new Shooter(rotation, flywheel, ringPusher);
        wobbleArm = new WobbleArm(wobbleGoalArm, finger);

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

        waitForStart();

        finger.setPower(1);

        wobbleArm.armUp(0.25, 200);

        wobbleGoalArm.setPower(0.15);

        driveTrain.forwards(0.5, 100);

        driveTrain.turnRight(0.5, 230);

        driveTrain.forwards(0.5, 2200);

        wobbleArm.armUp(0.5, 1000);

        finger.setPower(-1);

        driveTrain.backwards(0.5, 100);

        driveTrain.turnLeft(0.5, 500);

        driveTrain.forwards(0.5, 1500);

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

                            } else if (recognition.getLabel() == "Quad") {

                                driveTrain.turnRight(0.75, 300);

                                driveTrain.forwards(0.75, 1500);

                                wobbleArm.armUp(0.5, 1000);

                                wobbleArm.release(1, 1000);

                            }
                        }
                        telemetry.update();
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
