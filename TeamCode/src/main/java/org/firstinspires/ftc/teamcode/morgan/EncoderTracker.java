package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    DcMotor wobbleGoal;
    CRServo finger;

    CRServo jacksonE;

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

        wobbleGoal = hardwareMap.dcMotor.get("wobbleArm");
        finger = hardwareMap.crservo.get("finger");

    }

    @Override
    public void start () {

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

        telemetry.update();

    }

    @Override
    public void stop () {

    }

}
