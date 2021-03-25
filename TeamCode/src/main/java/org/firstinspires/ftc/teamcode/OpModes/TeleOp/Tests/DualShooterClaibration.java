package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Libs.DualShooterControlThread;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;

@TeleOp (name = "Dual Shooter Speed Calibration", group = "Calibration")
@Disabled
public class DualShooterClaibration extends LinearOpMode {
    MorganConstants robot;

    DcMotor frontFlywheel = null, backFlywheel = null;
    Servo ringPusher = null;

    DualShooterControlThread shooterControl;
    Thread shooterThread;
    RingPusherClass ringPusherClass;
    ElapsedTime timer = new ElapsedTime();

    private int shooterRPM = 5000;
    private boolean isPressed = false;

    @Override
    public void runOpMode() {
        robot = new MorganConstants(hardwareMap);

        frontFlywheel = hardwareMap.get(DcMotor.class, robot.FRONT_FLYWHEEL);
        backFlywheel = hardwareMap.get(DcMotor.class, robot.BACK_FLYWHEEL);
        ringPusher = hardwareMap.get(Servo.class, robot.RING_PUSHER);

        shooterControl = new DualShooterControlThread(frontFlywheel, backFlywheel);
        shooterThread = new Thread(shooterControl);
        ringPusherClass = new RingPusherClass(ringPusher);

        waitForStart();

        shooterThread.start();

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                shooterControl.setTargetShooterRPM(shooterRPM);

                if(gamepad2.right_trigger != 0) {
                    ringPusher.setPosition(robot.RING_PUSHER_TRIGGERED);
                } else {
                    ringPusher.setPosition(robot.RING_PUSHER_RETRACTED);
                }

                if(gamepad1.left_stick_y != 0) {
                    shooterRPM = shooterRPM + -((int)gamepad1.left_stick_y);
                } else if(gamepad2.dpad_up && timer.milliseconds() >= 10) {
                    shooterRPM++;
                    timer.reset();
                } else if(gamepad2.dpad_down && timer.milliseconds() >= 10) {
                    shooterRPM--;
                    timer.reset();
                }

                telemetry.addData("Shooter RPM", shooterRPM);
                telemetry.update();
            }

            shooterControl.setTargetShooterRPM(0);
            shooterControl.stopShooterThread();
        }
    }
}