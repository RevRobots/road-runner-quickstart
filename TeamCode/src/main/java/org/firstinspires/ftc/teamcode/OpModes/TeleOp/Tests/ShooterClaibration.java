package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Libs.RingPusherClass;
import org.firstinspires.ftc.teamcode.Libs.ShooterControlThread;

@TeleOp (name = "Shooter Speed Calibration", group = "Calibration")
@Disabled
public class ShooterClaibration extends LinearOpMode {
    MorganConstants robot;

    DcMotor flywheel = null;
    Servo ringPusher = null;

    ShooterControlThread shooterControlThread;
    Thread shooterThread;
    RingPusherClass ringPusherClass;
    ElapsedTime timer = new ElapsedTime();

    private int shooterRPM = 5000;
    private boolean isPressed = false;

    @Override
    public void runOpMode() {
        robot = new MorganConstants(hardwareMap);

        flywheel = hardwareMap.get(DcMotor.class, robot.FRONT_FLYWHEEL);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        ringPusher = hardwareMap.get(Servo.class, robot.RING_PUSHER);

        shooterControlThread = new ShooterControlThread(flywheel);
        shooterThread = new Thread(shooterControlThread);
        ringPusherClass = new RingPusherClass(ringPusher);

        waitForStart();

        shooterThread.start();

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                shooterControlThread.setTargetShooterRPM(shooterRPM);

                if(gamepad2.right_trigger != 0) {
                    ringPusher.setPosition(0.5);
                } else {
                    ringPusher.setPosition(0);
                }

                if(gamepad2.dpad_up && timer.milliseconds() >= 10) {
                    shooterRPM++;
                    timer.reset();
                } else if(gamepad2.dpad_down && timer.milliseconds() >= 10) {
                    shooterRPM--;
                    timer.reset();
                }

                telemetry.addData("Shooter RPM", shooterRPM);
                telemetry.update();
            }

            shooterControlThread.setTargetShooterRPM(0);
            shooterControlThread.stopShooterThread();
        }
    }
}