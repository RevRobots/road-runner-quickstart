package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

@TeleOp
@Disabled
public class ShooterTest extends LinearOpMode {
    MorganConstants robot;

    DcMotor flywheel = null;
    Servo ringPusher = null;

    @Override
    public void runOpMode() {
        robot = new MorganConstants(hardwareMap);

        flywheel = hardwareMap.get(DcMotor.class, robot.FRONT_FLYWHEEL);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        ringPusher = hardwareMap.get(Servo.class, robot.RING_PUSHER);

        waitForStart();

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                if (gamepad2.left_trigger != 0) {
                    flywheel.setPower(0.85);
                } else {
                    flywheel.setPower(0);
                }

                if (gamepad2.right_trigger != 0) {
                    //trigger();
                    ringPusher.setPosition(0.5);
                } else {
                    ringPusher.setPosition(0);
                }
            }
        }
    }
}
