package org.firstinspires.ftc.teamcode.morgan.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.morgan.RobotConfig;
import org.firstinspires.ftc.teamcode.morgan.Shooter;

@TeleOp (name="Shooter Testing", group="test")
public class ShooterTest extends LinearOpMode {
    DcMotor rotation, flywheel;
    Servo ringPusher;

    Shooter shooter;
    RobotConfig robotConfig = new RobotConfig(hardwareMap);

    @Override
    public void runOpMode () throws InterruptedException {
        rotation = hardwareMap.get(DcMotor.class, robotConfig.rotation);
        flywheel = hardwareMap.get(DcMotor.class, robotConfig.flywheel);
        ringPusher = hardwareMap.get(Servo.class, robotConfig.ringPusher);

        shooter = new Shooter(rotation, flywheel, ringPusher);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if(gamepad1.a) {
                    flywheel.setPower(0.85);

                    shooter.trigger();

                    Thread.sleep(3000);

                    shooter.trigger();

                    Thread.sleep(3000);

                    shooter.trigger();

                    flywheel.setPower(0);
                }
            }
        }

    }

}
