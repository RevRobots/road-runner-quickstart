package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Arcade Drive", group = "drive")
public class ArcadeDrive extends OpMode {

    DcMotor flywheel;
    Servo trigger;

    RobotConfig robotConfig;
    Shooter shooterClass;

    @Override
    public void init () {

        robotConfig = new RobotConfig();

        flywheel = hardwareMap.dcMotor.get(robotConfig.getShooter());
        trigger = hardwareMap.servo.get(robotConfig.getTrigger());

        shooterClass = new Shooter(flywheel, trigger);
    }

    @Override
    public void loop () {
        shooterClass.teleOpShooter(gamepad2);
    }

    @Override
    public void stop () {

    }

}
