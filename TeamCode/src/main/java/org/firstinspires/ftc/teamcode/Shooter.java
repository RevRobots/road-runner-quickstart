package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    DcMotor flywheel;
    Servo trigger;

    RobotConfig robotConfig = new RobotConfig();
    ElapsedTime shootTime = new ElapsedTime();

    public Shooter (DcMotor f, Servo t) {
        f = flywheel;
        t = trigger;
    }

    public void teleOpShooter (Gamepad gamepad2) {
        if (gamepad2.right_trigger != 0) {
            flywheel.setPower(1);
            if (shootTime.milliseconds() >= 1000) {
                trigger.setPosition(0.5);
                shootTime.reset();
            } else {
                trigger.setPosition(0);
            }
        } else {
            flywheel.setPower(0);
        }
    }

}
