package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

@Disabled
@Autonomous (name = "Red Left Line", group = "Red")
public class RedLeftLine extends LinearOpMode {
    MorganConstants robot;

    private DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;

    private DcMotor mainIntake = null;
    private CRServo hopperIntake = null;

    private DcMotorEx frontFlywheel = null, backFlywheel = null;
    private Servo ringPusher = null;

    private DcMotor wobbleArm = null;
    private CRServo finger = null;

    private BNO055IMU imu = null;

    @Override
    public void runOpMode() {

    }
}
