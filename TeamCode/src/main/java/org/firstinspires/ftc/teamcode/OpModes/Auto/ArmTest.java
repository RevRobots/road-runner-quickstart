package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;
import org.firstinspires.ftc.teamcode.Libs.ArmClass;

@Autonomous
public class ArmTest extends LinearOpMode {
    DcMotor arm = null;
    DigitalChannel extendedLimit = null, retractedLimit = null;

    ArmClass armClass;

    @Override
    public void runOpMode() {
        MorganConstants robot = new MorganConstants(hardwareMap);

        arm = hardwareMap.get(DcMotor.class, robot.WOBBLE_GOAL_ARM);
        extendedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_EXTENDED_LIMIT_SWITCH);
        retractedLimit = hardwareMap.get(DigitalChannel.class, robot.ARM_RETRACTED_LIMIT_SWITCH);

        armClass = new ArmClass(arm, null, retractedLimit, extendedLimit);

        waitForStart();

        while(opModeIsActive()) {
            armClass.extendArm(0.5);

            while(retractedLimit.getState() == true) {
                arm.setPower(-0.5);
            }
            arm.setPower(0);
        }
    }
}
