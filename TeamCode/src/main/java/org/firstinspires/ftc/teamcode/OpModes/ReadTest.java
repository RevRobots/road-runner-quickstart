package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp (name = "Read File Test", group = "Test")
@Disabled
public class ReadTest extends LinearOpMode {
    File armPositionFile = AppUtil.getInstance().getSettingsFile("armPosition.txt");

    @Override
    public void runOpMode() {
        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive()) {
                telemetry.addData("File:", ReadWriteFile.readFile(armPositionFile));
                telemetry.update();
            }
        }
    }
}
