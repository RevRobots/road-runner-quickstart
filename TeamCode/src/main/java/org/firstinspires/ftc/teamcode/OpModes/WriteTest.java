package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp (name = "Write File Test", group = "Test")
@Disabled
public class WriteTest extends LinearOpMode {
    File armPositionFile = AppUtil.getInstance().getSettingsFile("armPosition.txt");

    @Override
    public void runOpMode() {
        ReadWriteFile.writeFile(armPositionFile, String.valueOf(100));
        waitForStart();
        if(opModeIsActive()) {

        }
    }
}
