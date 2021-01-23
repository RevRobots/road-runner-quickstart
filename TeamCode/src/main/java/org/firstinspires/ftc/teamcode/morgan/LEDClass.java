package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LEDClass {

    RevBlinkinLedDriver led;

    public LEDClass (RevBlinkinLedDriver l) {
        led = l;
    }

    public void setNorm () {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
    }

}
