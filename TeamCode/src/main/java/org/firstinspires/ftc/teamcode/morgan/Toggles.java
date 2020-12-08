package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Toggles {

    ElapsedTime onOffTimer = new ElapsedTime();

    public Toggles () {

    }

    // TODO: create a return statement for the testFlag variable
    public void onOffToggle (Boolean testFlag, int delay, char buttonPressed, Gamepad gamepad) {

        if (buttonPressed == 'A') {

            if (gamepad.a && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.a && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        } else if (buttonPressed == 'B') {

            if (gamepad.b && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.b && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        } else if (buttonPressed == 'X') {

            if (gamepad.x && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.x && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        } else if (buttonPressed == 'Y') {

            if (gamepad.y && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.y && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        } else if (buttonPressed == 'U') {

            if (gamepad.dpad_up && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.dpad_up && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        } else if (buttonPressed == 'D') {

            if (gamepad.dpad_down && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.dpad_down && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        } else if (buttonPressed == 'L') {

            if (gamepad.dpad_left && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.dpad_left && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        } else if (buttonPressed == 'R') {

            if (gamepad.dpad_right && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.dpad_right && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        } else if (buttonPressed == 'M') {

            if (gamepad.left_bumper && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.left_bumper && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        } else if (buttonPressed == 'P') {

            if (gamepad.right_bumper && testFlag == true && onOffTimer.milliseconds() >= delay) {
                testFlag = false;
                onOffTimer.reset();
            } else if (gamepad.right_bumper && testFlag == false && onOffTimer.milliseconds() >= delay) {
                testFlag = true;
                onOffTimer.reset();
            }

        }

    }

    public void counter () {}

}
