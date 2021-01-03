package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Toggles {

    boolean isOnOffButtonPressed;
    boolean onOffFlag;

    boolean isMultiNumericTogglePressed;
    int multiNumericCounter;

    ElapsedTime onOffTimer = new ElapsedTime();

    public Toggles () {

    }

    public boolean onOffToggle (char buttonPressed, Gamepad gamepad) {

        if (buttonPressed == 'A') {

            if (gamepad.a) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.a && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        } else if (buttonPressed == 'B') {

            if (gamepad.b) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.b && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        } else if (buttonPressed == 'X') {

            if (gamepad.x) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.x && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        } else if (buttonPressed == 'Y') {

            if (gamepad.y) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.y && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        } else if (buttonPressed == 'U') {

            if (gamepad.dpad_up) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.dpad_up && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        } else if (buttonPressed == 'D') {

            if (gamepad.dpad_down) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.dpad_down && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        } else if (buttonPressed == 'L') {

            if (gamepad.dpad_left) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.dpad_left && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        } else if (buttonPressed == 'R') {

            if (gamepad.dpad_right) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.dpad_right && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        } else if (buttonPressed == 'M') {

            if (gamepad.left_bumper) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.left_bumper && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        } else if (buttonPressed == 'P') {

            if (gamepad.right_bumper) {

                isOnOffButtonPressed = true;

            } else if (!gamepad.right_bumper && isOnOffButtonPressed == true) {

                isOnOffButtonPressed = false;

                if (onOffFlag == false) {
                    onOffFlag = true;
                } else {
                    onOffFlag = false;
                }

            }

        }

        return onOffFlag;

    }

    public int multiNumericToggle (int minumumValue, int maximumValue, char buttonPressed, Gamepad gamepad) {

        if (buttonPressed == 'A') {

            if (gamepad.a) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.a && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        } else if (buttonPressed == 'B') {

            if (gamepad.b) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.b && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        } else if (buttonPressed == 'X') {

            if (gamepad.x) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.x && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        } else if (buttonPressed == 'Y') {

            if (gamepad.y) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.y && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        } else if (buttonPressed == 'U') {

            if (gamepad.dpad_up) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.dpad_up && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        } else if (buttonPressed == 'D') {

            if (gamepad.dpad_down) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.dpad_down && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        } else if (buttonPressed == 'L') {

            if (gamepad.dpad_left) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.dpad_left && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        } else if (buttonPressed == 'R') {

            if (gamepad.dpad_right) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.dpad_right && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        } else if (buttonPressed == 'M') {

            if (gamepad.left_bumper) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.left_bumper && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        } else if (buttonPressed == 'P') {

            if (gamepad.right_bumper) {
                isMultiNumericTogglePressed = true;
            } else if (!gamepad.right_bumper && isMultiNumericTogglePressed == true) {

                isMultiNumericTogglePressed = false;

                if (multiNumericCounter > maximumValue) {

                    multiNumericCounter = minumumValue;

                }

                multiNumericCounter++;

            }

        }

        return multiNumericCounter;

    }

}
