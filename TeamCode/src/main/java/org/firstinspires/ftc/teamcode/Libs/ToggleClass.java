package org.firstinspires.ftc.teamcode.Libs;

public class ToggleClass {
    private boolean firstRun = true;
    private boolean buttonPressed = false;
    private boolean flag;

    public ToggleClass() {

    }

    public boolean buttonReleaseToggle(boolean buttonPress, boolean initialFlagState) {
        if(firstRun) {
            flag = initialFlagState;
            firstRun = false;
        }

        if(buttonPress && !buttonPressed) {
            buttonPressed = true;
        } else if(!buttonPress && buttonPressed) {
            if(!flag) {
                flag = true;
            } else if(flag) {
                flag = false;
            }
            buttonPressed = false;
        }

        return flag;
    }
}
