package org.firstinspires.ftc.teamcode.Libs;

public class ToggleClass {
    //booleans used to keep track of the flag
    private boolean firstRun = true;
    private boolean buttonPressed = false;
    private boolean flag;

    public ToggleClass() {}

    /**
     * Method: buttonReleasedToggle(...)
     *  -
     * @param buttonPress - the button you want to press
     * @param initialFlagState - the starting point of the flags
     * @return - returns the value of the flags
     */
    public boolean buttonReleaseToggle(boolean buttonPress, boolean initialFlagState) {
        if(firstRun) {
            flag = initialFlagState;
            firstRun = false;
        }

        if(buttonPress && !buttonPressed) {
            buttonPressed = true;
        } else if(!buttonPress && buttonPressed) {
            if(flag) {
                flag = false;
            } else if(!flag) {
                flag = true;
            }
        }

        return flag;
    }
}
