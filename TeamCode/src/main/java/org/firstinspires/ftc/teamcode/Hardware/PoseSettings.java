package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseSettings {
    public static final Pose2d redFullStartPoint = new Pose2d(-63, -17.5, Math.toRadians(-90));
    public static final Pose2d redBoxCFirstPosition = new Pose2d(68, -36, Math.toRadians(-75));
    public static final Pose2d redBoxBFirstPosition = new Pose2d(35, -16, Math.toRadians(-90));

    public static Pose2d autoEndPosition;

    public PoseSettings() {
    }

    public void setEndingPosition(Pose2d pose) {
        this.autoEndPosition = pose;
    }
}
