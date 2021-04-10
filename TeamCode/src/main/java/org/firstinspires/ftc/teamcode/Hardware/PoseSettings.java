package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseSettings {
    public static final Pose2d redFullStartPoint = new Pose2d(-63, -17.5, Math.toRadians(-90));

    public static final Pose2d redGearsStartingPoint = new Pose2d(-63, -54, Math.toRadians(90));
    public static final Pose2d redMedusaStartPoint = new Pose2d(-63, -17.5, Math.toRadians(-90));

    public static final Pose2d redBoxCLeftDrop = new Pose2d(68, -36, Math.toRadians(-75));
    public static final Pose2d redBoxBLeftDrop = new Pose2d(35, -16, Math.toRadians(-90));
    public static final Pose2d redBoxABottomDrop = new Pose2d(-10, -54, Math.toRadians(0));

    public static final Pose2d blueGearsStartingPoint = new Pose2d(-63, 54, Math.toRadians(-90));
    public static final Pose2d blueMedusaStartingPoint = new Pose2d(-63, 17.5, Math.toRadians(90));

    public static final Pose2d blueBoxABottomDrop = new Pose2d(-10, 60, Math.toRadians(10));
    public static final Pose2d blueBoxBRightDrop = new Pose2d();

    public static Pose2d autoEndPosition;

    public PoseSettings() {
    }

    public void setEndingPosition(Pose2d pose) {
        this.autoEndPosition = pose;
    }
}
