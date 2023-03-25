package frc.robot.Objects;

import frc.robot.CameraConstants;

public class BlueTags {
    GoalApriltag leftTag;
    GoalApriltag middleTag;
    GoalApriltag rightTag;
    FeederApriltag feederTag;

    public BlueTags() {
        leftTag = new GoalApriltag(8, CameraConstants.CUBE_TAG_HEIGHT_INCHES);
        middleTag = new GoalApriltag(7, CameraConstants.CUBE_TAG_HEIGHT_INCHES);
        rightTag = new GoalApriltag(6, CameraConstants.CUBE_TAG_HEIGHT_INCHES);

        feederTag = new FeederApriltag(4, CameraConstants.FEEDER_STATION_TAG_HEIGHT_INCHES);
    }

    public GoalApriltag getLeftTag() {
        return leftTag;
    }

    public void setLeftTag(GoalApriltag leftTag) {
        this.leftTag = leftTag;
    }

    public GoalApriltag getMiddleTag() {
        return middleTag;
    }

    public void setMiddleTag(GoalApriltag middleTag) {
        this.middleTag = middleTag;
    }

    public GoalApriltag getRightTag() {
        return rightTag;
    }

    public void setRightTag(GoalApriltag rightTag) {
        this.rightTag = rightTag;
    }

    public FeederApriltag getFeederTag() {
        return feederTag;
    }

    public void setFeederTag(FeederApriltag feederTag) {
        this.feederTag = feederTag;
    }

    
}
