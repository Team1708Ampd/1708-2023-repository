package frc.robot.Objects;

import frc.robot.CameraConstants;

public class RedTags {
    private GoalApriltag leftTag;
    private GoalApriltag middleTag;
    private GoalApriltag rightTag;
    private FeederApriltag feederTag;

    public RedTags() {
        leftTag = new GoalApriltag(3, CameraConstants.CUBE_TAG_HEIGHT_INCHES);
        middleTag = new GoalApriltag(2, CameraConstants.CUBE_TAG_HEIGHT_INCHES);
        rightTag = new GoalApriltag(1, CameraConstants.CUBE_TAG_HEIGHT_INCHES);

        feederTag = new FeederApriltag(5, CameraConstants.FEEDER_STATION_TAG_HEIGHT_INCHES);
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
