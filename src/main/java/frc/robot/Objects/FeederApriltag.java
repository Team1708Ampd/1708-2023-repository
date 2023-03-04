package frc.robot.Objects;

public class FeederApriltag {
    private double tagId;
    private double tagHeight;

    public FeederApriltag(double tagId, double tagHeight) {
        this.tagId = tagId;
        this.tagHeight = tagHeight;
    }

    public double getTagId() {
        return tagId;
    }

    public void setTagId(double id) {
        this.tagId = id;
    }

    public double getTagHeightInches() {
        return tagHeight;
    }

    public double getTagHeightMeters() {
        return tagHeight * 0.0254;
    }

    public void setTagHeightInches(double height) {
        this.tagHeight = height;
    }

    public void setTagHeightMeters(double height) {
         this.tagHeight = height / 0.0254;
    }
}
