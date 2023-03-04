// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSub extends SubsystemBase {
  
  /** Creates a new CameraSub. */
  public CameraSub() {}

  public double estimateDistance(NetworkTable table, double cameraHeight, double targetHeight) {
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    double angleToGoalRadians = targetOffsetAngle_Vertical * (3.14159 / 180.0);

    double distanceInches = (targetHeight - cameraHeight) / Math.tan(angleToGoalRadians);
    return distanceInches * 0.0254;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
