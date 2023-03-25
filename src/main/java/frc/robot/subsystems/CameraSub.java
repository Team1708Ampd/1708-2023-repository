// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSub extends SubsystemBase {
  
  // Camera Server Instance
  MjpegServer camServer;
  // Limelights
  NetworkTable limelight_one;
  NetworkTable limelight_two;

  /** Creates a new CameraSub. */
  public CameraSub() {
    limelight_one = NetworkTableInstance.getDefault().getTable("limelight-one");
    limelight_two = NetworkTableInstance.getDefault().getTable("limelight-two");

    // Set Limelights to camera mode
    limelight_one.getEntry("camMode").setNumber(1);
    limelight_one.getEntry("stream").setNumber(0);

    // Forward ports
    for (int port = 5800; port <= 5805; port++) {
      PortForwarder.add(port, "limelight-one", port);
  }

    CameraServer.addServer("limelight-one");
    // Create the camera server
    camServer = CameraServer.startAutomaticCapture(null);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCameraDistance(NetworkTable table, double limelightHeight, double goalHeight) {
    table.getEntry("pipeline").setDouble(1.0);
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);


    double angleToGoalDegrees = targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    return (goalHeight - limelightHeight) / Math.tan(angleToGoalRadians);
  }
}
