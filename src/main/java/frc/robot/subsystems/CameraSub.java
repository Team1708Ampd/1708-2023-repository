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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoManager.TeamColor;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
public class CameraSub extends SubsystemBase {
  
  // Camera Server Instance
  MjpegServer camServer;
  // Limelights
  NetworkTable limelight_one;
  NetworkTable limelight_two;

  // Internal reference to the driver sub so that the vision pose estimations
  // can be used to update the robot pose
  DriveSub drive;

  // 2023 Charged Season Field Layout
  AprilTagFieldLayout field;


  /** Creates a new CameraSub. */
  public CameraSub(DriveSub driverSub, TeamColor alliance) {
    limelight_one = NetworkTableInstance.getDefault().getTable("limelight-one");
    limelight_two = NetworkTableInstance.getDefault().getTable("limelight-two");

    // Set Limelights to camera mode
    limelight_one.getEntry("camMode").setNumber(1);
    limelight_one.getEntry("stream").setNumber(0);

    // Forward ports
    for (int port = 5800; port <= 5805; port++) {
      PortForwarder.add(port, "limelight-one", port);
    }

    try{
      // Load the field 
      field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      if (alliance == TeamColor.BLUE)
      {
        field.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
      }
      else
      {
        field.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
      }

    }
    catch(IOException exc)
    {
      DriverStation.reportError("Oh no the field layout didnt load, so sad D: ", true);
    }

    drive = driverSub;

    // Set up for 
    limelight_one.getEntry("pipeline").setNumber(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    



  }

  public NetworkTable GetLimelight1()
  {
    return limelight_one;
  }

  public NetworkTable GetLimelight2()
  {
    return limelight_two;
  }

  public void SetLimelight1Pipeline(Number pID)
  {
    limelight_one.getEntry("pipeline").setNumber(pID);
  }

  public void SetLimelight2Pipeline(Number pID)
  {
    limelight_two.getEntry("pipeline").setNumber(pID);
  }

  public AprilTag GetAprilTagFromID(int id)
  {
    return field.getTags().get(id);
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
