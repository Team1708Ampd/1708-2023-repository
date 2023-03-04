// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CameraConstants;
import frc.robot.Robot;

public class HighConeCamera extends CommandBase {
  /** Creates a new HighConeCamera. */
  private NetworkTable table;
  private double cameraHeight;
  public HighConeCamera(NetworkTable table, double cameraHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.table = table;
    this.cameraHeight = cameraHeight;
    addRequirements(Robot.cameraSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table.getEntry("pipeline").setDouble(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = Robot.cameraSub.estimateDistance(table, cameraHeight, CameraConstants.HIGH_CONE_TAPE_HEIGHT_INCHES);
    System.out.println("Distance in meters to High cone: " + distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
