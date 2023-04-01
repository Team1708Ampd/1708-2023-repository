// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.IntakeSub;

public class ResetFOD extends CommandBase {
  // Intake subsystem
  DriveSub drive;

  /** Creates a new InvertIntake. */
  public ResetFOD(DriveSub driveS) {

    drive = driveS;

    addRequirements(driveS);
    // Use addRequirements() here to declare subsystem dependencies.

    drive.zeroGyroscope();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
