// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSub;

public class OuttakeAutoCommand extends CommandBase {

  IntakeSub intakeSubsystem;

  /** Creates a new OuttakeCommand. */
  double time;
  public OuttakeAutoCommand(IntakeSub intake) {

    intakeSubsystem = intake;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setIntake(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rc = false;

    if (Timer.getFPGATimestamp() > (time + 1))
    {
      rc = true;
    }

    return rc;
  }
}
