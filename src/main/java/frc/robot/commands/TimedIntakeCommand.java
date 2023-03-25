// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSub;

public class TimedIntakeCommand extends CommandBase {
    double time;
    double DELAY_TIME = 0.2;

  // Intake subsystem
  IntakeSub intakeSub;
  
  /** Creates a new IntakeCommand. */
  public TimedIntakeCommand(IntakeSub intake, double time) {

    // Get the delay time
    DELAY_TIME = time;

    intakeSub = intake;

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
    intakeSub.setIntake(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setIntake(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      boolean rc = false;

      if (Timer.getFPGATimestamp() > (time + DELAY_TIME))
      {
          rc = true;
      }

      return rc;
  }
}
