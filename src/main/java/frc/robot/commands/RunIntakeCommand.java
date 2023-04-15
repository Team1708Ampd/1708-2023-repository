// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.WristSub;

public class RunIntakeCommand extends CommandBase {
  // Create Intake subsystem reference 
  private IntakeSub intakeSub;

  //  Suppliers
  private BooleanSupplier intakeCmd;
  private BooleanSupplier outtakeCmd;

  private final double INTAKE_SPEED_MAX = 1;
  private final double INTAKE_SPEED_MIN = 0.3;

  // Speed for arm movement
  double speed = INTAKE_SPEED_MAX;

  /** Creates a new ManualArmDown. */
  public RunIntakeCommand(IntakeSub in, BooleanSupplier intake, BooleanSupplier outtake) {
    
    // Get the Intake
    intakeSub = in;

    // Get the suppliers
    intakeCmd = intake;
    outtakeCmd = outtake;

    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (outtakeCmd.getAsBoolean() == true)
    {
      intakeSub.setIntake(-speed);
    }
    else if (intakeCmd.getAsBoolean() == true)
    {
      intakeSub.setIntake(speed);
    }    
    else
    {
      intakeSub.setIntake(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setIntake(0);
  }

  // Returns true when the command should end.
  @Override

  public boolean isFinished() {
    return false;
  }
}
