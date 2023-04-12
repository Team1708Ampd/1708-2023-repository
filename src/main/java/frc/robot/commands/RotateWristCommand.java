// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.WristSub;

public class RotateWristCommand extends CommandBase {
  // Create Wrist subsystem reference 
  private WristSub wristSub;

  //  Suppliers
  private BooleanSupplier wristUpCmd;
  private BooleanSupplier wristDownCmd;

  private final double WRIST_SPEED_MAX = 0.5;
  private final double WRIST_SPEED_MIN = 0.3;

  // Speed for arm movement
  double speed = WRIST_SPEED_MAX;

  /** Creates a new ManualArmDown. */
  public RotateWristCommand(WristSub wrist, BooleanSupplier upCmd, BooleanSupplier downCmd) {
    
    // Get the wrist
    wristSub = wrist;

    // Get the suppliers
    wristUpCmd = upCmd;
    wristDownCmd = downCmd;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (wristDownCmd.getAsBoolean() == true)
    {
      wristSub.setWrist(-speed);
    }
    else if (wristUpCmd.getAsBoolean() == true)
    {
      wristSub.setWrist(speed);
    }    
    else
    {
      wristSub.setWrist(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSub.setWrist(0);
  }

  // Returns true when the command should end.
  @Override

  public boolean isFinished() {
    return false;
  }
}
