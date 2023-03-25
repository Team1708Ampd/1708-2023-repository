// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSub extends SubsystemBase {
  /** Creates a new ClawSub. */
  CANSparkMax wristMotor;

  // Requested Motor Output
  double motorOutRequested = 0;

  // Run motor inverted flag
  boolean motorInverted = false;

  // Internal encoder reported position
  double encoderWristPosition = 0;

  // Encoder counts per rev
  double encoderPPR = 42;

  // Relative encoder to get Spark Max Hall Effect Sensor
  RelativeEncoder sparkRelEncoder;

  public WristSub(Integer wristId) {
    wristMotor = new CANSparkMax(wristId, MotorType.kBrushless);
    wristMotor.setIdleMode(IdleMode.kBrake);

    // Init the encoder
    sparkRelEncoder = wristMotor.getEncoder();
      

    // Set the initial speed of the motor
    motorOutRequested = 0;
  }


  public void setWrist(double power) {
    wristMotor.set(power);
  }

  public void stopWrist(){
    wristMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Set the motor to the requested motor output
    if (motorInverted)
    {
      wristMotor.set((motorOutRequested *-1));
    }
    else
    {
      wristMotor.set(motorOutRequested);
    } 

    // Update the motor position from the internal encoder
    encoderWristPosition = sparkRelEncoder.getPosition();
  }
}
