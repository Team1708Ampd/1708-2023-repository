package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.swervelib.ctre.*;

import static frc.robot.Constants.*;

public class ClawSubsystem extends SubsystemBase {
    
  private CANSparkMax wristMotor;
  private TalonFX intakeMotor;
  private CANCoder wristEncoder;

  // PID configuration
  private double PIDkP = 0.0;
  private double PIDkI = 0.0;
  private double PIDkD = 0.0;

  // Feed forward configuration
  private double FFkG = 0;
  private double FFkV = 0;

  // Output power on the arm being requested
  private double wristOutRequested = 0;

  // Output power on the intake being requested
  private double intakeOutRequested = 0;

  // Current position of the arm in degrees
  private Rotation2d wristPositionCurrent;

  // Define rotation limits for the rotation of the arm
  private final double ROTATION_LIMIT_START = 0.0;
  private final double ROTATION_LIMIT_END = 270.0;


  // Constructor just taking motor and encoder IDS
  public ClawSubsystem(int sparkDeviceNum, int talonDeviceNum)
  {
    //Create the Two TalonFx Motors to drive the Arm
    wristMotor = new CANSparkMax(sparkDeviceNum, MotorType.kBrushless);
    intakeMotor = new TalonFX(talonDeviceNum);

    setMotorNeutralMode(NeutralMode.Brake);  

    /// Initialize to 0 power output. Use direct set to make sure Arm start at 0!
    wristMotor.set(0.0);
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  // Constructor taking device IDs and motor configuration
  public ClawSubsystem(int sparkDeviceNum, int talonDeviceNum, TalonFXConfiguration motorConfig) {
      
    //Components / Notes:
    

    //Create the Two TalonFx Motors to drive the wrist
    wristMotor = new CANSparkMax(sparkDeviceNum, MotorType.kBrushless);
    intakeMotor = new TalonFX(talonDeviceNum);

    setMotorNeutralMode(NeutralMode.Brake);    
    
    /// Initialize to 0 power output. Use direct set to make sure Arm start at 0!
    wristMotor.set(0.0);
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {    
    // Update the motor outputs to the current desired output value
    wristMotor.set(wristOutRequested);
    intakeMotor.set(TalonFXControlMode.PercentOutput, intakeOutRequested);
  }

  // Allow user to pass in Talon configuration
  public ClawSubsystem withTalonConfig(TalonFXConfiguration motorConfig)
  {
    // Update the config
    useMotorConfig(motorConfig);
    return this;
  }  

  // Sets the Wrist motor outputs
  public void setWristOutput(double power) {
    // Set the requested output power of the motor
    wristOutRequested = power;
  }

  // Sets the Intake motor outputs
  public void setIntakeOutput(double power) {
    // Set the requested output power of the motor
    intakeOutRequested = power;
  }

  private void useMotorConfig(TalonFXConfiguration config)
  {
      CtreUtils.checkCtreError(intakeMotor.configAllSettings(config), "Failed to configure Falcon 500: intakeMotor");
  }

  // Sets the behavior for neutral position of the motors
  private void setMotorNeutralMode(NeutralMode mode)
  {
    intakeMotor.setNeutralMode(mode);
  }
}