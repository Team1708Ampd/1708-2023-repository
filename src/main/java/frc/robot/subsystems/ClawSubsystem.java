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
  public ClawSubsystem(int sparkDeviceNum, int talonDeviceNum, int encoderNum)
  {
    //Create the Two TalonFx Motors to drive the Arm
    wristMotor = new CANSparkMax(sparkDeviceNum, MotorType.kBrushless);
    intakeMotor = new TalonFX(talonDeviceNum);
    wristEncoder = new CANCoder(encoderNum);

    setMotorNeutralMode(NeutralMode.Brake);  

    /// Initialize to 0 power output. Use direct set to make sure Arm start at 0!
    wristMotor.set(0.0);
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  // Constructor taking device IDs and motor configuration
  public ClawSubsystem(int sparkDeviceNum, int talonDeviceNum, int encoderNum, TalonFXConfiguration motorConfig) {
      
    //Components / Notes:
    

    //Create the Two TalonFx Motors to drive the wrist
    wristMotor = new CANSparkMax(sparkDeviceNum, MotorType.kBrushless);
    intakeMotor = new TalonFX(talonDeviceNum);
    wristEncoder = new CANCoder(encoderNum);

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

    // Update the current position of the Arm
    wristPositionCurrent = Rotation2d.fromRadians(getWristAngle());
  }

  // Allow user to pass in Talon configuration
  public ClawSubsystem withTalonConfig(TalonFXConfiguration motorConfig)
  {
    // Update the config
    useMotorConfig(motorConfig);
    return this;
  }  

  // Allow user to pass in the Encoder configuration
  public ClawSubsystem withEncoderConfiguration(CANCoderConfiguration cancoderConfig)
  {
    useEncoderConfig(cancoderConfig);
    return this;
  }

  // Allow user to specify PID constants for use in PID controller
  public ClawSubsystem withPIDConstants(double kP, double kI, double kD)
  {
    PIDkP = kP;
    PIDkI = kI;
    PIDkD = kD;
    return this;
  }

  // Allow user to specify Feed Forward constants for use in FF controller
  public ClawSubsystem withFeedForwardConstants(double kV, double kG)
  {
    FFkG = kG;
    FFkV = kV;
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

  // Get the current position of the encoder attached to the arm 
  public double getWristAngle() {
    int ATTEMPTS = 3;
    double angle = Math.toRadians(wristEncoder.getAbsolutePosition());

    ErrorCode code = wristEncoder.getLastError();

    for (int i = 0; i < ATTEMPTS; i++) {
        if (code == ErrorCode.OK) break;
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) { }
        angle = Math.toRadians(wristEncoder.getAbsolutePosition());
        code = wristEncoder.getLastError();
    }    

    CtreUtils.checkCtreError(code, "Failed to retrieve CANcoder "+wristEncoder.getDeviceID()+" absolute position after "+ATTEMPTS+" tries");

    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
        angle += 2.0 * Math.PI;
    }

    return angle;
  }

  // Get the current Wrist Angle Reported internally to the Arm
  public Rotation2d getWristAngleInternal()
  {
    return wristPositionCurrent;
  }

  // Get the PID Constants
  public double getPIDkP()
  {
    return PIDkP;
  }

  // Get the PID Constants
  public double getPIDkI()
  {
    return PIDkI;
  }

  // Get the PID Constants
  public double getPIDkD()
  {
    return PIDkD;
  }

  // Get the FF Constants
  public double getFFkG()
  {
    return FFkG;
  }

  // Get the FF Constants
  public double getFFkV()
  {
    return FFkV;
  }

  private void useMotorConfig(TalonFXConfiguration config)
  {
      CtreUtils.checkCtreError(intakeMotor.configAllSettings(config), "Failed to configure Falcon 500: intakeMotor");
  }

  private void useEncoderConfig(CANCoderConfiguration cancoderConfig)
  {
      // Init the Cancoder config
      CtreUtils.checkCtreError(wristEncoder.configAllSettings(cancoderConfig, 250), "Failed to configure CANCoder");
      CtreUtils.checkCtreError(wristEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250), "Failed to configure CANCoder update rate");
  }

  // Sets the behavior for neutral position of the motors
  private void setMotorNeutralMode(NeutralMode mode)
  {
    intakeMotor.setNeutralMode(mode);
  }
}