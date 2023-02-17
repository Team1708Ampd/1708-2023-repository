package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

//TODO Remove un-needed libraries
import frc.robot.swervelib.Mk4iSwerveModuleHelper;
import frc.robot.swervelib.SdsModuleConfigurations;
import frc.robot.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.robot.swervelib.DriveController;
import frc.robot.swervelib.DriveControllerFactory;
import frc.robot.swervelib.ModuleConfiguration;
import frc.robot.swervelib.ctre.*;

import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
    
    public ArmSubsystem(Integer talonDeviceNum1, Integer talonDeviceNum2) {
        
        //Components / Notes:
        // Two Falcon Fx Talon 500 that work together to drive the Arm angle. 
        // use like this: TalonFX motor = new TalonFX(driveConfiguration, "Hannibal the CANibal");
        // Will have encoder at top of arm bracket to tell position
        // Want to use PID controller: ref controller = motor.getPIDController(); (?)

        //Create the Two TalonFx Motors to drive the Arm
        TalonFX armMotor1 = new TalonFX(talonDeviceNum1, "ArmMotor1");
        TalonFX armMotor2 = new TalonFX(talonDeviceNum2, "ArmMotor2");

        //Create the config for the motors. Each are equally matched here. Defaults taken from documentation
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.supplyCurrLimit.enable = true;
        armConfig.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        armConfig.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        armConfig.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
        
        //Config the Current Limits for the motors 
        CtreUtils.checkCtreError(armMotor1.configAllSettings(armConfig), "Failed to configure Falcon 500: ArmMotor1");
        CtreUtils.checkCtreError(armMotor2.configAllSettings(armConfig), "Failed to configure Falcon 500: ArmMotor2");

        //Config the power (percent) output for the motors (default from docs)
        armMotor1.set(TalonFXControlMode.PercentOutput, 0.5); // runs the motor at 50% power
        armMotor2.set(TalonFXControlMode.PercentOutput, 0.5); // runs the motor at 50% power


    }

    @Override
    public void periodic() {



    }

    //Method to print out debug info for the two Arm motors
    public void debugMotors() {

        // System.out.println(armMotor1.GetSelectedSensorPosition);
        // std::cout << armMotor1.GetSelectedSensorPosition() << std::endl; // prints the position of the selected sensor
        // std::cout << motor.GetSelectedSensorVelocity() << std::endl; // prints the velocity recorded by the selected sensor
        // std::cout << motor.GetMotorOutputPercent() << std::endl; // prints the percent output of the motor (0.5)
        // std::cout << motor.GetStatorCurrent() << std::endl; // prints the output current of the motor
    }


}