package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSub;

public class WristSetPositionCommand extends CommandBase{

    // PID Controller 
    private PIDController pidController;

    // Arm Rotation Subsystem
    private WristSub armSubsystem;

    // PID constants
    private double kP, kI, kD;

    // Target angle for the command
    private double targetAngle = 0;

    boolean useSuppliedConstants = false;

    boolean debug = false;

    public WristSetPositionCommand(WristSub rotSubsystem, double newAngle, boolean db)
    {
        armSubsystem = rotSubsystem;
        targetAngle = newAngle;
        debug = db;

        if (db)
        {
            SmartDashboard.putNumber("WRISTKP", 0.1);
            SmartDashboard.putNumber("WRISTKI", 0.1);
            SmartDashboard.putNumber("WRISTD", 0.1);
            SmartDashboard.putNumber("WRISTTARGETANGLE", 30);      
            SmartDashboard.putNumber("WRISTCURRENTANGLE", 0);      
        }

        // Add the subsystem as a requirement
        addRequirements(rotSubsystem);        
    }

    @Override
    public void initialize(){

        if (debug)
        {
            kP = SmartDashboard.getNumber("WRISTKP", 0.1);
            kI = SmartDashboard.getNumber("WRISTKI", 0.1);
            kD = SmartDashboard.getNumber("WRISTKD", 0.1);
            targetAngle = SmartDashboard.getNumber("WRISTTARGETANGLE", 30);
        }
        else
        {
            kP = 1.0;
            kI = 0.0;
            kD = 0.0;
        }

        // Init the controllers. PID gets a tolerance
        pidController = new PIDController(kP, kI, kD);
        pidController.disableContinuousInput();
        pidController.setTolerance(4.0);

        // Get the first calculation and set the motors to start moving
        pidController.setSetpoint(targetAngle);

        // First iteration only uses feed forward to do the adjustment. All other iterations will feed back
        double output = (pidController.calculate(targetAngle));

        // Set the output
        armSubsystem.setWrist(output);
    }

    // Execute override
    @Override
    public void execute()
    {
        // Get the current angle of the Arm Subsystem
        double currentAngle = armSubsystem.getWristPosition();

        // Read the current arm angle and feedback
        armSubsystem.setWrist(pidController.calculate(currentAngle));

        if (debug)
        {
            SmartDashboard.putNumber("WRISTCURRENTANGLE", currentAngle);
        }

    }

    // End override
    @Override
    public void end(boolean interrupted)
    {
        // Set the arm output to no power
        armSubsystem.setWrist(0);
    }

    public WristSetPositionCommand withControlConstants(double newkP, double newkI, double newkD)
    {
        useSuppliedConstants = true;
        kP = newkP;
        kI = newkI;
        kD = newkD;
        return this;
    }

    // isfinished override
    @Override
    public boolean isFinished()
    {
        // Finish when the PID tolerance limit is met        
        return pidController.atSetpoint();
    }
    
}
