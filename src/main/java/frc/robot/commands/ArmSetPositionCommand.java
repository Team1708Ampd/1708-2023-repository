package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotationSub;

public class ArmSetPositionCommand extends CommandBase{

    // Feed forward controller
    private ArmFeedforward ffController;

    // PID Controller 
    private PIDController pidController;

    // Arm Rotation Subsystem
    private ArmRotationSub armSubsystem;

    // PID constants
    private double kP, kI, kD;

    // Feed Forward Constants
    private double kV, kG;

    // Target angle for the command
    private double targetAngle = 0;

    boolean debug = false;


    public ArmSetPositionCommand(ArmRotationSub rotSubsystem, double newAngle, boolean db)
    {
        armSubsystem = rotSubsystem;
        targetAngle = newAngle;
        debug = db;

        if (db)
        {
            SmartDashboard.putNumber("ARMKP", 0.1);
            SmartDashboard.putNumber("ARMKI", 0.1);
            SmartDashboard.putNumber("ARMKD", 0.1);
            SmartDashboard.putNumber("ARMTARGETANGLE", 30);      
            SmartDashboard.putNumber("ARMCURRENTANGLE", 0);      
        }
        
        // Add the subsystem as a requirement
        addRequirements(rotSubsystem);        
    }

    @Override
    public void initialize(){

        if (debug)
        {
            kP = SmartDashboard.getNumber("ARMKP", 0.1);
            kI = SmartDashboard.getNumber("ARMKI", 0.1);
            kD = SmartDashboard.getNumber("ARMKD", 0.1);
            targetAngle = SmartDashboard.getNumber("ARMTARGETANGLE", 30);
        }
        else
        {
            kP = 0.05;
            kI = 0.00;
            kD = 0.00;
        }

        // Init the controllers. PID gets a tolerance
        pidController = new PIDController(kP, kI, kD);
        pidController.disableContinuousInput();
        pidController.setTolerance(Math.toRadians(0.5));


        ffController = new ArmFeedforward(0, kG, kV);

        // Get the first calculation and set the motors to start moving
        pidController.setSetpoint(targetAngle);
        pidController.setTolerance(3.0);

        // First iteration only uses feed forward to do the adjustment. All other iterations will feed back
        double output = (ffController.calculate(targetAngle, 1, 1) +
                         pidController.calculate(targetAngle));

        // Set the output
        armSubsystem.setArmOutput(output);
    }

    // Execute override
    @Override
    public void execute()
    {
        // Get the current angle of the Arm Subsystem
        double currentAngle = armSubsystem.getArmAngle();

        // Read the current arm angle and feedback
        armSubsystem.setArmOutput(pidController.calculate(currentAngle));

        if (debug)
        {
            SmartDashboard.putNumber("ARMCURRENTANGLE", currentAngle);
        }
    }

    // End override
    @Override
    public void end(boolean interrupted)
    {
        // Set the arm output to no power
        armSubsystem.setArmOutput(0);
    }

    public ArmSetPositionCommand withControlConstants(double newkP, double newkI, double newkD, double newffV, double newffG)
    {
        kP = newkP;
        kI = newkI;
        kD = newkD;
        kV = newffV;
        kG = newffG;
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
