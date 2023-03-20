package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveConstants;
import frc.robot.subsystems.DriveSub;

// Command used to attempt to balance the platform underneath the robot
public class PlatformBalanceCommand extends CommandBase{

    // Drive subsystem for controlling drive
    DriveSub drive;

    // PID Controller for controller the robot
    PIDController controller = new PIDController(0.008, 0, 0);

    // Helper variables
    private double startTime;
    private int balanceCounter;
    private boolean tilting;
    
    // Constructor for specifying the drive
    public PlatformBalanceCommand(DriveSub drv)
    {
        // Get the drive
        drive = drv;
        addRequirements(drv);
    }

    @Override 
    public void initialize()
    {
        startTime = 0;
        balanceCounter = 0;
        tilting = false;

        // Init the PIDController
        controller.reset();
        controller.setSetpoint(0); // ideally we shoot for 0 degrees roll
        controller.setTolerance(2.0); // 2 degrees tolerance puts us on the ramp
        System.out.println("Initialized Balance");  
    }


    @Override
    public void execute()
    {
        double speed = 1;

        if (!tilting && Math.abs(drive.getRoll()) > 10)
        {
            tilting = true;
            startTime = Timer.getFPGATimestamp();    
            System.out.println("At ramp pitch");       
        }
        if (tilting && Timer.getFPGATimestamp() > startTime + 1.35)
        {
            speed = controller.calculate(Math.abs(drive.getRoll()));
            System.out.println("Balancing"); 
        }
        System.out.printf("Robot Roll %f\n", drive.getRoll());
        SmartDashboard.putNumber("Robot", speed);

        ChassisSpeeds cSpeeds = new ChassisSpeeds(speed, 0, 0);
        
        drive.drive(cSpeeds);
    }

    @Override
    public boolean isFinished()
    {
        if (controller.atSetpoint())
        {
            balanceCounter += 1;
        }
        else
        {
            balanceCounter = 0;
        }

        return (balanceCounter > 2);
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println("Finished Balancing"); 
        drive.stopBrake();
    }    
}
