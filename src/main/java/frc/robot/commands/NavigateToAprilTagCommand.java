package frc.robot.commands;

import java.time.Year;
import java.util.stream.Stream;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSub;
import frc.robot.subsystems.DriveSub;

public class NavigateToAprilTagCommand extends CommandBase{

    // Camera 
    CameraSub camera;

    // Drive 
    DriveSub drive;

    // April tag we are navigating to
    AprilTag navTag;

    // PID controller for strafing
    PIDController strafeController = new PIDController(0.25, 0, 0);

    // PID Controller for range
    PIDController rangeController = new PIDController(0.25, 0, 0);


    double MIN_MOTOR_EFFORT = 0.05;

    public NavigateToAprilTagCommand(CameraSub cam, DriveSub drv, AprilTag tag)
    {
        camera = cam;
        drive = drv;
        navTag = tag;

        addRequirements(cam);
        addRequirements(drv);
       

        // Set the vision pipeline to use for the tag. Should correspond to the tag ID
        camera.SetLimelight1Pipeline(tag.ID);
    }

    @Override
    public void initialize()
    {
        // Set the setpoint and tolerance for each PID Controller
        strafeController.setSetpoint(0); // TODO figure out setpoint
        strafeController.setTolerance(0.1); // TODO figure out tolerance

        rangeController.setSetpoint(0); // TODO figure out setpoint
        rangeController.setTolerance(0.1); // TODO figure out tolerance
    }

    @Override
    public void execute()
    {
        // First we get the current error with distance from the April tag
        NetworkTable lime = camera.GetLimelight1();

        SmartDashboard.putNumber("Vision State", lime.getEntry("tv").getDouble(0));

        // check to see if we have the target in vision
        if (lime.getEntry("tv").getDouble(0) == 1)
        {
            double xError = lime.getEntry("tx").getDouble(0);
            double yError = lime.getEntry("ty").getDouble(0);

            double thetaError = -drive.getGyroscopeRotation().getRadians();
            

            // Now calculate the next state of the drive output from both PID Controllers
            double strafeV = modifyAxis(strafeController.calculate(xError));
            double rangeV = modifyAxis(strafeController.calculate(yError));
            //double turn = modifyAxis(thetaError * 1.5);

            SmartDashboard.putNumber("Lime X Error", strafeV);
            SmartDashboard.putNumber("Lime Y Error", rangeV);
            //SmartDashboard.putNumber("Rotation Error", turn);

            
                      

            drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(                
                rangeV,
                strafeV,
                0,
                drive.getGyroscopeRotation()
            ));
        }
    }

    private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
            if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
            } else {
            return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
    
    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    @Override
    public boolean isFinished()
    {
        boolean rc = false;

        // if (rangeController.atSetpoint() && strafeController.atSetpoint())
        // {
        //     rc = true;
        // }

        return rc;
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.stopBrake();
    }
    
}
