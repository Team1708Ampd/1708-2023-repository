package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.DriveSub;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import javax.xml.xpath.XPathFactory;

public class DriveCommand extends CommandBase {
    private final DriveSub driveSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    // private final SlewRateLimiter velocityXFilter;
    // private final SlewRateLimiter velocityYFilter;

    public DriveCommand(DriveSub driveSub, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {

        this.driveSubsystem = driveSub;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(driveSubsystem);
    }


    @Override
    public void execute() {   

        driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        driveSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}