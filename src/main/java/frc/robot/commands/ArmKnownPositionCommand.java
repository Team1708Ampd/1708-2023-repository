package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmRotationSub;
import frc.robot.subsystems.ArmTelescopingSub;
import frc.robot.subsystems.WristSub;

public class ArmKnownPositionCommand{

    // Subs
    ArmRotationSub armRot;
    ArmTelescopingSub armTele;
    WristSub wrist;

    public ArmKnownPositionCommand(ArmTelescopingSub tele, ArmRotationSub rot, WristSub wr)
    {
        armRot = rot;
        armTele = tele;
        wrist = wr;
    }

    public Command getCommand(ARMPOSITION pos)
    {
        Command rc = null;

        switch(pos)
        {
            case SCOREHIGH:
                rc = new ParallelCommandGroup(new ArmSetPositionCommand(armRot, 120, false),
                                              new WristSetPositionCommand(wrist, 0, false));
            break;

            case SCOREMID:
            rc = new ParallelCommandGroup(new ArmSetPositionCommand(armRot, 138, false),
                                          new WristSetPositionCommand(wrist, 11, false));

            break;

            case SCORELOW:
            rc = new ParallelCommandGroup(new ArmSetPositionCommand(armRot, 209, false),
                                          new WristSetPositionCommand(wrist, 11, false));
            break;

            case INTAKEGROUND:
            rc = new ParallelCommandGroup(new ArmSetPositionCommand(armRot, 30, false),
                                          new WristSetPositionCommand(wrist, 130, false));
            break;

            case INTAKEFEEDER:
             rc = new ParallelCommandGroup(new ArmSetPositionCommand(armRot, 30, false),
                                          new WristSetPositionCommand(wrist, 130, false));
            break;

            case SCOREMIDCONE:
             rc = new ParallelCommandGroup(new ArmSetPositionCommand(armRot, 30, false),
                                          new WristSetPositionCommand(wrist, 130, false));
            break;

            case SCOREHIGHCONE:
             rc = new ParallelCommandGroup(new ArmSetPositionCommand(armRot, 30, false),
                                          new WristSetPositionCommand(wrist, 130, false));
            break;
        }

        return rc;
    }


    public enum ARMPOSITION
    {
        SCOREHIGH, 
        SCOREMID,
        SCORELOW, 
        INTAKEFEEDER,
        INTAKEGROUND,
        SCOREMIDCONE,
        SCOREHIGHCONE
    }

}
