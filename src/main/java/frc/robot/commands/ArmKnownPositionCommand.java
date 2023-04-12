package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotationSub;
import frc.robot.subsystems.ArmTelescopingSub;
import frc.robot.subsystems.WristSub;

public class ArmKnownPositionCommand extends CommandBase{

    // Subs
    ArmRotationSub armRot;
    ArmTelescopingSub armTele;
    WristSub wrist;
    ARMPOSITION position;

    public ArmKnownPositionCommand(ArmTelescopingSub tele, ArmRotationSub rot, WristSub wr, ARMPOSITION pos)
    {
        armRot = rot;
        armTele = tele;
        wrist = wr;
        position = pos;
    }

    @Override 
    public void initialize()
    {

    }

    @Override
    public void execute(){
        

    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    enum ARMPOSITION
    {
        VACCUUM, 
        CHERRYPICK,
        MID, 
        REVERSEVACCUUM
    }

}
