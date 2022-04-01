package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbRetract extends CommandBase
{
    Climber climber;
    class ControlData 
    {
        public double currentPosition = 5;
        public double targetPosition = 0;
        public double velocityScale = 1.0;
        public double errorTolerance = 0.1;
    }

    private ControlData spoolData;
    private ControlData shoulderData;


    public ClimbRetract(Climber climber_)
    {
        climber = climber_;
        spoolData = new ControlData();
        shoulderData = new ControlData();
        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        spoolData.currentPosition = climber.getSpoolPosition();
        double spoolVelocity = (spoolData.targetPosition - spoolData.currentPosition) * spoolData.velocityScale;
        climber.setSpoolVelocity(spoolVelocity);

        shoulderData.currentPosition = climber.getExtensionPosition();
        double shoulderVelocity = (shoulderData.targetPosition - shoulderData.currentPosition) * shoulderData.velocityScale;
        climber.setExtensionVelocity(shoulderVelocity);
    }

    @Override
    public void end(boolean interrupted)
    {
        climber.setSpoolVelocity(0);
        climber.setExtensionVelocity(0);
    }
    @Override
    public boolean isFinished()
    {
        if (Math.abs(spoolData.targetPosition - spoolData.currentPosition) < spoolData.errorTolerance )
        {
            return true;
        }
        return false;
    }
}
