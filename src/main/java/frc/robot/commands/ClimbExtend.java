package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbExtend extends CommandBase
{
    Climber climber;
    private double currentPosition = 0;
    private double targetPosition = 5;
    private double velocityScale = 1.0;
    private double errorTolerance = 0.1;

    public ClimbExtend(Climber climber_)
    {
        climber = climber_;

        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        currentPosition = climber.getSpoolPosition();
        double spoolVelocity = (targetPosition - currentPosition) * velocityScale;
        climber.setSpoolVelocity(spoolVelocity);
    }

    @Override
    public void end(boolean interrupted)
    {
        climber.setSpoolVelocity(0);
    }

    @Override
    public boolean isFinished()
    {
        if (Math.abs(targetPosition - currentPosition) < errorTolerance 
                || Climber.getSensorReading(3) && Climber.getSensorReading(5)
                && !Climber.getSensorReading(2) && !Climber.getSensorReading(4))
        {
            return true;
        }
        return false;
    }
}
