// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Bling;

public class TeleopCollector extends CommandBase 
{
  Collector collector;
  Bling bling;
  private double collectorVelocity=20; 
  private double loweredCollectorPosition = 6.0; 
  private double raisedCollectorPosition = -6.0; 
 
  /** Creates a new TeleopCollector. */
  public TeleopCollector(Collector collector) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    collector.setIntakeVelocity(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { 
    // moves collector up and down
    if (OI.driverController.getAButtonPressed())
    {
      collector.setLiftPosition(raisedCollectorPosition);
    }
    else if (OI.driverController.getBButtonPressed())
    {
      collector.setLiftPosition(loweredCollectorPosition);
    }

    // spins the collector
    if (OI.driverController.getLeftBumperPressed())
    {
      collector.setIntakeVelocity(collectorVelocity);
    }
    else if (OI.driverController.getRightBumperPressed())
    {
      collector.setIntakeVelocity(-collectorVelocity);
    }
    if(OI.driverController.getLeftBumperReleased()||OI.driverController.getRightBumperReleased()){
      collector.setIntakeVelocity(0);      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if (!interrupted)
    {
      collector.setIntakeVelocity(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}