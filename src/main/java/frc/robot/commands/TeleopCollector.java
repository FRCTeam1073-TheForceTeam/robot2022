// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.OI;

public class TeleopCollector extends CommandBase 
{
  Collector collector;
  private double collectorVelocity;
  private double collectorPosition;

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
    if (OI.driverController.getAButtonPressed())
    {
      collectorPosition = 0.5;
    }
    else if (OI.driverController.getBButtonPressed())
    {
      collectorPosition = 1;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
