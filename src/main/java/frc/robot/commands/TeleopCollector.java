// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class TeleopCollector extends CommandBase 
{
  Collector collector;
  Drivetrain drivetrain;
  Bling bling;
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  boolean isCollectorDown = false;

  private double collectorVelocity = 12;
  private double extraCollectorVelocity = 4.0;
 
  /** Creates a new TeleopCollector. */
  public TeleopCollector(Collector collector, Drivetrain drivetrain) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    this.drivetrain = drivetrain;
    bling = Robot.getBling();
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    collector.setIntakeVelocity(0.0);
    chassisSpeeds = new ChassisSpeeds();
    isCollectorDown = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // The collector raises when the A button (cross button on the PS4 controller) and goes back up when it's released.
    // The isCollectorDown variable is updated so the command knows which state it's in.
    if (OI.operatorController.getAButtonPressed()) {
      collector.setLiftPosition(Collector.Constants.loweredCollectorPosition);
      isCollectorDown=true;
    } else if (OI.operatorController.getAButtonReleased()) {
      collector.setLiftPosition(Collector.Constants.raisedCollectorPosition);
      isCollectorDown=false;
    }
    drivetrain.getChassisSpeeds(chassisSpeeds);

    //If the collector is down, run the intake wheels inwards.
    //If it's down AND the left trigger is past 50% (same as with TeleopIndexer), it runs outwards.
    //If it's up, it doesn't run at all.
    if (isCollectorDown) {
      if (OI.operatorController.getLeftTriggerAxis() > 0.5) {
        collector.setLinearIntakeVelocity(-collectorVelocity);
      } else {
        collector.setLinearIntakeVelocity(chassisSpeeds.vxMetersPerSecond * 2.0 + extraCollectorVelocity);
      }
    } else {
      collector.setLinearIntakeVelocity(0);
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
