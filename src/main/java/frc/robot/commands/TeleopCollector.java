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
  private double collectorVelocity = 32;
  private double loweredCollectorPosition = 1.5;
  private double raisedCollectorPosition = 0;
  boolean isCollectorDown = false;
 
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
    // moves collector up and down
    if (OI.operatorController.getAButtonPressed()) {
      collector.setLiftPosition(loweredCollectorPosition);
      isCollectorDown=true;
    } else if (OI.operatorController.getAButtonReleased()) {
      collector.setLiftPosition(raisedCollectorPosition);
      isCollectorDown=false;
    }
    drivetrain.getChassisSpeeds(chassisSpeeds);

    if (isCollectorDown) {
      if (OI.operatorController.getLeftTriggerAxis() > 0.5) {
        collector.setLinearIntakeVelocity(-collectorVelocity);
      } else {
        collector.setLinearIntakeVelocity(chassisSpeeds.vxMetersPerSecond + collectorVelocity);
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
