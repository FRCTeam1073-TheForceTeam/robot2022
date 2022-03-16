// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HubTracking;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.HubTracking.HubData;

public class ShooterRangeTargetCommand extends CommandBase {
  Shooter shooter;
  HubTracking hubTracking;
  double range=0;
  HubData data;

  /** Creates a new ShooterRangeTargetCommand. */
  public ShooterRangeTargetCommand(Shooter shooter_, HubTracking hubTracking_) {
    shooter=shooter_;
    hubTracking=hubTracking_;
    data=new HubData();
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("VISIBLE", hubTracking.isHubVisible());
    SmartDashboard.putNumber("RANGE", range);
    if (hubTracking.isHubVisible()) {
      hubTracking.sampleHubData(data);
      range = data.range;
      double targetFlywheelVelocity = ShooterTargetCommand.flywheelTable.getValue(range);
      double targetHoodAngle = ShooterTargetCommand.hoodTable.getValue(range);
      shooter.setFlywheelVelocity(targetFlywheelVelocity);
      shooter.setHoodPosition(targetHoodAngle);
    } else {
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooter.setFlywheelVelocity(0);
      shooter.setHoodPosition(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(shooter.getHoodTargetPosition() - shooter.getHoodPosition()) < Shooter.Constants.kAcceptableHoodPositionError
      && Math.abs(shooter.getFlywheelTargetVelocity() - shooter.getFlywheelVelocity()) < Shooter.Constants.kAcceptableFlywheelVelocityError);
  }
}
