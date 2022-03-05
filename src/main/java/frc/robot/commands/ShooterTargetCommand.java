// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.components.InterpolatorTable;
import frc.robot.components.InterpolatorTable.InterpolatorTableEntry;
import frc.robot.subsystems.HubTracking;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.HubTracking.HubData;

public class ShooterTargetCommand extends CommandBase {

  HubTracking hubTracking;
  Shooter shooter;
  HubData data;
  double range = 0;
  boolean dataCollected = false;

  InterpolatorTable flywheelTable = new InterpolatorTable(
    new InterpolatorTableEntry(0.0, 320),
    new InterpolatorTableEntry(0.5, 350),
    new InterpolatorTableEntry(1.0, 350),
    new InterpolatorTableEntry(2.0, 370),
    new InterpolatorTableEntry(3.0, 387),
    new InterpolatorTableEntry(4.0, 418)
  );

  InterpolatorTable hoodTable = new InterpolatorTable(
    new InterpolatorTableEntry(0.0, 0.08),
    new InterpolatorTableEntry(0.5, 0.16),
    new InterpolatorTableEntry(1.0, 0.21),
    new InterpolatorTableEntry(2.0, 0.26),
    new InterpolatorTableEntry(3.0, 0.338),
    new InterpolatorTableEntry(4.0, 0.41)
  );

  double targetFlywheelVelocity = 0;
  double targetHoodAngle = 0;

  boolean overrideHubTracking;
  double overrideDistance;

  /** Creates a new ShooterTargetCommand. */
  public ShooterTargetCommand(Shooter shooter_, HubTracking hubTracking_, boolean overrideHubTracking_, double overrideDistance_) {
    hubTracking = hubTracking_;
    shooter = shooter_;
    overrideHubTracking = overrideHubTracking_;
    overrideDistance = overrideDistance_;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    range = 0;
    targetFlywheelVelocity = 0;
    targetHoodAngle = 0;
    dataCollected = false;
    if (overrideHubTracking) {
      range = overrideDistance;
      targetFlywheelVelocity = flywheelTable.getValue(range);
      targetHoodAngle = hoodTable.getValue(range);
      shooter.setFlywheelVelocity(targetFlywheelVelocity);
      shooter.setHoodPosition(targetHoodAngle);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumberArray("[S-TC] Target vel, Target angle",new Double[]{targetFlywheelVelocity, targetHoodAngle});
    if (!overrideHubTracking && !dataCollected) {
      hubTracking.sampleHubData(data);
      range = data.range;
      if (range > 0) {
        dataCollected = true;
        targetFlywheelVelocity = flywheelTable.getValue(range);
        targetHoodAngle = hoodTable.getValue(range);
        shooter.setFlywheelVelocity(targetFlywheelVelocity);
        shooter.setHoodPosition(targetHoodAngle);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetHoodAngle - shooter.getHoodPosition()) <= Shooter.Constants.ACCEPTABLE_HOOD_ERROR
        && Math.abs(targetFlywheelVelocity - shooter.getFlywheelVelocity()) <= Shooter.Constants.ACCEPTABLE_FLYWHEEL_ERROR;
  }
}
