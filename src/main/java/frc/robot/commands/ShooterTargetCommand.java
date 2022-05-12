// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.components.InterpolatorTable;
import frc.robot.components.InterpolatorTable.InterpolatorTableEntry;
import frc.robot.subsystems.HubTracking;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.HubTracking.HubData;

public class ShooterTargetCommand extends CommandBase {

  
  public static InterpolatorTable flywheelTable = new InterpolatorTable(
    // 2022-03-28 tables
    new InterpolatorTableEntry(1.0, 325),
    new InterpolatorTableEntry(1.5, 345),
    new InterpolatorTableEntry(2.0, 355),
    new InterpolatorTableEntry(2.5, 365),
    new InterpolatorTableEntry(3.0, 380),
    new InterpolatorTableEntry(3.5, 405),
    new InterpolatorTableEntry(4.0, 430),
    new InterpolatorTableEntry(4.5, 450),
    new InterpolatorTableEntry(5.0, 463),
    new InterpolatorTableEntry(5.5, 485)
  );
  
  public static InterpolatorTable hoodTable = new InterpolatorTable(
    // 2022-03-28 tables
    new InterpolatorTableEntry(1.0, 0.30),
    new InterpolatorTableEntry(1.5, 0.40),
    new InterpolatorTableEntry(2.0, 0.46),
    new InterpolatorTableEntry(2.5, 0.52),
    new InterpolatorTableEntry(3.0, 0.56),
    new InterpolatorTableEntry(3.5, 0.59),
    new InterpolatorTableEntry(4.0, 0.65),
    new InterpolatorTableEntry(4.5, 0.72),
    new InterpolatorTableEntry(5.0, 0.77),
    new InterpolatorTableEntry(5.5, 0.80)
  );
  
  double targetFlywheelVelocity = 0;
  double targetHoodAngle = 0;

  Shooter shooter;
  HubData data;
  double range = 0;

  /** Creates a new ShooterTargetCommand. */
  public ShooterTargetCommand(Shooter shooter_, double range_) {
    shooter = shooter_;
    range = range_;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetFlywheelVelocity = flywheelTable.getValue(range);
    targetHoodAngle = hoodTable.getValue(range);
    shooter.setFlywheelVelocity(targetFlywheelVelocity);
    shooter.setHoodPosition(targetHoodAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooter.setFlywheelVelocity(0);
      shooter.zeroHood();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(shooter.getHoodTargetPosition() - shooter.getHoodPosition()) < Shooter.Constants.kAcceptableHoodPositionError
        && Math.abs(shooter.getFlywheelTargetVelocity() - shooter.getFlywheelVelocity()) < Shooter.Constants.kAcceptableFlywheelVelocityError);
  }
}
