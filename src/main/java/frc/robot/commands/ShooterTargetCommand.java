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
    new InterpolatorTableEntry(0.0, 320),
    new InterpolatorTableEntry(0.5, 350),
    new InterpolatorTableEntry(1.0, 350),
    new InterpolatorTableEntry(2.0, 370),
    new InterpolatorTableEntry(3.0, 387),
    new InterpolatorTableEntry(3.5, 407),
    new InterpolatorTableEntry(4.0, 418),
    new InterpolatorTableEntry(5.0, 460)
  );
  // new InterpolatorTableEntry(0.0, 320),
  // new InterpolatorTableEntry(1.0, 350),
  // new InterpolatorTableEntry(2.0, 370),
  // new InterpolatorTableEntry(3.0, 400),
  // new InterpolatorTableEntry(4.0, 437),
  // new InterpolatorTableEntry(5.0, 465)
  
  // // new InterpolatorTableEntry(0.5, 350),
  // // new InterpolatorTableEntry(1.0, 335),
  // // new InterpolatorTableEntry(2.0, 375),
  // // new InterpolatorTableEntry(3.0, 387),
  // // new InterpolatorTableEntry(4.0, 418)
  
  public static InterpolatorTable hoodTable = new InterpolatorTable(
    new InterpolatorTableEntry(0.0, 0.08),
    new InterpolatorTableEntry(0.5, 0.16),
    new InterpolatorTableEntry(1.0, 0.21),
    new InterpolatorTableEntry(2.0, 0.26),
    new InterpolatorTableEntry(3.0, 0.338),
    new InterpolatorTableEntry(3.5, 0.36),
    new InterpolatorTableEntry(4.0, 0.41),
    new InterpolatorTableEntry(5.0, 0.456)
  );
  // new InterpolatorTableEntry(0.0, 0.1),
  // new InterpolatorTableEntry(1.0, 0.23),
  // new InterpolatorTableEntry(2.0, 0.34),
  // new InterpolatorTableEntry(3.0, 0.385),
  // new InterpolatorTableEntry(4.0, 0.439),
  // new InterpolatorTableEntry(5.0, 0.470)
  
  // new InterpolatorTableEntry(0.5, 0.16-0.03*0),
  // new InterpolatorTableEntry(1.0, 0.16-0.03*0),
  // new InterpolatorTableEntry(2.0, 0.26-0.03*0),
  // new InterpolatorTableEntry(3.0, 0.338-0.03*0),
  // new InterpolatorTableEntry(4.0, 0.41-0.03*0)
  
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
    targetHoodAngle = hoodTable.getValue(range) + Shooter.additionalHoodAngle;
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
