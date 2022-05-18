// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Shooter;

public class TeleopShooter extends CommandBase {

  private Shooter shooter;
  
  /** Creates a new TeleopShooter. */
  public TeleopShooter(Shooter shooter) {
    this.shooter = shooter;

    SmartDashboard.putNumber("[T-Sh] Flywheel vel (rad.s)", 0);
    SmartDashboard.putNumber("[T-Sh] Hood angle (rads)", 0);
    SmartDashboard.putBoolean("[T-Sh] Update", false);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("[T-Sh] Flywheel vel (rad.s)", 0);
    SmartDashboard.putNumber("[T-Sh] Hood angle (rads)", 0);
    SmartDashboard.putBoolean("[T-Sh] Update", false);
    // shooter.setFlywheelVelocity(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(SmartDashboard.getBoolean("[T-Sh] Update", false)) {
      shooter.setFlywheelVelocity(SmartDashboard.getNumber("[T-Sh] Flywheel vel (rad.s)", 0));
      shooter.setHoodPosition(SmartDashboard.getNumber("[T-Sh] Hood angle (rads)", 0));
      SmartDashboard.putBoolean("[T-Sh] Update", false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}