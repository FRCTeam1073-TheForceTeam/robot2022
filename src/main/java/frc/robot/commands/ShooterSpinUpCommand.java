// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterSpinUpCommand extends CommandBase {
  Shooter shooter;
  double velocity;
  double hoodAngle;
  /** Creates a new ShooterSpinUpCommand. */
  public ShooterSpinUpCommand(Shooter shooter_, double velocity_, double hoodAngle_) {
    shooter = shooter_;
    velocity = velocity_;
    hoodAngle = hoodAngle_;
    hoodAngle = MathUtil.clamp(hoodAngle, 0, Shooter.maximumHoodAngle);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setHoodPosition(hoodAngle);
    shooter.setFlywheelVelocity(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(shooter.getHoodTargetPosition() - shooter.getHoodPosition()) < Shooter.Constants.kAcceptableHoodPositionError
      && Math.abs(shooter.getFlywheelTargetVelocity() - shooter.getFlywheelVelocity()) < Shooter.Constants.kAcceptableFlywheelVelocityError);
  }
}
