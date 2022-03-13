// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class TurnCommand extends CommandBase {
  Drivetrain drivetrain;

  double startAngle;
  double targetAngle;
  double angularVelocity;
  double angleDriven;
  double angleDiff;
  ChassisSpeeds speeds;
  int blinkCounter = 0;

  /** Creates a new TurnCommand. */
  public TurnCommand(Drivetrain drivetrain_, double targetAngle_, double angularVelocity_) {
    drivetrain = drivetrain_;
    targetAngle = targetAngle_;
    angularVelocity = angularVelocity_;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = drivetrain.getPoseMeters().getRotation().getRadians();
    speeds = new ChassisSpeeds();
    drivetrain.setChassisSpeeds(speeds);
    blinkCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleDriven = drivetrain.getPoseMeters().getRotation().getRadians() - startAngle;
    angleDiff = targetAngle - angleDriven;

    speeds.vxMetersPerSecond = 0;
    speeds.vyMetersPerSecond = 0;
    speeds.omegaRadiansPerSecond = Math.signum(targetAngle) * angularVelocity;
    drivetrain.setChassisSpeeds(speeds);

    SmartDashboard.putNumber("[DF_T] Target angle (radians)", targetAngle);
    SmartDashboard.putNumber("[DF_T] Current angle (radians)", angleDriven);
    SmartDashboard.putNumber("[DF_T] Angle remaining (radians)", angleDiff);

    blinkCounter++;
    if (((int) (blinkCounter / 10)) % 2 == 0) {
      // Robot.getBling().setSlot(1, 255, 0, 255);
    } else {
      // Robot.getBling().setSlot(1, 0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    speeds.vxMetersPerSecond = 0;
    speeds.vyMetersPerSecond = 0;
    speeds.omegaRadiansPerSecond = 0;
    drivetrain.setChassisSpeeds(speeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angleDiff) <= 0.05;
  }
}
