// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriveTranslateCommand extends CommandBase {
  Drivetrain drivetrain;
  double targetDistance;
  double currentDistance;
  double velocityScale;
  ChassisSpeeds chassisSpeeds;
  Pose2d startingPose;
  int blinkCounter = 0;

  /** Creates a new DriveForwardCommand. */
  public DriveTranslateCommand(Drivetrain drivetrain_, double targetDistance_, double velocityScale_) {
    drivetrain = drivetrain_;
    targetDistance = targetDistance_;
    velocityScale = velocityScale_;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDistance = 0;
    chassisSpeeds = new ChassisSpeeds();
    startingPose = drivetrain.getPoseMeters();
    drivetrain.setChassisSpeeds(chassisSpeeds);
    blinkCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDistance = drivetrain.getPoseMeters().getTranslation().getDistance(startingPose.getTranslation());
    chassisSpeeds.vxMetersPerSecond = Math.signum(targetDistance - currentDistance) * velocityScale;
    chassisSpeeds.vyMetersPerSecond = 0;
    chassisSpeeds.omegaRadiansPerSecond = 0;
    drivetrain.setChassisSpeeds(chassisSpeeds);
    SmartDashboard.putNumber("[DF_C] Target distance (meters)", targetDistance);
    SmartDashboard.putNumber("[DF_C] Current distance (meters)", currentDistance);
    SmartDashboard.putNumber("[DF_C] Distance remaining (meters)", targetDistance - currentDistance);
    SmartDashboard.putNumber("[DF_C] Target velocity (meters per second)", targetDistance - currentDistance);
    blinkCounter++;
    if (((int) (blinkCounter / 10)) % 2 == 0) {
      Robot.getBling().setSlot(1, 255, 0, 255);
    } else {
      Robot.getBling().setSlot(1, 0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds.vxMetersPerSecond = 0;
    chassisSpeeds.vyMetersPerSecond = 0;
    chassisSpeeds.omegaRadiansPerSecond = 0;
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentDistance >= targetDistance;
  }
}
