// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardCommand extends CommandBase {
  Drivetrain drivetrain;
  double targetDistance;
  double currentDistance;
  double velocity;
  ChassisSpeeds chassisSpeeds;
  Pose2d startingPose;

  /** Creates a new DriveForwardCommand. */
  public DriveForwardCommand(Drivetrain drivetrain_, double targetDistance_, double velocity_) {
    drivetrain = drivetrain_;
    targetDistance = targetDistance_;
    velocity = velocity_;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDistance = 0;
    chassisSpeeds = new ChassisSpeeds();
    startingPose = drivetrain.getPoseMeters();
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDistance = drivetrain.getPoseMeters().getTranslation().getDistance(startingPose.getTranslation());
    chassisSpeeds.vxMetersPerSecond = velocity;
    chassisSpeeds.vyMetersPerSecond = 0;
    chassisSpeeds.omegaRadiansPerSecond = 0;
    drivetrain.setChassisSpeeds(chassisSpeeds);
    SmartDashboard.putNumber("[DF_C] Target distance (meters)", targetDistance);
    SmartDashboard.putNumber("[DF_C] Current distance (meters)", currentDistance);
    SmartDashboard.putNumber("[DF_C] Distance remaining (meters)", targetDistance - currentDistance);
    SmartDashboard.putNumber("[DF_C] Target velocity (meters per second)", targetDistance - currentDistance);
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
