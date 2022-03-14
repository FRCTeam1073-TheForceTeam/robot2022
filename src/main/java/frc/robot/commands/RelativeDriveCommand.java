// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RelativeDriveCommand extends CommandBase {

  Pose2d robotPose;
  Vector2d x_robot = new Vector2d();
  Vector2d y_robot = new Vector2d();

  Pose2d targetPose;
  Vector2d x_target = new Vector2d();
  Vector2d y_target = new Vector2d();

  Vector2d offsetVector = new Vector2d();

  Drivetrain drivetrain;

  ChassisSpeeds chassisSpeeds;

  double maxVelocity = 2;
  double maxAngularVelocity = 5;

  double velocityScale = 1;
  double rotationScale = 1;

  private final double forwardTolerance = 0.05;
  private final double directionTolerance = 0.005;
  private final double distanceTolerance = 0.1;

  double distance;

  /** Creates a new RelativeDriveCommand. */
  public RelativeDriveCommand(Drivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = 0;
    double targetAngle = targetPose.getRotation().getRadians();
    x_target.x = Math.cos(targetAngle);
    x_target.y = Math.sin(targetAngle);
    y_target.x = Math.cos(targetAngle + Math.PI * 0.5);
    y_target.y = Math.sin(targetAngle + Math.PI * 0.5);

    chassisSpeeds = new ChassisSpeeds();
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = drivetrain.getPoseMeters();
    distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
    offsetVector.x = targetPose.getTranslation().minus(robotPose.getTranslation()).getX() / distance;
    offsetVector.y = targetPose.getTranslation().minus(robotPose.getTranslation()).getY() / distance;
    double robotAngle = robotPose.getRotation().getRadians();

    x_robot.x = Math.cos(robotAngle);
    x_robot.y = Math.sin(robotAngle);
    y_robot.x = Math.cos(robotAngle + Math.PI * 0.5);
    y_robot.y = Math.sin(robotAngle + Math.PI * 0.5);

    double targetVelocity = maxVelocity * velocityScale * rangeFunction() * directionFunction();
    double targetRotation = maxAngularVelocity * rotationScale * angleFunction();
    chassisSpeeds.vxMetersPerSecond = targetVelocity;
    chassisSpeeds.omegaRadiansPerSecond = targetRotation;
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  double rangeFunction() {
    return 1.0 - Math.exp(-2.0 * distance);
  }
  
  double directionFunction() {
    return Math.max(0, x_robot.dot(x_target));
  }

  double angleFunction() {
    double orientationValue = x_robot.dot(y_target);
    double directionValue = x_robot.dot(offsetVector);
    return 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds = new ChassisSpeeds();
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double forwardValue = x_robot.dot(x_target);
    double rotationValue = x_robot.dot(y_target);
    return ((1-forwardValue)<forwardTolerance)&&(Math.abs(rotationValue)<directionTolerance)&&(distance<distanceTolerance);
  }
}
