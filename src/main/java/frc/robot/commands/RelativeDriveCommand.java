// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  double scaleFactor = 0.3;

  double maxVelocity = 4*scaleFactor;
  double maxAngularVelocity = 14*scaleFactor;

  private final double distanceTolerance = 0.1;

  double distance;

  double alpha = 1;
  double beta = 1.5;
  double gamma = 1.5;

  /** Creates a new RelativeDriveCommand. */
  public RelativeDriveCommand(Drivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    SmartDashboard.putString("RelDrive/Status", "[NOT STARTED]");
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

    SmartDashboard.putString("RelDrive/Status", "INIT");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = drivetrain.getPoseMeters();
    distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
    offsetVector.x = targetPose.getTranslation().minus(robotPose.getTranslation()).getX();
    offsetVector.y = targetPose.getTranslation().minus(robotPose.getTranslation()).getY();
    if (offsetVector.magnitude() > 0.1) {
      offsetVector.x /= distance;
      offsetVector.y /= distance;
    }
    double robotAngle = robotPose.getRotation().getRadians();

    x_robot.x = Math.cos(robotAngle);
    x_robot.y = Math.sin(robotAngle);
    y_robot.x = Math.cos(robotAngle + Math.PI * 0.5);
    y_robot.y = Math.sin(robotAngle + Math.PI * 0.5);

    double falloffDistance = Math.max(0, Math.min(1, 1 - Math.exp(-10 * (distance - 0.1))));

    // double targetVelocity = maxVelocity * MathUtil.clamp(-1, 1,
    //   alpha * (1 - Math.exp(-2 * distance)) * (Math.max(0, -x_robot.dot(offsetVector)))
    // );

    double targetVelocity = maxVelocity * MathUtil.clamp(
      x_robot.dot(offsetVector),
      -1,1
    );

    double targetRotation = 
        maxAngularVelocity * MathUtil.clamp(
        beta * (y_robot.dot(x_target)) * (1 - falloffDistance)
          + gamma * (y_robot.dot(offsetVector) - 0.4 * falloffDistance * (y_robot.dot(x_target))) * falloffDistance,
        -1, 1
    );

    SmartDashboard.putString("RelDrive/Status", "EXECUTING");
    SmartDashboard.putNumber("RelDrive/Vel", targetVelocity);
    SmartDashboard.putNumber("RelDrive/RVel", targetRotation);
    SmartDashboard.putNumber("RelDrive/FalloffDistance", falloffDistance);
    SmartDashboard.putNumber("RelDrive/Distance", distance);
    SmartDashboard.putNumber("RelDrive/RobotX*TargetY", x_robot.dot(y_target));

    chassisSpeeds.vxMetersPerSecond = targetVelocity;
    chassisSpeeds.omegaRadiansPerSecond = targetRotation;
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds = new ChassisSpeeds();
    drivetrain.setChassisSpeeds(chassisSpeeds);
    SmartDashboard.putString("RelDrive/Status", "ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (distance < distanceTolerance) && Math.abs(x_robot.dot(y_target)) < 0.1;
  }
}
