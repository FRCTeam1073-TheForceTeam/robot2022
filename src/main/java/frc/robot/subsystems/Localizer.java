// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.HubTracking;
import frc.robot.subsystems.HubTracking.HubData;

public class Localizer extends SubsystemBase {

  public enum LocalizationMode {
    Continuous,
    OnLaunch
  }

  private Drivetrain drivetrain;
  private HubTracking hubTracking;
  private IMU imu;

  private ChassisSpeeds robotSpeeds;
  
  // private Pose2d initialPose;
  // private double initialHeading;
  // private Pose2d estimatedPose;
  // private double currentHeading;
  
  // private Field2d fieldDisplay;

  // private DifferentialDriveOdometry odometry;
  Translation2d hubPosition;
  HubData hubData;

  LocalizationMode mode = LocalizationMode.OnLaunch;

  public static final double azimuthThreshold = 0.05;
  public static final double upperHubRadius = 0.086; //Distance from 'fender' to center of lower hub is ~86cm as per game manual
  int floatCounter = 0;
  private final int maxCount = 10;

  public Localizer(Drivetrain drivetrain_, HubTracking hubTracking_) {
    drivetrain = drivetrain_;
    imu = drivetrain.getIMU();
    hubTracking = hubTracking_;

    robotSpeeds = new ChassisSpeeds();

    // fieldDisplay = new Field2d();

    // initialPose = new Pose2d();
    // initialHeading = initialPose.getRotation().getRadians();

    // estimatedPose = new Pose2d();
    // currentHeading = estimatedPose.getRotation().getRadians();

    // odometry = new DifferentialDriveOdometry(new Rotation2d(initialHeading));

    hubData = new HubData();

    // setInitialOdometryPose(new Pose2d());
    hubPosition = new Translation2d();
  }
  
  int ctr = 0;
  @Override
  public void periodic() {
    drivetrain.getChassisSpeeds(robotSpeeds);
    hubTracking.sampleHubData(hubData);
    Robot.getBling().setSlot(4, 0, 0, 0);

    if (mode == LocalizationMode.OnLaunch) {
      Translation2d candidate = getCandidateHubPosition();
      if ((Math.abs(robotSpeeds.omegaRadiansPerSecond) < 0.02) &&
          (Math.abs(robotSpeeds.vxMetersPerSecond) < 0.1) &&
          hubTracking.isHubVisible() &&
          (Math.abs(hubData.azimuth) < azimuthThreshold) &&
          (candidate.getDistance(hubPosition) <= 3)) {
        floatCounter = Math.max(0, Math.min(floatCounter + 1, maxCount));
        if (floatCounter >= maxCount) {
          floatCounter = 0;
          Robot.getBling().setSlot(4, 0, 128, 255);
          hubPosition = new Translation2d(candidate.getX(), candidate.getY());
        }
      } else {
        floatCounter = Math.max(0, Math.min(floatCounter - 1, maxCount));
      }
    }

    // currentHeading = imu.getAngleRadians() + initialHeading;
    // double leftPos = drivetrain.getLeftWheelPosition();
    // double rightPos = drivetrain.getRightWheelPosition();

    // estimatedPose = odometry.update(
    //     new Rotation2d(currentHeading),
    //     leftPos,
    //     rightPos
    // );

    // if(hubTracking.isHubVisible()){
    //   hubTracking.sampleHubData(hubData);
    //   if(Math.abs(hubData.azimuth)<azimuthThreshold){
    //     updatePositionEstimate(
    //       -(hubData.range+upperHubRadius)*Math.cos(currentHeading),
    //       -(hubData.range+upperHubRadius)*Math.sin(currentHeading)
    //     );
    //   }
    // }

    // fieldDisplay.setRobotPose(estimatedPose);
    // SmartDashboard.putData(fieldDisplay);
    // SmartDashboard.putNumber("[Localizer] Estimated pose/X position (meters)", estimatedPose.getTranslation().getX());
    // SmartDashboard.putNumber("[Localizer] Estimated pose/Y position (meters)", estimatedPose.getTranslation().getY());
    // SmartDashboard.putNumber("[Localizer] Estimated pose/Angle (radians)", estimatedPose.getRotation().getRadians());
    // if (ctr % 10 == 0) {
    //   System.out.println("TIME: " + Timer.getMatchTime() + "ESTIMATED POSE:" + estimatedPose.toString());
    // }
    // ctr++;
  }
  
  public Translation2d getCandidateHubPosition() {
    double range = hubData.range + upperHubRadius;
    double angle = imu.getAngleRadians();
    return (new Translation2d(range * Math.cos(angle), range * Math.sin(angle)))
      .plus(drivetrain.getPoseMeters().getTranslation());
  }
  
  public double getAngleToHub() {
    Translation2d diff = hubPosition.minus(drivetrain.getPoseMeters().getTranslation());
    return Math.atan2(diff.getY(), diff.getX());
  }
  
  // public void updatePositionEstimate(double newX, double newY) {
  //   estimatedPose = new Pose2d(newX, newY, new Rotation2d(currentHeading));
  // }
  
  // public double getAngleToHub() {
  //   return Math.atan2(-estimatedPose.getY(),-estimatedPose.getX());
  // }
  
  // public void setInitialOdometryPose(Pose2d startingPose) {
  //   initialPose = new Pose2d(startingPose.getTranslation(),startingPose.getRotation());
  //   initialHeading = initialPose.getRotation().getRadians();
  //   odometry = new DifferentialDriveOdometry(new Rotation2d(initialHeading));
  //   estimatedPose = new Pose2d(initialPose.getX(),initialPose.getY(), new Rotation2d(initialHeading));
  // }
}
