// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HubTracking;
import frc.robot.subsystems.HubTracking.HubData;

public class Localizer extends SubsystemBase {

  private Drivetrain drivetrain;
  private HubTracking hubTracking;
  private IMU imu;
  
  // private Pose2d initialPose;
  // private double initialHeading;
  // private Pose2d estimatedPose;
  // private double currentHeading;
  
  // private Field2d fieldDisplay;

  // private DifferentialDriveOdometry odometry;
  Translation2d hubPosition;
  HubData hubData;

  public static final double azimuthThreshold = 0.1;
  public static final double upperHubRadius = 0.086; //Distance from 'fender' to center of lower hub is ~86cm as per game manual
  
  public Localizer(Drivetrain drivetrain_, HubTracking hubTracking_) {
    drivetrain = drivetrain_;
    imu = drivetrain.getIMU();
    hubTracking = hubTracking_;

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
    hubTracking.sampleHubData(hubData);
    if (hubTracking.isHubVisible() && (Math.abs(hubData.azimuth) < azimuthThreshold)) {
      double range = hubData.range + Localizer.upperHubRadius;
      double angle = imu.getAngleRadians();
      hubPosition = (new Translation2d(range * Math.cos(angle), range * Math.sin(angle)))
          .plus(drivetrain.getPoseMeters().getTranslation());
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
  
  public double getAngleToHub() {
    Translation2d diff=hubPosition.minus(drivetrain.getPoseMeters().getTranslation());
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
