// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HubTracking;
import frc.robot.subsystems.HubTracking.HubData;

public class Localizer extends SubsystemBase {

  private Drivetrain drivetrain;
  private HubTracking hubTracking;
  private IMU imu;
  
  private Pose2d initalPose;
  private double initialHeading;
  private Pose2d estimatedPose;
  private double currentHeading;
  
  private Field2d fieldDisplay;

  private DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveWheelSpeeds wheelSpeeds;
  HubData hubData=new HubData();

  public static final double azimuthThreshold=0.1;
  public static final double upperHubRadius=86; //Distance from 'fender' to center of lower hub is ~86cm as per game manual
  
  public Localizer(Drivetrain drivetrain_, HubTracking hubTracking_) {
    drivetrain = drivetrain_;
    imu = drivetrain.getIMU();
    hubTracking = hubTracking_;
    
    fieldDisplay=new Field2d();
    
    initialPose = new Pose2d();
    initialHeading = initialPose.getRotation().getRadians();

    estimatedPose = new Pose2d();
    currentHeading=estimatedPose.getRotation().getRadians();

    odometry = new DifferentialDriveOdometry(new Rotation2d(initialHeading));
    
    setInitialOdometryPose(new Pose2d());
  }
  
  @Override
  public void periodic() {
    currentHeading=imu.getAngleRadians() + initialHeading;
  	double leftPos = drivetrain.getLeftWheelPosition();
    double rightPos = drivetrain.getRightWheelPosition();

    estimatedPose = odometry.update(
        new Rotation2d(currentHeading),
        leftPos,
        rightPos
    );
    
    if(hubTracking.isHubVisible()){
      hubTracking.sampleHubData(hubData);
      if(Math.abs(hubData.azimuth)<azimuthThreshold){
        updatePositionEstimate(
          -(hubData.range+upperHubRadius)*Math.cos(currentHeading),
          -(hubData.range+upperHubRadius)*Math.sin(currentHeading)
        );
      }
    }

    fieldDisplay.setRobotPose(estimatedPose);
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("[Localizer] Estimated pose/X position (meters)", estimatedPose.getTranslation().getX());
    SmartDashboard.putNumber("[Localizer] Estimated pose/Y position (meters)", estimatedPose.getTranslation().getY());
    SmartDashboard.putNumber("[Localizer] Estimated pose/Angle (radians)", estimatedPose.getRotation().getRadians());
  }
  
  public void updatePositionEstimate(double newX, double newY){
    estimatedPose=new Pose2d(newX, newY, new Rotation2d(currentHeading));
  }
  
  public double getAngleToHub(){
    return Math.atan2(-estimatedPose.getY(),-estimatedPose.getX());
  }
  
  public void setInitialOdometryPose(Pose2d startingPose) {
    initialPose = new Pose2d(startingPose.getTranslation(),startingPose.getRotation());
    initialHeading = initialPose.getRotation().getRadians();
    odometry = new DifferentialDriveOdometry(new Rotation2d(initialHeading));
    estimatedPose = new Pose2d(initialPose.getX(),initialPose.getY(), new Rotation2d(initialHeading));
  }
}