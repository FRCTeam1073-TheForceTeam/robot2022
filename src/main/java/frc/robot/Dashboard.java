// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;

public class Dashboard extends SubsystemBase {

  Drivetrain drivetrain;
  Collector collector;
  Indexer indexer;
  FrontSonar frontSonar;
  HubTracking hubTracking;
  IMU imu;
  
  ShuffleboardTab driverStation;
  ShuffleboardLayout layoutDrivetrain;

  NetworkTableEntry drivetrainVelocity;
  NetworkTableEntry drivetrainVelocityMetersPerSecond;
  ChassisSpeeds chassisSpeeds;
  NetworkTableEntry tgtDrivetrainVelocity;

  /** Creates a new Dashboard. */
  public Dashboard(
    Drivetrain drivetrain,
    Collector collector,
    Indexer indexer,
    FrontSonar frontSonar,
    HubTracking hubTracking,
    IMU imu
  ) {
    this.drivetrain = drivetrain;
    this.collector = collector;
    this.indexer = indexer;
    this.frontSonar = frontSonar;
    this.hubTracking = hubTracking;
    this.imu = imu;

    driverStation = Shuffleboard.getTab("Driver Station");
    drivetrainVelocity = driverStation.add("Drivetrain Speed (ft per s)", 0).getEntry();
    tgtDrivetrainVelocity = driverStation.add("Target Drivetrain Speed (ft per s)", 0).getEntry();
    drivetrainVelocityMetersPerSecond = driverStation.add("Drivetrain Speed (m per s)", 0).getEntry();
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  }

  @Override
  public void periodic() {
    drivetrain.getChassisSpeeds(chassisSpeeds);
    drivetrainVelocity.setDouble(Units.metersToFeet(chassisSpeeds.vxMetersPerSecond));
    drivetrainVelocityMetersPerSecond.setDouble(chassisSpeeds.vxMetersPerSecond);
    tgtDrivetrainVelocity.setDouble(Units.metersToFeet(drivetrain.targetChassisSpeeds.vxMetersPerSecond));
  }
}
