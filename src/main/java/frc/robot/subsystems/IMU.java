// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMU extends SubsystemBase {
  // https://docs.ctre-phoenix.com/en/stable/ch11_BringUpPigeon.html
  // https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1sensors_1_1_pigeon_i_m_u.html
  // reference: store.ctr-electronics.com/content/api/java/html/index.html
  // reference: 2021 robot drivetrain subsystem (github)

  // TODO: Create Pigeon IMU Object (WPI_PigeonIMU)

  /** Creates a new IMU. */
  public IMU() {
    // TODO: Calibrate (2021 code)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO: Read the gyro data (getFusedHeading)

    // TODO: Display data to shuffleboard
  }

  // Access gyro data read in periodic
  public double getAngleDegrees() {
    return 0.0;
  }

  public double getAngleRadians() {
    return 0.0;
  }

  public void resetHeading() {
    // TODO: Set heading to 0 (setFusedHeading)
  }
}