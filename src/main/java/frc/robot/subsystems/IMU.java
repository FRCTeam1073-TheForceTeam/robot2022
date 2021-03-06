// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

public class IMU extends SubsystemBase 
{
  // https://docs.ctre-phoenix.com/en/stable/ch11_BringUpPigeon.html
  // https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1sensors_1_1_pigeon_i_m_u.html
  // reference: store.ctr-electronics.com/content/api/java/html/index.html
  // reference: 2021 robot drivetrain subsystem (github)

  private PigeonIMU pigeonIMU;
  private double heading;
  private double[] accelAngles;

  /** Creates a new IMU. */
  public IMU() 
  {
    pigeonIMU = new PigeonIMU(9);
    pigeonIMU.configFactoryDefault();
    pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 50);
    pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 50);
    pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 50);
    pigeonIMU.setFusedHeading(0);
    accelAngles = new double[3];
  }

  @Override
  public void periodic() 
  {
    heading = pigeonIMU.getFusedHeading();
    // pigeonIMU.getAccelerometerAngles(accelAngles);
  }

  // Access gyro data read in periodic
  public double getAngleDegrees() 
  {
    return heading;
  }

  public double getAngleRadians() 
  {
    return Math.toRadians(heading);
  }

  public double getPitch()
  {
    return accelAngles[1];
  }

  public double getRoll()
  {
    return accelAngles[0];
  }

  public void resetHeading() 
  {
    pigeonIMU.setFusedHeading(0);
    heading = 0;
  }
}