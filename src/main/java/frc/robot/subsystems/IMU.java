// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class IMU extends SubsystemBase 
{
  // https://docs.ctre-phoenix.com/en/stable/ch11_BringUpPigeon.html
  // https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1sensors_1_1_pigeon_i_m_u.html
  // reference: store.ctr-electronics.com/content/api/java/html/index.html
  // reference: 2021 robot drivetrain subsystem (github)

  private WPI_PigeonIMU pigeonIMU;
  private double heading;
  private double[] accelAngles;

  /** Creates a new IMU. */
  public IMU() 
  {
    pigeonIMU = new WPI_PigeonIMU(9);
    pigeonIMU.setFusedHeading(0);
    accelAngles = new double[3];
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    heading = pigeonIMU.getFusedHeading();
    pigeonIMU.getAccelerometerAngles(accelAngles);

    System.out.println("testing");
    SmartDashboard.putNumber("pigeon/fusedHeading", heading); 
    SmartDashboard.putNumber("pigeon/roll", accelAngles[0]);
    SmartDashboard.putNumber("pigeon/pitch", accelAngles[1]);
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
