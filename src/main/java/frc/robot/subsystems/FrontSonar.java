// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FrontSonar extends SubsystemBase {
  // https://www.maxbotix.com/ultrasonic_sensors/mb1043.htm
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/analog-inputs-software.html

  /** Creates a new FrontSonar. */
  public FrontSonar() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO: Read the sensor and save to a variable
    // TODO: Look into filtering
  }

  // sensorNumber determines which sensor to read from
  // If invalid, return -1, otherwise range in meters
  public double getRange(int sensorNumber) {
    return 0.0;
  }
}
