// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FrontSonar extends SubsystemBase {
  // DEBUG:
  private final boolean debug = false;



  // https://www.maxbotix.com/ultrasonic_sensors/mb1043.htm
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/analog-inputs-software.html

  //TODO: NEEDS SECOND SENSOR

  AnalogInput analog = new AnalogInput(0);
  double inputMeters = -1;
  final double voltageRatio = 0.005 / 0.00488;

  /** Creates a new FrontSonar. */
  public FrontSonar() {
    //This will return an average voltage sampled over 2^4 readings
    analog.setAverageBits(4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputMeters = analog.getAverageVoltage()*voltageRatio;
    // The sensor doesn't measure beyond five meters
    if (inputMeters > 5) {
      inputMeters = -1;
    } else if (debug) {
      SmartDashboard.putNumber("Sonar Range", inputMeters);
    }
  }

  // sensorNumber determines which sensor to read from
  // If invalid, return -1, otherwise range in meters
  public double getRangeRight(int sensorNumber) {
    return inputMeters;
  }

  public double getRangeLeft(int sensorNumber){
    return inputMeters + 0.2;
    //Currently set up for testing on roadkill, needs to be set up for a separate sensor for the real robot
  }
}