// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  /** Creates a new Collector. */
  public Collector() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetPosition(double pos) 
  {

  }

  public double getActualPosition() 
  {
    return 0;
  }

  // velocity is the speed of the ball in the intake in meters/second
  public void setIntakeVelocity(double velocity) {

  }

  public double getIntakeVelocity() {
    return 0.0;
  }

  public boolean isIntakeStalled() {
    return false;
  }
}
