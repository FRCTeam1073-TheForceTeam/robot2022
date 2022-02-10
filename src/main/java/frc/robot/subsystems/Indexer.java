// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Indexer extends SubsystemBase {
  private double motorVelocity;

  /** Creates a new Indexer. */
  public Indexer() {
    motorVelocity = 0.0;
  }

  @Override
  public void periodic() {
    // tofFreq = tofDutyCycleInput.getFrequency();
    // tofDutyCycle = tofDutyCycleInput.getOutput();
    // tofRange = tofScaleFactor * (tofDutyCycle / tofFreq - 0.001);
    // SmartDashboard.putNumber("TOF Frequency", tofFreq);
    // SmartDashboard.putNumber("TOF Duty Cycle", tofDutyCycle);
    // SmartDashboard.putNumber("TOF Time", tofDutyCycle / tofFreq);
    // SmartDashboard.putNumber("TOF Range", tofRange);

    if (motorVelocity > 0.0) {
      Robot.getBling().setSlot(1, 60, 168, 50);
    } else {
      Robot.getBling().setSlot(1, 168, 50, 50);
    }
  }

  public void setWheelVelocity(double velocity) {
    motorVelocity = velocity;
  }

  public double getWheelVelocity() {
    return 0.0;
  }

  public boolean isCurrentCargoThere(){
    return false;
  }

  public boolean isNextCargoThere(){
    return false;
  }

  public boolean isCurrentCargoRed(){
    return false;
  }

  public boolean isNextCargoRed(){
    return false;
  }

  public int getNumCargo(){
    return 0;
  }

  /*
   * TODO: Does this launch the cargo,
   * is it position based or velocity based,
   * how many motors
   */
}