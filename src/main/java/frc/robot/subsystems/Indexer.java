// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private DigitalInput tofInput;
  private DutyCycle tofDutyCycleInput;

  private double tofFreq;
  private double tofDutyCycle;
  private double tofRange;

  private final double tofScaleFactor = 0.004 / (1e-6);

  /** Creates a new Indexer. */
  public Indexer() {
    tofInput = new DigitalInput(0);
    tofDutyCycleInput = new DutyCycle(tofInput);
    tofFreq = 0;
    tofRange = 0;
  }

  @Override
  public void periodic() {
    tofFreq = tofDutyCycleInput.getFrequency();
    tofDutyCycle = tofDutyCycleInput.getOutput();
    tofRange = tofScaleFactor * (tofDutyCycle / tofFreq - 0.001);
    SmartDashboard.putNumber("TOF Frequency", tofFreq);
    SmartDashboard.putNumber("TOF Duty Cycle", tofDutyCycle);
    SmartDashboard.putNumber("TOF Time", tofDutyCycle / tofFreq);
    SmartDashboard.putNumber("TOF Range", tofRange);
  }

  /*
   * TODO: Does this launch the cargo,
   * is it position based or velocity based,
   * how many motors
   */
}
