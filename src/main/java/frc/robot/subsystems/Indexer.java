// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Indexer extends SubsystemBase {
  private double motorPower;
  private WPI_TalonSRX indexerMotor;
  private double rawCurrent;
  private double filteredCurrent;

  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotor = new WPI_TalonSRX(19);
    resetMotor();

    motorPower = 0.0;
  }

  @Override
  public void periodic() {}

  public void setPower(double power) {
    motorPower = power;
    indexerMotor.set(ControlMode.PercentOutput, motorPower);
  }

  public double getPower() {
    return motorPower;
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

  public boolean isStalled() {
    return false;
  }

  public double getFilteredCurrent(){
    return filteredCurrent;
  }

  public double getRawCurrent(){
    return rawCurrent;
  }

  public void resetMotor(){
    indexerMotor.configFactoryDefault();
    indexerMotor.setSafetyEnabled(false);
    indexerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5)); //Values copied from WPI sample code!
    indexerMotor.setNeutralMode(NeutralMode.Brake);
    indexerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
  }
}