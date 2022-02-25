// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Indexer extends SubsystemBase {
  private double motorPower;
  private WPI_TalonSRX indexerMotor;
  private LinearFilter filter;
  private double rawCurrent;
  private double filteredCurrent;
  private NetworkTableEntry redIndexerArea;
  private NetworkTableEntry blueIndexerArea;
  private boolean hasCargo = false;
  private boolean cargoIsRed = false;
  // private boolean isCurrentCargoRed;
  //private boolean isNextCargoThere;
  //private boolean isCurrentCargo;
  //private boolean isNextCargoRed;
  private int getNumCargo;


  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotor = new WPI_TalonSRX(19);
    resetMotor();

    motorPower = 0.0;

    filter = LinearFilter.singlePoleIIR(0.75, 0.02);
// Network Tables
    redIndexerArea = NetworkTableInstance.getDefault().getTable("INDEXER").getEntry("Red Indexer Area");
    blueIndexerArea = NetworkTableInstance.getDefault().getTable("INDEXER").getEntry("Blue Indexer Area");
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

    SmartDashboard.putNumber("o",OI.operatorController.getLeftY());

    SmartDashboard.putNumber("[Indexer] Current (A)", indexerMotor.getStatorCurrent());
    SmartDashboard.putNumber("[Indexer] Output power", indexerMotor.getMotorOutputPercent());

    rawCurrent = indexerMotor.getStatorCurrent();
    filteredCurrent = filter.calculate(rawCurrent);
    // Vision Input
    double redArea = redIndexerArea.getDouble(0);
    double blueArea = blueIndexerArea.getDouble(0);
    
    hasCargo = false;
    cargoIsRed = false;

    if (redArea > 0) {
      hasCargo = true;
      cargoIsRed =true;
    }
    if (blueArea > 0) {
       hasCargo = true;
    }

  }

  public void setPower(double power) {
    motorPower = power;
    indexerMotor.set(ControlMode.PercentOutput, motorPower);
  }

  public double getPower() {
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

  public boolean isStalled(){
    return 27.85 < Math.abs(getFilteredCurrent()); //Value copied from WPI sample code!
  }

  public double getFilteredCurrent(){
    return filteredCurrent;
  }

  public double getRawCurrent(){
    return rawCurrent;
  }

  public boolean hasCargo(){
    return hasCargo;
  }

  public boolean cargoIsRed(){
    return cargoIsRed;
  }

  public void resetMotor(){
    indexerMotor.configFactoryDefault();
    indexerMotor.setSafetyEnabled(false);
    indexerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5)); //Values copied from WPI sample code!
    indexerMotor.setNeutralMode(NeutralMode.Brake);
    // indexerMotor.enableCurrentLimit(true);
    // indexerMotor.configPeakCurrentLimit(28, 500);
    // indexerMotor.configPeakCurrentDuration(750,500);
    // indexerMotor.configContinuousCurrentLimit(15,500);
  }
}