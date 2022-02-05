// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  //ToF sensors
  private DigitalInput tof1Input;
  private DutyCycle tof1DutyCycleInput;
  private double tof1Freq;
  private double tof1DutyCycle;
  private double tof1Range;
  private boolean ball1Held;
  private final double tof1ScaleFactor = 0.004 / (1e-6);

  private DigitalInput tof2Input;
  private DutyCycle tof2DutyCycleInput;
  private double tof2Freq;
  private double tof2DutyCycle;
  private double tof2Range;
  private boolean ball2Held;
  private final double tof2ScaleFactor = 0.004 / (1e-6);
  
  private WPI_TalonFX  flywheelMotor;
  private WPI_TalonFX loaderMotor;
  private WPI_TalonFX hoodMotor;

  private final double flywheelTicksPerRadian = 1;
  private final double loaderTicksPerRadian =1;
  private final double hoodTicksPerRadian =1;

  private double flywheelTargetVelocity;
  private double loaderTargetVelocity;
  private double hoodTargetPosition;

  /** Creates a new Shooter. */
  public Shooter() {
    tof1Input = new DigitalInput(0);
    tof1DutyCycleInput = new DutyCycle(tof1Input);
    tof1Freq = 0;
    tof1Range = 0;
    ball1Held = false;

    tof2Input = new DigitalInput(0);
    tof2DutyCycleInput = new DutyCycle(tof2Input);
    tof2Freq = 0;
    tof2Range = 0;
    ball2Held = false;

    // CAN IDs currently for roadkill
    flywheelMotor = new WPI_TalonFX(20);
    flywheelMotor.configFactoryDefault();
    flywheelMotor.setSafetyEnabled(false);
    //flywheelMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 25, 0.2));
    flywheelMotor.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putNumber("Flywheel Motor Velocity", 0);

    loaderMotor = new WPI_TalonFX(46);
    loaderMotor.configFactoryDefault();
    loaderMotor.setSafetyEnabled(false);
    //loaderMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 25, 0.2));
    loaderMotor.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putNumber("Loader Motor Velocity", 0);

    hoodMotor = new WPI_TalonFX(1);
    hoodMotor.configFactoryDefault();
    hoodMotor.setSafetyEnabled(false);
    //hoodMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 25, 0.2));
    hoodMotor.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putNumber("Hood Motor Velocity", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tof1Freq = tof1DutyCycleInput.getFrequency();
    tof1DutyCycle = tof1DutyCycleInput.getOutput();
    tof1Range = tof1ScaleFactor * (tof1DutyCycle / tof1Freq - 0.001);
    SmartDashboard.putNumber("TOF 1 Frequency", tof1Freq);
    SmartDashboard.putNumber("TOF 1 Duty Cycle", tof1DutyCycle);
    SmartDashboard.putNumber("TOF 1 Time", tof1DutyCycle / tof1Freq);
    SmartDashboard.putNumber("TOF 1 Range", tof1Range);

    tof2Freq = tof2DutyCycleInput.getFrequency();
    tof2DutyCycle = tof2DutyCycleInput.getOutput();
    tof2Range = tof2ScaleFactor * (tof2DutyCycle / tof2Freq - 0.001);
    SmartDashboard.putNumber("TOF 2 Frequency", tof2Freq);
    SmartDashboard.putNumber("TOF 2 Duty Cycle", tof2DutyCycle);
    SmartDashboard.putNumber("TOF 2 Time", tof2DutyCycle / tof2Freq);
    SmartDashboard.putNumber("TOF 2 Range", tof2Range);

    // PrintMotorTelemetry();
  }

  public double getRange1() {
    return tof1Range;
  }

  public double getRange2() {
    return tof2Range;
  }

  public boolean isBallInShooter() {
    return ball1Held;
    // ^ This returns true if the time of flight sensor detects that the ball is in the wheels.
  }

  public boolean isBallInIndexer() {
    return ball2Held;
  }

  public void setFlywheelVelocity(double velocity){
    flywheelTargetVelocity = velocity;
    flywheelMotor.set(ControlMode.Velocity, velocity * 0.1 * flywheelTicksPerRadian);
  }

  public void setLoaderVelocity(double velocity){
    loaderTargetVelocity = velocity;
    loaderMotor.set(ControlMode.Velocity, velocity * 0.1 * loaderTicksPerRadian);
  }

  public void setHoodPosition(double hoodPosition){
    hoodTargetPosition = hoodPosition;
    hoodMotor.set(ControlMode.Position, hoodPosition * hoodTicksPerRadian);
  }

  public double getFlywheelVelocity(){
    return flywheelMotor.getSelectedSensorVelocity() * 10.0 / flywheelTicksPerRadian;
  }

  public double getLoaderVelocity(){
    return loaderMotor.getSelectedSensorVelocity() * 10.0 / loaderTicksPerRadian;
  }

  public double getHoodPosition(){
    return hoodMotor.getSelectedSensorPosition() / hoodTicksPerRadian;
  }

  /*
  public void setPrototypePower(double power){
    prototypeMotor.set(ControlMode.PercentOutput, power);
    prototypeMotor.getMotorOutputPercent();
    prototypeMotor.getMotorOutputVoltage();
    prototypeMotor.getStatorCurrent();
    prototypeMotor.getSupplyCurrent();
    prototypeMotor.getTemperature();
    prototypeMotor.getSelectedSensorVelocity();
  }
  */

  /*
  public void PrintMotorTelemetry(){
    double outputPercent;
    double outputVoltage;
    double statorCurrent;
    double supplyCurrent;
    double temperature;
    double sensorVelocity;
    outputPercent = flywheelMotor.getMotorOutputPercent();
    outputVoltage = flywheelMotor.getMotorOutputVoltage();
    statorCurrent = flywheelMotor.getStatorCurrent();
    supplyCurrent = flywheelMotor.getSupplyCurrent();
    temperature = flywheelMotor.getTemperature();
    sensorVelocity = flywheelMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("Output Percentage", outputPercent);
    SmartDashboard.putNumber("Output Voltage", outputVoltage);
    SmartDashboard.putNumber("Stator Current", statorCurrent);
    SmartDashboard.putNumber("Supply Current", supplyCurrent);
    SmartDashboard.putNumber("Temperature", temperature);
    SmartDashboard.putNumber("Sensor Velocity", sensorVelocity);
  }
  */
}