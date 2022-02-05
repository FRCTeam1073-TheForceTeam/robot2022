// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX  flywheelMotor;
  private WPI_TalonFX loaderMotor;
  // private WPI_TalonFX hoodMotor;

  /** Creates a new Shooter. */
  public Shooter() {
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

    //hood will be position controlled
    /*
    hoodMotor = new WPI_TalonFX(32);
    hoodMotor.configFactoryDefault();
    hoodMotor.setSafetyEnabled(false);
    //hoodMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 25, 0.2));
    hoodMotor.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putNumber("Hood Motor Velocity", 0);
    */
  }

  public void setFlywheelVelocity(double power){
    flywheelMotor.set(ControlMode.PercentOutput, power);
  }

  public void setLoaderVelocity(double power){
    loaderMotor.set(ControlMode.PercentOutput, power);
  }

  public void setHoodPosition(){

  }

  public void getFlywheelVelocity(){

  }

  public void getLoaderVelocity(){

  }

  public void getHoodPosition(){

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    // PrintMotorTelemetry();
  }
}