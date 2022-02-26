// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  
  //motors
  // private WPI_TalonFX  flywheelMotor;
  // private WPI_TalonFX loaderMotor;
  private WPI_TalonFX hoodMotor;

  private double hood_kP = 0.1;
  private double hood_kI = 0;
  private double hood_kD = 0;
  private double hood_kF = 0;

  // private double loader_kP;
  // private double loader_kI;
  // private double loader_kD;
  // private double loader_kF;
  
  // private double flywheel_kP;
  // private double flywheel_kI;
  // private double flywheel_kD;
  // private double flywheel_kF;

  // private double flywheelTargetVelocity;
  // private double loaderTargetVelocity;
  // private double hoodTargetPosition;

  private TrapezoidProfile hoodProfile;
  private TrapezoidProfile.State previousState;
  private double hoodProfileStartTime;

  private double targetHoodPosition;
  private final double maxHoodVelocity = 1.0; //Units :radians/s
  private final double maxHoodAcceleration = 3.0; //Units: radians/s^2
  
  // private final double flywheelTicksPerRadian = 1;
  // private final double loaderTicksPerRadian =1;
  private final double hoodTicksPerRadian = 2048.1;

  private double maxHoodHeight = 1.0;

  public Shooter() {
    tof1Input = new DigitalInput(0);
    tof1DutyCycleInput = new DutyCycle(tof1Input);
    tof1Freq = 0;
    tof1Range = 0;
    ball1Held = false;

    tof2Input = new DigitalInput(1);
    tof2DutyCycleInput = new DutyCycle(tof2Input);
    tof2Freq = 0;
    tof2Range = 0;
    ball2Held = false;

    // CAN IDs currently for roadkill
    // flywheelMotor = new WPI_TalonFX(20);
    // loaderMotor = new WPI_TalonFX(46);
    hoodMotor = new WPI_TalonFX(20);

    previousState = new TrapezoidProfile.State(0,0);
    targetHoodPosition = 0;
    hoodProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        maxHoodVelocity,maxHoodAcceleration
      ),
      previousState,
      previousState
    );
    hoodProfileStartTime = System.currentTimeMillis() / 1000.0;

    // SmartDashboard.putNumber("Flywheel Motor Velocity", 0);
    // SmartDashboard.putNumber("Loader Motor Velocity", 0);
    // SmartDashboard.putNumber("Hood Motor Velocity", 0);

    // resetMotors(flywheelMotor);
    // resetMotors(loaderMotor);
    resetMotors(hoodMotor);
  }

  int ctr=0;
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

    if (tof1Range < 0.03) {
      ball1Held = true;
    } else {
      ball1Held = false;
    }

    tof2Freq = tof2DutyCycleInput.getFrequency();
    tof2DutyCycle = tof2DutyCycleInput.getOutput();
    tof2Range = tof2ScaleFactor * (tof2DutyCycle / tof2Freq - 0.001);
    SmartDashboard.putNumber("TOF 2 Frequency", tof2Freq);
    SmartDashboard.putNumber("TOF 2 Duty Cycle", tof2DutyCycle);
    SmartDashboard.putNumber("TOF 2 Time", tof2DutyCycle / tof2Freq);
    SmartDashboard.putNumber("TOF 2 Range", tof2Range);

    if (tof2Range < 0.1) {
      ball2Held = true;
    } else {
      ball2Held = false;
    }

    // PrintMotorTelemetry();

    ctr++;
    previousState = hoodProfile.calculate(
      ((double)System.currentTimeMillis())/1000.0 - hoodProfileStartTime
    );
    hoodMotor.set(
      ControlMode.Position,
      MathUtil.clamp(previousState.position, 0, maxHoodHeight)*hoodTicksPerRadian
      );
    SmartDashboard.putNumber("Hood closed loop error", hoodMotor.getClosedLoopError());
    SmartDashboard.putNumber("Trapezoid position", previousState.position);
    SmartDashboard.putNumber("Trapezoid velocity", previousState.velocity);

    if(SmartDashboard.getBoolean("Update", false)){
      hood_kP=SmartDashboard.getNumber("shooter-hood_kP",0);
      hood_kI=SmartDashboard.getNumber("shooter-hood_kI",0);
      hood_kD=SmartDashboard.getNumber("shooter-hood_kD",0);
      hood_kF=SmartDashboard.getNumber("shooter-hood_kF",0);
    
      // loader_kP=SmartDashboard.getNumber("shooter-loader_kP",0);
      // loader_kI=SmartDashboard.getNumber("shooter-loader_kI",0);
      // loader_kD=SmartDashboard.getNumber("shooter-loader_kD",0);
      // loader_kF=SmartDashboard.getNumber("shooter-loader_kF",0);

      // flywheel_kP=SmartDashboard.getNumber("shooter-flywheel_kP",0);
      // flywheel_kI=SmartDashboard.getNumber("shooter-flywheel_kI",0);
      // flywheel_kD=SmartDashboard.getNumber("shooter-flywheel_kD",0);
      // flywheel_kF=SmartDashboard.getNumber("shooter-flywheel_kF",0);

      System.out.printf(">\t%f\n",hood_kP);

      SmartDashboard.putBoolean("Update", false);
    }
    
    SmartDashboard.putNumberArray("hood position vs target position",new Double []{hoodMotor.getSelectedSensorPosition(),targetHoodPosition});
  }

  public void setHoodPosition(double targetPosition){
    targetHoodPosition = targetPosition;
    hoodProfileStartTime = (double) (System.currentTimeMillis() / 1000.0);
    hoodProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        maxHoodVelocity,
        maxHoodAcceleration
      ),
      previousState,
      new TrapezoidProfile.State(
        targetHoodPosition,
        0
      )
    );
  }

  public double getHoodPosition(){
    return hoodMotor.getSelectedSensorPosition() / hoodTicksPerRadian;
  }

  public double getRange1() {
    return tof1Range;
  }

  public double getRange2() {
    return tof2Range;
  }

  public boolean isBallInShooter() {
    return ball2Held;
    // ^ This returns true if the time of flight sensor detects that the ball is in the wheels.
  }

  public boolean isBallInIndexer() {
    return ball1Held;
  }

  // public void setFlywheelVelocity(double velocity){
  //   flywheelTargetVelocity = velocity;
  //   flywheelMotor.set(ControlMode.Velocity, velocity * 0.1 * flywheelTicksPerRadian);
  // }

  // public void setLoaderVelocity(double velocity){
  //   loaderTargetVelocity = velocity;
  //   loaderMotor.set(ControlMode.Velocity, velocity * 0.1 * loaderTicksPerRadian);
  // }

  // public void setHoodPosition(double hoodPosition){
  //   hoodTargetPosition = hoodPosition;
  //   hoodMotor.set(ControlMode.Position, hoodPosition * hoodTicksPerRadian);
  // }

  // public double getFlywheelVelocity(){
  //   return flywheelMotor.getSelectedSensorVelocity() * 10.0 / flywheelTicksPerRadian;
  // }

  // public double getLoaderVelocity(){
  //   return loaderMotor.getSelectedSensorVelocity() * 10.0 / loaderTicksPerRadian;
  // }

  // public double getHoodPosition(){
  //   return hoodMotor.getSelectedSensorPosition() / hoodTicksPerRadian;
  // }

  public void resetMotors(WPI_TalonFX motor){
    motor.configFactoryDefault();
    motor.setSafetyEnabled(false);
    //motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 25, 0.2));
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void updatePID(WPI_TalonFX motor, double p_val, double i_val, double d_val, double f_val){
    motor.config_kP(0, p_val);
    motor.config_kI(0, i_val);
    motor.config_kD(0, d_val);
    motor.config_kF(0, f_val);
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