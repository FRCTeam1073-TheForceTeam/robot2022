// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  //ToF sensors
  private DigitalInput tof1Input;
  private DutyCycle tof1DutyCycleInput;
  private double tof1Freq;
  private double tof1DutyCycle;
  private double tof1Range;
  private boolean ball1Stored;
  private final double tof1ScaleFactor = 0.004 / (1e-6);

  private DigitalInput tof2Input;
  private DutyCycle tof2DutyCycleInput;
  private double tof2Freq;
  private double tof2DutyCycle;
  private double tof2Range;
  private boolean ball2Stored;
  private final double tof2ScaleFactor = 0.004 / (1e-6);
  
  //Motors
  private WPI_TalonFX feederMotor;
  private WPI_TalonFX flywheelMotor;
  private WPI_TalonFX hoodMotor;

  private double feeder_kP = 0.1;
  private double feeder_kI = 0;
  private double feeder_kD = 0;
  private double feeder_kF = 0.048;
  private double feeder_maxIntegrator = 1000;
  private double feederTicksPerRadian = 2048.0 / (2.0 * Math.PI);

  private double flywheel_kP = 0;
  private double flywheel_kI = 0;
  private double flywheel_kD = 0;
  private double flywheel_kF = 0;
  private double flywheel_maxIntegrator = 0;
  private double flywheelTicksPerRadian = 0;

  private double hood_kP = 0;
  private double hood_kI = 0;
  private double hood_kD = 0;
  private double hood_kF = 0;
  private double hood_maxIntegrator = 0;
  private double hoodTicksPerRadian = 0;

  private final double feederRateLimit = 120;
  private final double flywheelRateLimit = 0;

  private double feederTargetVelocity = 0;
  private SlewRateLimiter feederRateLimiter = new SlewRateLimiter(feederRateLimit);
  private double limitedFeederTargetVelocity = 0;

  private double flywheelTargetVelocity = 0;
  private SlewRateLimiter flywheelRateLimiter = new SlewRateLimiter(flywheelRateLimit);


  private double hoodTargetPosition = 0;
  private double hoodProfileTargetPosition = 0;
  private TrapezoidProfile hoodProfile;
  private TrapezoidProfile.State previousState;
  private double hoodProfileStartTime;

  private final double maxHoodVelocity = 1.0; //Units :radians/s
  private final double maxHoodAcceleration = 3.0; //Units: radians/s^2

  private final double maximumHoodAngle = 0;
  
  public Shooter() {
    tof1Input = new DigitalInput(0);
    tof1DutyCycleInput = new DutyCycle(tof1Input);
    tof1Freq = 0;
    tof1Range = 0;
    ball1Stored = false;

    tof2Input = new DigitalInput(1);
    tof2DutyCycleInput = new DutyCycle(tof2Input);
    tof2Freq = 0;
    tof2Range = 0;
    ball2Stored = false;

    feederMotor = new WPI_TalonFX(42);
    resetMotors(feederMotor);
    feederMotor.setInverted(true);
    feederMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.25));
    updatePID(feederMotor,
      feeder_kP,
      feeder_kI,
      feeder_kD,
      feeder_kF,
      feeder_maxIntegrator
    );
    updatePID(flywheelMotor,
      flywheel_kP,
      flywheel_kI,
      flywheel_kD,
      flywheel_kF,
      flywheel_maxIntegrator
    );
    updatePID(hoodMotor,
      hood_kP,
      hood_kI,
      hood_kD,
      hood_kF,
      hood_maxIntegrator
    );

    // flywheelMotor = new WPI_TalonFX(61);
    // hoodMotor = new WPI_TalonFX(62);

    previousState = new TrapezoidProfile.State(0,0);
    hoodTargetPosition = 0;
    hoodProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        maxHoodVelocity,maxHoodAcceleration
      ),
      previousState,
      previousState
    );
    hoodProfileStartTime = System.currentTimeMillis() / 1000.0;

    SmartDashboard.putBoolean("[Shooter] Update", false);

    SmartDashboard.putNumber("[Shooter] feeder/kP", feeder_kP);
    SmartDashboard.putNumber("[Shooter] feeder/kI", feeder_kI);
    SmartDashboard.putNumber("[Shooter] feeder/kD", feeder_kD);
    SmartDashboard.putNumber("[Shooter] feeder/kF", feeder_kF);

    SmartDashboard.putNumber("[Shooter] flywheel/kP", 0);
    SmartDashboard.putNumber("[Shooter] flywheel/kI", 0);
    SmartDashboard.putNumber("[Shooter] flywheel/kD", 0);
    SmartDashboard.putNumber("[Shooter] flywheel/kF", 0);

    SmartDashboard.putNumber("[Shooter] hood/kP", 0);
    SmartDashboard.putNumber("[Shooter] hood/kI", 0);
    SmartDashboard.putNumber("[Shooter] hood/kD", 0);
    SmartDashboard.putNumber("[Shooter] hood/kF", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tof1Freq = tof1DutyCycleInput.getFrequency();
    tof1DutyCycle = tof1DutyCycleInput.getOutput();
    tof1Range = tof1ScaleFactor * (tof1DutyCycle / tof1Freq - 0.001);
    SmartDashboard.putNumber("TOF 1/Frequency", tof1Freq);
    SmartDashboard.putNumber("TOF 1/Duty Cycle", tof1DutyCycle);
    SmartDashboard.putNumber("TOF 1/Time", tof1DutyCycle / tof1Freq);
    SmartDashboard.putNumber("TOF 1/Range", tof1Range);

    if (tof1Range < 0.03) {
      ball1Stored = true;
    } else {
      ball1Stored = false;
    }

    tof2Freq = tof2DutyCycleInput.getFrequency();
    tof2DutyCycle = tof2DutyCycleInput.getOutput();
    tof2Range = tof2ScaleFactor * (tof2DutyCycle / tof2Freq - 0.001);
    SmartDashboard.putNumber("TOF 2/Frequency", tof2Freq);
    SmartDashboard.putNumber("TOF 2/Duty Cycle", tof2DutyCycle);
    SmartDashboard.putNumber("TOF 2/Time", tof2DutyCycle / tof2Freq);
    SmartDashboard.putNumber("TOF 2/Range", tof2Range);

    if (tof2Range < 0.1) {
      ball2Stored = true;
    } else {
      ball2Stored = false;
    }

    limitedFeederTargetVelocity=feederRateLimiter.calculate(feederTargetVelocity);
    feederMotor.set(ControlMode.Velocity, limitedFeederTargetVelocity * 0.1 * feederTicksPerRadian);
    SmartDashboard.putNumber("A", limitedFeederTargetVelocity * 0.1 * feederTicksPerRadian);

    double currentVel = feederMotor.getSelectedSensorVelocity() * 10.0 / feederTicksPerRadian;

    SmartDashboard.putNumberArray("[Shooter] Feeder actual vs. target vel", new Double[] {
      limitedFeederTargetVelocity,
      currentVel,
      currentVel-limitedFeederTargetVelocity
    });
    SmartDashboard.putNumber("[Shooter] Feeder error ratio", (currentVel-limitedFeederTargetVelocity)/limitedFeederTargetVelocity);
    SmartDashboard.putNumber("[Shooter] Feeder position", feederMotor.getSelectedSensorPosition() / feederTicksPerRadian);
    // flywheelMotor.set(ControlMode.Velocity, flywheelTargetVelocity * 0.1 / flywheelTicksPerRadian);
    
    // previousState = hoodProfile.calculate(
    //   ((double)System.currentTimeMillis())/1000.0 - hoodProfileStartTime
    // );
    // hoodMotor.set(
    //   ControlMode.Position,
    //   MathUtil.clamp(previousState.position, 0, maximumHoodAngle)*hood_ticksPerRadian
    // );
    // SmartDashboard.putNumber("[Shooter] Hood closed loop error", hoodMotor.getClosedLoopError());
    // SmartDashboard.putNumber("[Shooter] Trapezoid position", previousState.position);
    // SmartDashboard.putNumber("[Shooter] Trapezoid velocity", previousState.velocity);

    if (SmartDashboard.getBoolean("[Shooter] Update", false)) {
      feeder_kP = SmartDashboard.getNumber("[Shooter] feeder/kP", 0);
      feeder_kI = SmartDashboard.getNumber("[Shooter] feeder/kI", 0);
      feeder_kD = SmartDashboard.getNumber("[Shooter] feeder/kD", 0);
      feeder_kF = SmartDashboard.getNumber("[Shooter] feeder/kF", 0);

      flywheel_kP = SmartDashboard.getNumber("[Shooter] flywheel/kP", 0);
      flywheel_kI = SmartDashboard.getNumber("[Shooter] flywheel/kI", 0);
      flywheel_kD = SmartDashboard.getNumber("[Shooter] flywheel/kD", 0);
      flywheel_kF = SmartDashboard.getNumber("[Shooter] flywheel/kF", 0);

      hood_kP = SmartDashboard.getNumber("[Shooter] hood/kP", 0);
      hood_kI = SmartDashboard.getNumber("[Shooter] hood/kI", 0);
      hood_kD = SmartDashboard.getNumber("[Shooter] hood/kD", 0);
      hood_kF = SmartDashboard.getNumber("[Shooter] hood/kF", 0);

      updatePID(
        feederMotor,
        feeder_kP,
        feeder_kI,
        feeder_kD,
        feeder_kF,
        feeder_maxIntegrator
      );

      updatePID(
        flywheelMotor,
        flywheel_kP,
        flywheel_kI,
        flywheel_kD,
        flywheel_kF,
        flywheel_maxIntegrator
      );

      updatePID(
        hoodMotor,
        hood_kP,
        hood_kI,
        hood_kD,
        hood_kF,
        hood_maxIntegrator
      );
      SmartDashboard.putBoolean("[Shooter] Update", false);
    }
  }

  public void setHoodPosition(double targetPosition){
    hoodTargetPosition = targetPosition;
    hoodProfileStartTime = (double) (System.currentTimeMillis() / 1000.0);
    hoodProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        maxHoodVelocity,
        maxHoodAcceleration
      ),
      previousState,
      new TrapezoidProfile.State(
        hoodTargetPosition,
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

  public boolean isBallInIndexer() {
    return ball1Stored;
  }

  public boolean isBallInShooter() {
    return ball2Stored;
    // ^ This returns true if the time of flight sensor detects that the ball is in the wheels.
  }

  public void setFeederVelocity(double velocity) {
    feederTargetVelocity = velocity;
  }

  public void setFlywheelVelocity(double velocity) {
    flywheelTargetVelocity = velocity;
  }

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
    if (motor == null) {
      return;
    }
    motor.configFactoryDefault();
    motor.setSafetyEnabled(false);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setSelectedSensorPosition(0);
    motor.setIntegralAccumulator(0);
  }

  public void updatePID(WPI_TalonFX motor, double p_val, double i_val, double d_val, double f_val, double max_integrator) {
    if (motor == null) {
      return;
    }
    motor.setIntegralAccumulator(0);
    motor.configMaxIntegralAccumulator(0, max_integrator);
    System.out.println("P" + p_val);
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