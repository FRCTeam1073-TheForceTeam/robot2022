// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private WPI_TalonFX feederMotor;

  private double feeder_kP = 0.1;
  private double feeder_kI = 0;
  private double feeder_kD = 0;
  private double feeder_kF = 0.048;
  private double feeder_maxIntegrator = 1000;
  private double feederTicksPerRadian = 2048.0 / (2.0 * Math.PI);

  private final double feederRateLimit = 120;

  private double feederTargetVelocity = 0;
  private SlewRateLimiter feederRateLimiter = new SlewRateLimiter(feederRateLimit);
  private double limitedFeederTargetVelocity = 0;

  private final boolean FEEDER_TUNING_DEBUG = false;

  private double currentFeederVelocity = 0;
  private double currentFeederPosition = 0;

  /** Creates a new Feeder. */
  public Feeder() {
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

    SmartDashboard.putBoolean("[Feeder] feeder/update", false);
    SmartDashboard.putNumber("[Feeder] feeder/kP", feeder_kP);
    SmartDashboard.putNumber("[Feeder] feeder/kI", feeder_kI);
    SmartDashboard.putNumber("[Feeder] feeder/kD", feeder_kD);
    SmartDashboard.putNumber("[Feeder] feeder/kF", feeder_kF);
  }

  @Override
  public void periodic() {
    // Feeder periodic code
    limitedFeederTargetVelocity = feederRateLimiter.calculate(feederTargetVelocity);
    feederMotor.set(ControlMode.Velocity, limitedFeederTargetVelocity * 0.1 * feederTicksPerRadian);
    currentFeederVelocity = feederMotor.getSelectedSensorVelocity() * 10.0 / feederTicksPerRadian;
    currentFeederPosition = feederMotor.getSelectedSensorPosition() / feederTicksPerRadian;
    
    if (FEEDER_TUNING_DEBUG) {
      SmartDashboard.putNumber("A", limitedFeederTargetVelocity * 0.1 * feederTicksPerRadian);
      SmartDashboard.putNumberArray("[Shooter] Feeder actual vs. target vel", new Double[] {
        limitedFeederTargetVelocity,
        currentFeederVelocity,
        currentFeederVelocity-limitedFeederTargetVelocity
      });
      SmartDashboard.putNumber("[Shooter] Feeder error ratio", (currentFeederVelocity - limitedFeederTargetVelocity) / limitedFeederTargetVelocity);
    }
    
    if (SmartDashboard.getBoolean("[Shooter] feeder/update", false)) {
      feeder_kP = SmartDashboard.getNumber("[Shooter] feeder/kP", 0);
      feeder_kI = SmartDashboard.getNumber("[Shooter] feeder/kI", 0);
      feeder_kD = SmartDashboard.getNumber("[Shooter] feeder/kD", 0);
      feeder_kF = SmartDashboard.getNumber("[Shooter] feeder/kF", 0);

      updatePID(
        feederMotor,
        feeder_kP,
        feeder_kI,
        feeder_kD,
        feeder_kF,
        feeder_maxIntegrator
      );
    }
  }

  public void setFeederVelocity(double velocity) {
    feederTargetVelocity = velocity;
  }

  public void zeroFeeder() {
    feederTargetVelocity = 0;
    feederRateLimiter.reset(0);
  }

  public double getFeederPosition() {
    return currentFeederPosition;
  }

  public double getFeederVelocity() {
    return currentFeederVelocity;
  }

  public double getFeederTargetVelocity() {
    return limitedFeederTargetVelocity;
  }

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
}
