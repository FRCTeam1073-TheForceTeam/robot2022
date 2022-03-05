// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase 
{
  WPI_TalonFX liftMotor;
  WPI_TalonFX collectMotor;

  private double lift_kP=0.2;
  private double lift_kI=0.001;
  private double lift_kD=0;
  private double lift_kF=0;

  private double collect_kP=0.2;
  private double collect_kI=0.001;
  private double collect_kD=0;
  private double collect_kF=0.05;

  private TrapezoidProfile liftProfile;
  private TrapezoidProfile.State previousState;
  private double liftProfileStartTime;

  private double targetLiftPosition; //Units: radians
  private double currentLiftPosition; //Units: radians
  private final double maxLiftVelocity = 16.0; //Units: radians/s
  private final double maxLiftAcceleration = 4.0; //Units: radians/s^2
  private final double liftGearRatio = 20.0;
  private final double liftTicksPerRadian = 2048.0 * liftGearRatio / (2.0 * Math.PI);

  private final double intakeGearRatio = 5.0;
  private final double intakeTicksPerRadian = 2048.0 * intakeGearRatio / (2.0 * Math.PI);
  private double targetIntakeVelocity = 0;
  private double currentIntakeVelocity = 0;

  private final double intakeWheelRadius = Units.inchesToMeters(1.5);
  SlewRateLimiter collectFilter=new SlewRateLimiter(80.0);

  /** Creates a new Collector. */
  public Collector() 
  {
    liftMotor = new WPI_TalonFX(49); // set CAN ID
    collectMotor = new WPI_TalonFX(48); // set CAN ID

    resetMotor(liftMotor);
    liftMotor.setSelectedSensorPosition(0);
    resetMotor(collectMotor);
    collectMotor.setInverted(true);

    liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 22, 28, 0.25));
    collectMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 22, 28, 0.25));

    setPIDs(liftMotor, lift_kP, lift_kI, lift_kD, lift_kF);
    setPIDs(collectMotor, collect_kP, collect_kI, collect_kD, collect_kF);

    previousState = new TrapezoidProfile.State(0, 0);
    targetLiftPosition = 0;
    currentLiftPosition = 0;
    liftProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            maxLiftVelocity, maxLiftAcceleration),
        previousState,
        previousState);
    liftProfileStartTime = System.currentTimeMillis() / 1000.0;
    SmartDashboard.putNumber("collector-lift_kP", lift_kP);
    SmartDashboard.putNumber("collector-lift_kI", lift_kI);
    SmartDashboard.putNumber("collector-lift_kD", lift_kD);
    SmartDashboard.putNumber("collector-lift_kF", lift_kF);

    SmartDashboard.putNumber("collector-collect_kP", collect_kP);
    SmartDashboard.putNumber("collector-collect_kI", collect_kI);
    SmartDashboard.putNumber("collector-collect_kD", collect_kD);
    SmartDashboard.putNumber("collector-collect_kF", collect_kF);
    SmartDashboard.putBoolean("Update", false);

  }

  int ctr=0;
  @Override
  public void periodic() {
    ctr++;
    previousState = liftProfile.calculate(
      ((double)System.currentTimeMillis())/1000.0 - liftProfileStartTime
    );
    liftMotor.set(
      ControlMode.Position,
      previousState.position*liftTicksPerRadian
    );
    SmartDashboard.putNumber("VALUE", collectMotor.getMotorOutputPercent());
    currentLiftPosition = liftMotor.getSelectedSensorPosition() / liftTicksPerRadian;
    if(SmartDashboard.getBoolean("Update", false)){
      lift_kP=SmartDashboard.getNumber("collector-lift_kP",0);
      lift_kI=SmartDashboard.getNumber("collector-lift_kI",0);
      lift_kD=SmartDashboard.getNumber("collector-lift_kD",0);
      lift_kF=SmartDashboard.getNumber("collector-lift_kF",0);
    
      collect_kP=SmartDashboard.getNumber("collector-collect_kP",0);
      collect_kI=SmartDashboard.getNumber("collector-collect_kI",0);
      collect_kD=SmartDashboard.getNumber("collector-collect_kD",0);
      collect_kF=SmartDashboard.getNumber("collector-collect_kF",0);

      liftMotor.config_kP(0,lift_kP);
      liftMotor.config_kI(0,lift_kI);
      liftMotor.config_kD(0,lift_kD);
      liftMotor.config_kF(0,lift_kF);
      liftMotor.setIntegralAccumulator(0);

      collectMotor.config_kP(0,collect_kP);
      collectMotor.config_kI(0,collect_kI);
      collectMotor.config_kD(0,collect_kD);
      collectMotor.config_kF(0,collect_kF);
      collectMotor.setIntegralAccumulator(0);

      SmartDashboard.putBoolean("Update", false);
    }
    double intakeVel = collectFilter.calculate(targetIntakeVelocity);
    double rawIntakeVel = intakeVel * intakeTicksPerRadian * 0.1;
    
    collectMotor.set(ControlMode.Velocity, rawIntakeVel);

    currentIntakeVelocity = collectMotor.getSelectedSensorVelocity() / intakeTicksPerRadian * 10.0;

    SmartDashboard.putNumberArray("[Collector] Lift position vs target position (radians)", new Double[]{currentLiftPosition, targetLiftPosition});
    SmartDashboard.putNumber("[Collector] Lift position (radians)", currentLiftPosition);
    SmartDashboard.putNumber("[Collector] Lift error ratio", (currentLiftPosition - targetLiftPosition) / targetLiftPosition);
    SmartDashboard.putNumber("[Collector] TrapezoidProfile position", previousState.position);
    SmartDashboard.putNumber("[Collector] TrapezoidProfile velocity", previousState.velocity);
    SmartDashboard.putNumberArray("[Collector] Intake velocity vs target velocity (radians per second)", new Double[]{intakeVel, currentIntakeVelocity});
    SmartDashboard.putNumber("[Collector] Intake velocity error ratio", (intakeVel-currentIntakeVelocity)/intakeVel);
  }

  public void setLiftPosition(double targetPosition) {
    targetLiftPosition = targetPosition;
    liftMotor.setIntegralAccumulator(0);
    // System.out.println(getLiftPosition());
    liftProfileStartTime = (double) (System.currentTimeMillis() / 1000.0);
    liftProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        maxLiftVelocity,
        maxLiftAcceleration
      ),
      new TrapezoidProfile.State(
        targetLiftPosition,
        0
      ),
      new TrapezoidProfile.State(
        getLiftPosition(),
        0
      )
    );
  }

  public double getLiftPosition() {
    return currentLiftPosition;
  }

  public void hardSetIntakeVelocity(double tgt) {
    targetIntakeVelocity = tgt;
    collectFilter.reset(tgt);
  }

  /**
   * Sets the <b>angular</b> velocity of the intake wheels.
   * @param velocity Velocity in radians/second
   */
  public void setIntakeVelocity(double velocity) {
    targetIntakeVelocity = velocity;
  }

  /**
   * Sets the <b>linear</b> velocity of the intake wheels.
   * @param linearVelocity Velocity in meters/second
   */
  public void setLinearIntakeVelocity(double linearVelocity) {
    targetIntakeVelocity = linearVelocity / (2.0 * Math.PI * intakeWheelRadius);
  }

  public double getIntakeVelocity() {
    return collectMotor.getSelectedSensorVelocity() / intakeTicksPerRadian * 10.0;
  }

  public double getLinearIntakeVelocity() {
    return collectMotor.getSelectedSensorVelocity() * (2.0 * Math.PI * intakeWheelRadius) / intakeTicksPerRadian * 10.0;
  }

  public boolean isIntakeStalled() {
    return false;
  }

  private void resetMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setSafetyEnabled(false);
    collectMotor.setSafetyEnabled(false);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setSelectedSensorPosition(0);
  }
    
  private void setPIDs(WPI_TalonFX motor, double motor_kP, double motor_kI, double motor_kD, double motor_kF) {
    motor.config_kP(0, motor_kP);
    motor.config_kI(0, motor_kI);
    motor.config_kD(0, motor_kD);
    motor.config_kF(0, motor_kF);
    motor.configMaxIntegralAccumulator(0, 300);
  }
}