// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase 
{
  WPI_TalonFX liftMotor;
  WPI_TalonFX collectMotor;

  private double lift_kP=0;
  private double lift_kI=0;
  private double lift_kD=0;
  private double lift_kF=0;

  private double collect_kP=0;
  private double collect_kI=0;
  private double collect_kD=0;
  private double collect_kF=0;

  private TrapezoidProfile liftProfile;
  private TrapezoidProfile.State previousState;
  private double liftProfileStartTime;

  private double targetLiftPosition; //Units: radians
  private double currentLiftPosition; //Units: radians
  private final double maxLiftVelocity = 4.0; //Units: radians/s
  private final double maxLiftAcceleration = 0.5; //Units: radians/s^2
  private final double liftGearRatio = 1.0;
  private final double liftTicksPerRadian = 2048.0 * liftGearRatio / (2.0 * Math.PI);

  private final double intakeTicksPerRadian = 1000.0;
  private double targetIntakeVelocity = 0;
  private double currentIntakeVelocity = 0;

  SlewRateLimiter collectFilter = new SlewRateLimiter(0.5);

  /** Creates a new Collector. */
  public Collector() 
  {
    liftMotor = new WPI_TalonFX(49); // set CAN ID
    collectMotor = new WPI_TalonFX(48); // set CAN ID

    resetMotor(liftMotor);
    resetMotor(collectMotor);

    setPIDs(liftMotor, lift_kP, lift_kI, lift_kD, lift_kF);
    setPIDs(collectMotor, collect_kP, collect_kI, collect_kD, collect_kF);

    previousState = new TrapezoidProfile.State(0,0);
    targetLiftPosition = 0;
    currentLiftPosition = 0;
    liftProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        maxLiftVelocity, maxLiftAcceleration
      ),
      previousState,
      previousState
    );
    collectFilter=new SlewRateLimiter(10.0);
    liftProfileStartTime = System.currentTimeMillis() / 1000.0;
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
      liftMotor.config_kF(0, lift_kF);
      liftMotor.setIntegralAccumulator(0);

      collectMotor.config_kP(0,collect_kP);
      collectMotor.config_kI(0,collect_kI);
      collectMotor.config_kD(0,collect_kD);
      collectMotor.config_kF(0,collect_kF);
      collectMotor.setIntegralAccumulator(0);

      SmartDashboard.putBoolean("Update", false);
    }
    // if(ctr%50==0){
    //   SmartDashboard.putNumber("collector-lift_kP", lift_kP);
    //   SmartDashboard.putNumber("collector-lift_kI", lift_kI);
    //   SmartDashboard.putNumber("collector-lift_kD", lift_kD);
    //   SmartDashboard.putNumber("collector-lift_kF", lift_kF);
  
    //   SmartDashboard.putNumber("collector-collect_kP", collect_kP);
    //   SmartDashboard.putNumber("collector-collect_kI", collect_kI);
    //   SmartDashboard.putNumber("collector-collect_kD", collect_kD);
    //   SmartDashboard.putNumber("collector-collect_kF", collect_kF);
    //   SmartDashboard.putBoolean("Update", false);
    // }
    double intakeVel = collectFilter.calculate(targetIntakeVelocity);
    double rawIntakeVel = intakeVel * intakeTicksPerRadian * 0.1;
    
    currentIntakeVelocity = collectMotor.getSelectedSensorVelocity() / intakeTicksPerRadian * 10.0;

    collectMotor.set(ControlMode.Velocity, rawIntakeVel);

    SmartDashboard.putNumberArray("[Collector] Lift position vs target position (radians)", new Double[]{currentLiftPosition, targetLiftPosition});
    SmartDashboard.putNumber("[Collector] Lift position (radians)", currentLiftPosition);
    SmartDashboard.putNumber("[Collector] Lift error ratio", (currentLiftPosition - targetLiftPosition) / targetLiftPosition);
    SmartDashboard.putNumber("[Collector] TrapezoidProfile position", previousState.position);
    SmartDashboard.putNumber("[Collector] TrapezoidProfile velocity", previousState.velocity);
    SmartDashboard.putNumberArray("[Collector] Intake velocity vs target velocity (radians per second)", new Double[]{intakeVel, currentIntakeVelocity});
  }


  public void setLiftPosition(double targetPosition) {
    targetLiftPosition = targetPosition;
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
    return liftMotor.getSelectedSensorPosition() / liftTicksPerRadian;
  }

  // velocity is the speed of the ball in the intake in meters/second
  public void setIntakeVelocity(double velocity) {
    targetIntakeVelocity = velocity;
  }

  public double getIntakeVelocity() {
    return collectMotor.getSelectedSensorVelocity() / intakeTicksPerRadian * 10.0;
  }

  public boolean isIntakeStalled() {
    return false; //TODO: How do we do this?
  }

  private void resetMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setSafetyEnabled(false);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.25));
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setSelectedSensorPosition(0);
  }
    
  private void setPIDs(WPI_TalonFX motor, double motor_kP, double motor_kI, double motor_kD, double motor_kF){
    motor.config_kP(0, motor_kP);
    motor.config_kI(0, motor_kI);
    motor.config_kD(0, motor_kD);
    motor.config_kF(0, motor_kF);
  }
}