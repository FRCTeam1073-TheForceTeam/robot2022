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

  private double lift_kP=0.1;
  private double lift_kI=0;
  private double lift_kD=0;
  private double lift_kF=0;

  private double collect_kP=0.1;
  private double collect_kI=0;
  private double collect_kD=0;
  private double collect_kF=0;

  private TrapezoidProfile liftProfile;
  private TrapezoidProfile.State previousState;
  private double liftProfileStartTime;

  private double targetLiftPosition; //Units: radians
  private final double maxLiftVelocity = 6.0; //Units: radians/s
  private final double maxLiftAcceleration = 1.5; //Units: radians/s^2
  private final double liftBeltRatio = 1.0;
  private final double liftTicksPerRadian = 2048.0 * liftBeltRatio;

  private final double intakeTicksPerRadian = 1000.0;
  private double targetIntakeVelocity = 0;

  private double maxLiftHeight = 1.0;

  SlewRateLimiter collectFilter = new SlewRateLimiter(0.5);

  /** Creates a new Collector. */
  public Collector() 
  {
    liftMotor = new WPI_TalonFX(46); // set CAN ID
    collectMotor = new WPI_TalonFX(20); // set CAN ID
    resetMotors();
    previousState = new TrapezoidProfile.State(0,0);
    targetLiftPosition = 0;
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

      collectMotor.config_kP(0,collect_kP);
      collectMotor.config_kI(0,collect_kI);
      collectMotor.config_kD(0,collect_kD);
      collectMotor.config_kF(0,collect_kF);

      System.out.printf(">\t%f\n",lift_kP);

      SmartDashboard.putBoolean("Update", false);
    }
    if(ctr%50==0){
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
    double vel=collectFilter.calculate(targetIntakeVelocity);
    SmartDashboard.putNumber("Yeah", vel);
    collectMotor.set(ControlMode.Velocity, vel*intakeTicksPerRadian*0.1);
    SmartDashboard.putNumberArray("collector lift position vs target position",new Double []{liftMotor.getSelectedSensorPosition(),targetLiftPosition});
    //SmartDashboard.putNumber("collector lift target position",);
    SmartDashboard.putNumber("collector lift close loop error",liftMotor.getClosedLoopError());
    SmartDashboard.putNumber("collector lift trap position",previousState.position);
    SmartDashboard.putNumber("collector lift trap velocity",previousState.velocity);
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
//    collectMotor.set(ControlMode.Velocity, collectFilter.calculate(targetIntakeVelocity * intakeTicksPerRadian * 0.1));
  }

  public double getIntakeVelocity() {
    return collectMotor.getSelectedSensorVelocity() / intakeTicksPerRadian * 10.0;
  }

  public boolean isIntakeStalled() {
    return false; //TODO: How do we do this?
  }

  private void resetMotors() {
    liftMotor.configFactoryDefault();
    collectMotor.configFactoryDefault();

    liftMotor.setSafetyEnabled(false);
    collectMotor.setSafetyEnabled(false);

    liftMotor.setNeutralMode(NeutralMode.Brake);
    collectMotor.setNeutralMode(NeutralMode.Brake);

    liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.25));
    collectMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.25));

    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    collectMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    liftMotor.setSelectedSensorPosition(0);
    collectMotor.setSelectedSensorPosition(0);

    liftMotor.config_kP(0, lift_kP);
    liftMotor.config_kI(0, lift_kI);
    liftMotor.config_kD(0, lift_kD);
    liftMotor.config_kF(0, lift_kF);

    collectMotor.config_kP(0, collect_kP);
    collectMotor.config_kI(0, collect_kI);
    collectMotor.config_kD(0, collect_kD);
    collectMotor.config_kF(0, collect_kF);
  }
}
