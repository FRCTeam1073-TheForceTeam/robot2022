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
import edu.wpi.first.math.util.Units;
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

  private double flywheel_kP = 0.2;
  private double flywheel_kI = 0.004;
  private double flywheel_kD = 0;
  private double flywheel_kF = 0.05;
  private double flywheel_maxIntegrator = 6000;
  private double flywheelTicksPerRadian = 2048.0 / (2.0 * Math.PI);

  //Yes, I know it's not just a belt, there's a gearbox in there too, but I don't really care?
  private final double hoodGearRatio = 75;

  private double hood_kP = 0.1;
  private double hood_kI = 0.01;
  private double hood_kD = 0.005;
  private double hood_kF = 0;
  // private double hood_kP = 0.18;
  // private double hood_kI = 0.02;
  // private double hood_kD = 0.015;
  // private double hood_kF = 0;

  private double hood_maxIntegrator = 4000;
  private double hoodTicksPerRadian = 2048.0 * hoodGearRatio / (2.0 * Math.PI);


  private final double feederRateLimit = 120;
  private final double flywheelRateLimit = 600;

  private double feederTargetVelocity = 0;
  private SlewRateLimiter feederRateLimiter = new SlewRateLimiter(feederRateLimit);
  private double limitedFeederTargetVelocity = 0;

  private double flywheelTargetVelocity = 0;
  private SlewRateLimiter flywheelRateLimiter = new SlewRateLimiter(flywheelRateLimit);
  private double limitedFlywheelTargetVelocity = 0;

  private double hoodTargetPosition = 0;
  private double hoodProfileTargetPosition = 0;
  private TrapezoidProfile hoodProfile;
  private TrapezoidProfile.State previousState;
  private double hoodProfileStartTime;

  private final double maxHoodVelocity = 5.0; //Units :radians/s
  private final double maxHoodAcceleration = 6.0; //Units: radians/s^2

  private final double maximumHoodAngle = 1.27;

  private final boolean FEEDER_TUNING_DEBUG = false;
  private final boolean FLYWHEEL_TUNING_DEBUG = true;
  private final boolean HOOD_TUNING_DEBUG = true;

  private double currentFeederVelocity = 0;
  private double currentFlywheelVelocity = 0;
  
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

    flywheelMotor = new WPI_TalonFX(45);
    resetMotors(flywheelMotor);
    flywheelMotor.setInverted(true);
    flywheelMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 32, 0.25));
    // flywheelMotor.setInverted(false);

    hoodMotor = new WPI_TalonFX(18);
    resetMotors(hoodMotor);
    hoodMotor.setSelectedSensorPosition(0);

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

    SmartDashboard.putNumber("[Shooter] flywheel/kP", flywheel_kP);
    SmartDashboard.putNumber("[Shooter] flywheel/kI", flywheel_kI);
    SmartDashboard.putNumber("[Shooter] flywheel/kD", flywheel_kD);
    SmartDashboard.putNumber("[Shooter] flywheel/kF", flywheel_kF);

    SmartDashboard.putNumber("[Shooter] hood/kP", hood_kP);
    SmartDashboard.putNumber("[Shooter] hood/kI", hood_kI);
    SmartDashboard.putNumber("[Shooter] hood/kD", hood_kD);
    SmartDashboard.putNumber("[Shooter] hood/kF", hood_kF);
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


    // Feeder periodic code
    limitedFeederTargetVelocity = feederRateLimiter.calculate(feederTargetVelocity);
    feederMotor.set(ControlMode.Velocity, limitedFeederTargetVelocity * 0.1 * feederTicksPerRadian);
    currentFeederVelocity = feederMotor.getSelectedSensorVelocity() * 10.0 / feederTicksPerRadian;
    
    if (FEEDER_TUNING_DEBUG) {
      SmartDashboard.putNumber("A", limitedFeederTargetVelocity * 0.1 * feederTicksPerRadian);
      SmartDashboard.putNumberArray("[Shooter] Feeder actual vs. target vel", new Double[] {
        limitedFeederTargetVelocity,
        currentFeederVelocity,
        currentFeederVelocity-limitedFeederTargetVelocity
      });
      SmartDashboard.putNumber("[Shooter] Feeder error ratio", (currentFeederVelocity - limitedFeederTargetVelocity) / limitedFeederTargetVelocity);
      SmartDashboard.putNumber("[Shooter] Feeder position", feederMotor.getSelectedSensorPosition() / feederTicksPerRadian);
    }
    
    // Flywheel periodic code
    limitedFlywheelTargetVelocity = flywheelRateLimiter.calculate(flywheelTargetVelocity);
    flywheelMotor.set(ControlMode.Velocity, limitedFlywheelTargetVelocity * 0.1 * flywheelTicksPerRadian);
    currentFlywheelVelocity = flywheelMotor.getSelectedSensorVelocity() * 10.0 / flywheelTicksPerRadian;

    if (FLYWHEEL_TUNING_DEBUG) {
      SmartDashboard.putNumber("[Shooter] Flywheel actual vel",
        currentFlywheelVelocity
      );
      SmartDashboard.putNumber("[Shooter] Flywheel target vel",
        limitedFlywheelTargetVelocity
      );
      SmartDashboard.putNumber("[Shooter] Flywheel error vel",
        limitedFlywheelTargetVelocity-currentFlywheelVelocity
      );
      SmartDashboard.putNumber("[Shooter] Flywheel error ratio", (currentFlywheelVelocity - limitedFlywheelTargetVelocity) / limitedFlywheelTargetVelocity);
      SmartDashboard.putNumber("[Shooter] Flywheel rotations", flywheelMotor.getSelectedSensorPosition() / (2.0 * Math.PI * flywheelTicksPerRadian));        
      SmartDashboard.putNumber("[Shooter] Flywheel Data/Raw vel", flywheelMotor.getSelectedSensorVelocity());
      SmartDashboard.putNumber("[Shooter] Flywheel Data/Out", flywheelMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("[Shooter] Flywheel Data/Current", flywheelMotor.getSupplyCurrent());
      SmartDashboard.putNumber("[Shooter] Flywheel Data/Temperature", flywheelMotor.getTemperature());
    }


    // Hood periodic code
    previousState = hoodProfile.calculate(
      ((double)System.currentTimeMillis()) / 1000.0 - hoodProfileStartTime
    );
    hoodMotor.set(
      ControlMode.Position,
      previousState.position * hoodTicksPerRadian
    );
    double currentHoodPos = hoodMotor.getSelectedSensorPosition() / hoodTicksPerRadian;

    if (HOOD_TUNING_DEBUG) {
      SmartDashboard.putNumber("[Shooter] Hood tuning/Target pos",
        previousState.position
      );
      SmartDashboard.putNumber("[Shooter] Hood tuning/Actual pos",
        currentHoodPos
      );
      SmartDashboard.putNumber("[Shooter] Hood tuning/Error pos",
        currentHoodPos-previousState.position
      );
      SmartDashboard.putNumber("[Shooter] Hood tuning/Error ratio", (currentHoodPos - previousState.position) / previousState.position);
      SmartDashboard.putNumber("[Shooter] Hood tuning/Current",
        hoodMotor.getSupplyCurrent()
      );
      SmartDashboard.putNumber("[Shooter] Hood tuning/Accumulator",
        hoodMotor.getIntegralAccumulator()
      );
      SmartDashboard.putNumber("[Shooter] Hood tuning/Degrees",
        Units.radiansToDegrees(currentHoodPos)
      );
    }


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
    hoodMotor.setIntegralAccumulator(0);
    System.out.println(getHoodPosition());
    hoodProfileStartTime = (double) (System.currentTimeMillis() / 1000.0);
    hoodProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        maxHoodVelocity,
        maxHoodAcceleration
      ),
      new TrapezoidProfile.State(
        hoodTargetPosition,
        0
      ),
      new TrapezoidProfile.State(
        getHoodPosition(),
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

  public double getFlywheelVelocity() {
    return currentFlywheelVelocity;
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