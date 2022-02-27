// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Shooter;

public class TeleopShooter extends CommandBase {
  private double hoodVelocity = 20;

  private Shooter shooter;

  private double hoodPosition1 = 6.0; 
  private double hoodPosition2 = -6.0;
  
  private double feederVelocity = 192;

  /** Creates a new TeleopShooter. */
  public TeleopShooter(Shooter shooter) {
    this.shooter = shooter;

    SmartDashboard.putNumber("[T-Sh] Flywheel vel (rad.s)", 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("[T-Sh] Flywheel vel (rads)", 0);
    // shooter.setFlywheelVelocity(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double flywheelTargetVel = SmartDashboard.getNumber("[T-Sh] Flywheel vel (rad.s)", 0);
    shooter.setFlywheelVelocity(flywheelTargetVel);

    // if (OI.operatorController.getXButton()) {
    //   shooter.setFeederVelocity(feederVelocity);      
    // } else {
    //   shooter.setFeederVelocity(0);
    // }

    if (OI.operatorController.getXButtonPressed()) {
      shooter.setHoodPosition(Units.degreesToRadians(30.0));
    }
    if (OI.operatorController.getYButtonPressed()) {
      shooter.setHoodPosition(Units.degreesToRadians(60.0));
    }
    if (OI.operatorController.getXButtonReleased()||OI.operatorController.getYButtonReleased()){
      shooter.setHoodPosition(Units.degreesToRadians(0.0));      
    }


    /*Just an arbitrary temporary number for the joystick because the old xbox
    code here was causing an error with the new OI code from main*/
    // if (OI.driverController.getRawButtonPressed(8)) {
    //   shooter.setHoodPosition(hoodPosition1);
    // }
    //Temporary button number!
    // else if (OI.driverController.getRawButtonPressed(7)) {
    //   shooter.setHoodPosition(hoodPosition2);
    // }

    // double flywheelVelocity;
    // double loaderVelocity;
    // double hoodPosition;

    // flywheelVelocity = SmartDashboard.getNumber("Flywheel Motor Velocity", 0);
    // if (flywheelVelocity < -1) {
    //   flywheelVelocity = -1;
    // }
    // if (flywheelVelocity > 1) {
    //   flywheelVelocity = 1;
    // }

    // loaderVelocity = SmartDashboard.getNumber("Loader Motor Velocity", 0);
    // if (loaderVelocity < -1) {
    //   loaderVelocity = -1;
    // }
    // if (loaderVelocity > 1) {
    //   loaderVelocity = 1;
    // }

    // hoodPosition = SmartDashboard.getNumber("Set Hood Position", 0);
    // if (hoodPosition < -1) {
    //   hoodPosition = -1;
    // }
    // if (hoodPosition >1) {
    //   hoodPosition = 1;
    // }

    // System.out.println(flywheelVelocity);
    // shooter.setFlywheelVelocity(flywheelVelocity);

    // System.out.println(loaderVelocity);
    // shooter.setLoaderVelocity(loaderVelocity);

    // System.out.println(hoodPosition);
    // shooter.setHoodPosition(hoodPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}