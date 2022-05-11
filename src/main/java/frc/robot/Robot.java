// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.LogRecord;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AlignToHub;
import frc.robot.subsystems.*;
import frc.robot.subsystems.HubTracking.HubData;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private RobotContainer robotContainer;
  private Command autonomousCommand;
  private Command teleopCommand;
  private Command testCommand;
  private static Bling bling;
  Timer timer;
  int counter = 0;
  int numReadouts = 0;

  @Override
  public void robotInit() 
  {
    robotContainer = new RobotContainer();
    bling = robotContainer.bling;
    timer = new Timer();
    counter = 0;
    numReadouts = 0;
    SmartDashboard.putBoolean("Hub Tracker/LEDs active when disabled", false);
    SmartDashboard.putNumber("AlignToHub/Min velocity", AlignToHub.minVelocity);
    SmartDashboard.putNumber("AlignToHub/Scale factor", AlignToHub.scaleFactor);
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
    OI.update();
    counter++;
    if ((DriverStation.isEnabled()||timer.get()!=0) && timer.get() <= 45) {
      // Runs around every 0.2 seconds
      if (counter % 10 == 0) {
        Pose2d pose=robotContainer.drivetrain.getPoseMeters();
        numReadouts++;
        System.out.println(
          String.format(
            "ODOMETRY POINT READOUT #%d:\tTIME=%.2f SECONDS; XPOS=%.3f METERS; YPOS=%.3f METERS; ANGLE=%.4f RADIANS",
            numReadouts,
            timer.get(),
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
          )
        );
      }
    }
    AlignToHub.minVelocity = SmartDashboard.getNumber("AlignToHub/Min velocity", 0);
    AlignToHub.scaleFactor = SmartDashboard.getNumber("AlignToHub/Scale factor", 0);
  }

  @Override
  public void autonomousInit() 
  {
    robotContainer.drivetrain.setBrakeMode(true);
    robotContainer.drivetrain.setRateLimit(Drivetrain.Constants.autoRateLimit);
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null)
    {
      autonomousCommand.schedule();
    }

    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    robotContainer.drivetrain.setRateLimit(Drivetrain.Constants.teleopRateLimit);
    HubData u = new HubData();
    robotContainer.hubTracking.sampleHubData(u);
    System.out.println("[teleopInit] RANGE:"+u.range+"O"+u.area);
    OI.onEnable();
    teleopCommand = robotContainer.getTeleopCommand();

    if (teleopCommand != null)
    {
      teleopCommand.schedule();
    }
    robotContainer.drivetrain.setBrakeMode(true);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
    if (Drivetrain.initMotorsInCoastMode) {
      robotContainer.drivetrain.setBrakeMode(false);
    }
  }

  @Override
  public void disabledPeriodic() {
    if (SmartDashboard.getBoolean("Hub Tracker/LEDs active when disabled", false)) {
      robotContainer.hubTracking.setLEDIntensity(
        robotContainer.hubTracking.ledPower,
        robotContainer.hubTracking.ledPower,
        robotContainer.hubTracking.ledPower
      );
    } else {
      robotContainer.hubTracking.setLEDIntensity(0, 0, 0);
    }
  }

  @Override
  public void testInit() 
  {
    testCommand = robotContainer.getTestCommand();

    if (testCommand != null)
    {
      testCommand.schedule();
    }
  }

  @Override
  public void testPeriodic() {}

  public static Bling getBling() {
    return bling;
  }
}