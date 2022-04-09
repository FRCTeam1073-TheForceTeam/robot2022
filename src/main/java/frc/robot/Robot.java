// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;

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
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
    OI.update();
    counter++;
    if (timer.get() <= 45) {
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
  }

  @Override
  public void autonomousInit() 
  {
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
    OI.onEnable();
    teleopCommand = robotContainer.getTeleopCommand();

    if (teleopCommand != null)
    {
      teleopCommand.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

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