// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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

  @Override
  public void robotInit() 
  {
    robotContainer = new RobotContainer();
    bling = robotContainer.bling;
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
    OI.update();
  }

  @Override
  public void autonomousInit() 
  {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null)
    {
      autonomousCommand.schedule();
    }
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