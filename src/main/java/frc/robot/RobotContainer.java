// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

// Import subsystems: Add subsystems here.

import frc.robot.subsystems.*;

// Import controls: Add controls here.

// Import commands: Add commands here.

// Import components: add software components (ex. InterpolatorTable, ErrorToOutput, DataRecorder) here

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems: Add subsystems here
  IMU imu = new IMU();
  
  // Subsystems: Add subsystems.
  HubTracking hubTracking;

  FrontSonar frontSonar = new FrontSonar();

  Indexer indexer = new Indexer();

  Climber climber = new Climber();

  Drivetrain drivetrain = new Drivetrain(imu);

  Collector collector = new Collector();

  Bling bling = new Bling();

  Shooter shooter = new Shooter();

  HubTracking hubTracker = new HubTracking();
  
  CargoTracking cargoTracker = new CargoTracking();

  Dashboard dashboard = new Dashboard(drivetrain, collector, indexer, frontSonar, hubTracking, imu);

  NetworkTable initTable;
  NetworkTableEntry autoCheckBox;

  // Controls: Add controls here.
  DriveControls teleopDrivetrain = new DriveControls(drivetrain);
  TeleopIndexer teleopIndexer = new TeleopIndexer(indexer, shooter);
  TeleopShooter teleopShooter = new TeleopShooter(shooter);
  TeleopHubTracking teleopHubTracking = new TeleopHubTracking(hubTracker);
  TeleopClimber teleopClimber = new TeleopClimber(climber);
  TeleopCargoTracking teleopCargoTracking = new TeleopCargoTracking(cargoTracker);
  TeleopCollector teleopCollector = new TeleopCollector(collector, drivetrain);
  HangStartPositioning hangStartPositioning = new HangStartPositioning(drivetrain, frontSonar);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize static OI class:
    OI.init();

    hubTracker.setDefaultCommand(teleopHubTracking);
    cargoTracker.setDefaultCommand(teleopCargoTracking);

    drivetrain.setDefaultCommand(teleopDrivetrain);
    indexer.setDefaultCommand(teleopIndexer);
    shooter.setDefaultCommand(teleopShooter);
    climber.setDefaultCommand(teleopClimber);
    collector.setDefaultCommand(teleopCollector);

    initTable = NetworkTableInstance.getDefault().getTable("Init");
    autoCheckBox=initTable.getEntry("Enable autonomous?");
    autoCheckBox.setBoolean(false);
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(OI.operatorController, XboxController.Button.kB.value).whileActiveContinuous(
      new SequentialCommandGroup(
        new ShooterSpinUpCommand(shooter, 550.0, Units.degreesToRadians(60.0)),
        new ShooterFeedCommand(shooter, 2.0),
        new ShooterSpinDownCommand(shooter)        
      )
    );
    OI.getOperatorDPadDown().whileActiveContinuous(
      new SequentialCommandGroup(
        new ShooterTargetCommand(shooter, hubTracking, true, 1.0),
        new WaitToLevel(shooter, 2.0),
        new ShooterFeedCommand(shooter, 2.5),
        new ShooterSpinDownCommand(shooter)
      )
    );
    OI.getOperatorDPadLeft().whileActiveContinuous(
      new SequentialCommandGroup(
        new ShooterTargetCommand(shooter, hubTracking, true, 2.0),
        new WaitToLevel(shooter, 2.0),
        new ShooterFeedCommand(shooter, 2.5),
        new ShooterSpinDownCommand(shooter)
      )
    );
    OI.getOperatorDPadUp().whileActiveContinuous(
      new SequentialCommandGroup(
        new ShooterTargetCommand(shooter, hubTracking, true, 3.0),
        new WaitToLevel(shooter, 2.0),
        new ShooterFeedCommand(shooter, 2.5),
        new ShooterSpinDownCommand(shooter)
      )
    );
    OI.getOperatorDPadRight().whileActiveContinuous(
      new SequentialCommandGroup(
        new ShooterTargetCommand(shooter, hubTracking, true, 4.0),
        new WaitToLevel(shooter, 2.0),
        new ShooterFeedCommand(shooter, 2.5),
        new ShooterSpinDownCommand(shooter)
      )
    );
    (new JoystickButton(OI.operatorController,XboxController.Button.kStart.value)).whileHeld(
      new SequentialCommandGroup(
        new ShooterTargetCommand(shooter, hubTracking, true, 2.17),
        new WaitToLevel(shooter, 2.0),
        new ShooterFeedCommand(shooter, 2.5),
        new ShooterSpinDownCommand(shooter)
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCheckBox.getBoolean(false)) {
      //1-ball auto
      return new SequentialCommandGroup(
        new PrintCommand("DTC"),
        new DriveTranslateCommand(drivetrain, 1.0, 2.0),
        new PrintCommand("STC"),
        new ShooterTargetCommand(shooter, hubTracking, true, 1.0),
        new PrintCommand("WTL"),
        new WaitToLevel(shooter, 2.0),
        new PrintCommand("SFC"),
        new ShooterFeedCommand(shooter, 2.5),
        new PrintCommand("SSDC"),
        new ShooterSpinDownCommand(shooter),
        new PrintCommand("TC"),
        new TurnCommand(drivetrain, -Units.degreesToRadians(70.0), 1.0),
        new PrintCommand("DTC2"),
        new DriveTranslateCommand(drivetrain, 2.5, 2.0)
        // Apparently this turns to the right angle for the two-ball auto? Neato.
        // new TurnCommand(drivetrain, Units.degreesToRadians(21.0), 1.0)
      );
    } else {
      return null;
    }
  }

  public Command getTeleopCommand() {
    // Return the command that will run during teleop ('return null' means no
    // command will be run)
    return null;
  }

  public Command getTestCommand() {
    // Return the command that will run during test mode (it's not that important)
    // ('return null' means no command will be run)
    return null;
  }
}