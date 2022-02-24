// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private static final FrontSonar frontSonar = new FrontSonar();

  Indexer indexer = new Indexer();

  Climber climber = new Climber();

  Drivetrain drivetrain = new Drivetrain(imu);

  Collector collector = new Collector();

  Bling bling = new Bling();

  HubTracking hubTracker = new HubTracking();
  CargoTracking cargoTracker = new CargoTracking();

  Dashboard dashboard = new Dashboard(drivetrain, collector, indexer, frontSonar, hubTracking, imu);

  NetworkTable initTable;
  NetworkTableEntry autoCheckBox;

  // Controls: Add controls here.
  DriveControls teleopDrivetrain = new DriveControls(drivetrain);
  TeleopIndexer teleopIndexer = new TeleopIndexer(indexer);
  TeleopHubTracking teleopHubTracking = new TeleopHubTracking(hubTracker);
  TeleopClimber teleopClimber = new TeleopClimber(climber);
  TeleopCargoTracking teleopCargoTracking = new TeleopCargoTracking(cargoTracker);
  TeleopCollector teleopCollector = new TeleopCollector(collector, drivetrain);

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
    climber.setDefaultCommand(teleopClimber);
    collector.setDefaultCommand(teleopCollector);

    initTable = NetworkTableInstance.getDefault().getTable("Init");
    autoCheckBox=initTable.getEntry("Enable autonomous?");
    autoCheckBox.setBoolean(false);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCheckBox.getBoolean(false)) {
      return new DriveForwardCommand(drivetrain, 2.0, 1.5).andThen(
        new WaitCommand(5.0)
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