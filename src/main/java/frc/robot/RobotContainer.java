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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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
  FrontSonar frontSonar = new FrontSonar();

  Indexer indexer = new Indexer();

  Climber climber = new Climber();

  Drivetrain drivetrain = new Drivetrain(imu);

  Collector collector = new Collector();

  Bling bling = new Bling();

  Shooter shooter = new Shooter();

  Feeder feeder = new Feeder();

  HubTracking hubTracking = new HubTracking();
  
  CargoTracking cargoTracker = new CargoTracking();

  Dashboard dashboard = new Dashboard(drivetrain, collector, indexer, frontSonar, hubTracking, imu);

  SendableChooser<Command> autoChooser;

  // Controls: Add controls here.
  DriveControls teleopDrivetrain = new DriveControls(drivetrain);
  TeleopIndexer teleopIndexer = new TeleopIndexer(indexer, shooter);
  TeleopShooter teleopShooter = new TeleopShooter(shooter);
  TeleopHubTracking teleopHubTracking = new TeleopHubTracking(hubTracking);
  TeleopFeeder teleopFeeder = new TeleopFeeder(feeder);
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

    hubTracking.setDefaultCommand(teleopHubTracking);
    cargoTracker.setDefaultCommand(teleopCargoTracking);

    drivetrain.setDefaultCommand(teleopDrivetrain);
    indexer.setDefaultCommand(teleopIndexer);
    shooter.setDefaultCommand(teleopShooter);
    feeder.setDefaultCommand(teleopFeeder);
    climber.setDefaultCommand(teleopClimber);
    collector.setDefaultCommand(teleopCollector);

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("<Select a command>", null);

    autoChooser.addOption("Auto-2Ball",
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new TurnCommand(drivetrain, Units.degreesToRadians(20.0), 1.0).andThen(new PrintCommand("Finished Turning")),
          new SequentialCommandGroup(
            new IndexCommand(indexer, shooter),
            new FeedCommand(feeder, shooter),
            new FeederAdvanceCommand(feeder, 0.4*(2.0*Math.PI)),
            new InstantCommand(feeder::zeroFeeder)
          ).andThen(new PrintCommand("Finished Feeding")),
          new ShooterTargetCommand(shooter, hubTracking, true, 2.0).andThen(new PrintCommand("Finished Spinning Up"))
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(0.4).andThen(new DriveTranslateCommand(drivetrain, 1.5, 0.6)),
          new CollectCommand(collector, drivetrain)
        ),
        new SequentialCommandGroup(
          new FeederLaunchCommand(feeder, shooter),
          new InstantCommand(feeder::zeroFeeder),
          new IndexCommand(indexer, shooter),
          new FeedCommand(feeder, shooter),
          new FeederLaunchCommand(feeder, shooter),
          new InstantCommand(feeder::zeroFeeder),
          new WaitCommand(0.5)
        ),
        new ShooterSpinDownCommand(shooter)
      )

    // new SequentialCommandGroup(
      //   new DriveTranslateCommand(drivetrain, 1.0, 2.0),
      //   new ShooterTargetCommand(shooter, hubTracking, true, 1.0),
      //   new WaitToLevel(shooter, 2.0),
      //   new ShooterFeedCommand(shooter, 2.5),
      //   new ShooterSpinDownCommand(shooter),
      //   new TurnCommand(drivetrain, Units.degreesToRadians(21.0), 1.0),
      //   new SequentialCommandGroup(
      //     new ParallelDeadlineGroup(
      //       new CollectCargoCommand(collector, indexer, shooter).withTimeout(6.0),
      //       new DriveTranslateCommand(drivetrain,0.8,0.5)
      //     ),
      //     new TurnCommand(drivetrain, Units.degreesToRadians(-21.0), 1.0),
      //     new FeedCommand(shooter).withTimeout(2.0),
      //     new ShooterTargetCommand(shooter, hubTracking, true, 2.0),
      //     new FeederLaunchCommand(shooter).withTimeout(2.0),
      //     new ShooterSpinDownCommand(shooter)
      //   ),
      //   new DriveTranslateCommand(drivetrain, -0.5, 2.0),
      //   new TurnCommand(drivetrain, -Units.degreesToRadians(90.0), 1.0),
      //   new DriveTranslateCommand(drivetrain, 1.5, 2.0)
  
      //   // Apparently this turns to the right angle for the two-ball auto? Neato.
      //   // new TurnCommand(drivetrain, Units.degreesToRadians(21.0), 1.0)
      // )
    );
    autoChooser.addOption("IndexFeedShoot-SensorBased",
      new SequentialCommandGroup(
        // new ParallelDeadlineGroup(
        //   new CollectCargoCommand(collector, indexer, shooter).withTimeout(6.0),
        //   new DriveTranslateCommand(drivetrain,0.8,0.5)
        // ),
        // new TurnCommand(drivetrain, Units.degreesToRadians(-21.0), 1.0),
        new ParallelDeadlineGroup(
          new IndexCommand(indexer, shooter),
          new CollectCommand(collector, drivetrain)
        ),
        new FeedCommand(feeder, shooter),
        new InstantCommand(feeder::zeroFeeder),
        new FeederAdvanceCommand(feeder, 0.2),
        new ShooterTargetCommand(shooter, hubTracking, true, 2.0),
        new FeederLaunchCommand(feeder, shooter),
        new WaitCommand(0.5),
        new ShooterSpinDownCommand(shooter)
      )
    );

    autoChooser.addOption("IndexFeed",
      new SequentialCommandGroup(
        new IndexCommand(indexer, shooter),
        new FeedCommand(feeder, shooter),
        new FeederAdvanceCommand(feeder, 1.5*(2.0*Math.PI)),
        new InstantCommand(feeder::zeroFeeder)
      )
    );

    autoChooser.addOption("IndexFeedCollectIndex",
    new SequentialCommandGroup(
      new IndexCommand(indexer, shooter),
      new FeedCommand(feeder, shooter),
      new FeederAdvanceCommand(feeder, 1.5*(2.0*Math.PI)),
      new InstantCommand(feeder::zeroFeeder),
      new ParallelDeadlineGroup(
        new IndexCommand(indexer, shooter),
        new CollectCommand(collector, drivetrain)
      )
    )
  );

    SmartDashboard.putData("Init/Auto Selector", autoChooser);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(OI.operatorController, XboxController.Button.kB.value).whileActiveContinuous(
    //   new SequentialCommandGroup(
    //     new ShooterSpinUpCommand(shooter, 550.0, Units.degreesToRadians(60.0)),
    //     new ShooterFeedCommand(feeder, shooter, 2.5),
    //     new ShooterSpinDownCommand(shooter)        
    //   )
    // );
    // OI.getOperatorDPadDown().whileActiveContinuous(
    //   new SequentialCommandGroup(
    //     new ShooterTargetCommand(shooter, hubTracking, true, 1.0),
    //     new WaitToLevel(feeder, shooter, 2.0),
    //     new ShooterFeedCommand(feeder, shooter, 2.5),
    //     new ShooterSpinDownCommand(shooter)
    //   )
    // );
    OI.getOperatorDPadDown().whenActive(
        new ShooterTargetCommand(shooter, hubTracking, true, 1.0)
    );
    OI.getOperatorDPadLeft().whenActive(
        new ShooterTargetCommand(shooter, hubTracking, true, 2.0)
    );
    OI.getOperatorDPadUp().whenActive(
        new ShooterTargetCommand(shooter, hubTracking, true, 3.0)
    );
    OI.getOperatorDPadRight().whenActive(
        new ShooterTargetCommand(shooter, hubTracking, true, 4.0)
    );
    (new JoystickButton(OI.operatorController,XboxController.Button.kB.value)).whenPressed(
      new SequentialCommandGroup(
        new FeedCommand(feeder, shooter),
        new FeederAdvanceCommand(feeder, 0.4*(2.0*Math.PI))
      )
    );
    (new JoystickButton(OI.operatorController,XboxController.Button.kBack.value)).whenPressed(
      new ShooterTargetCommand(shooter, hubTracking)
    );
    (new JoystickButton(OI.operatorController, XboxController.Button.kY.value)).whenPressed(
      new SequentialCommandGroup(
        new IndexCommand(indexer, shooter).withTimeout(1.0).withInterrupt(OI.operatorController::getYButtonReleased),
        new FeederLaunchCommand(feeder, shooter)
      )
    );
    (new JoystickButton(OI.operatorController,XboxController.Button.kRightBumper.value)).whenPressed(
      new  ShooterSpinDownCommand(shooter)
    );

    (new JoystickButton(OI.driverController,4)).whileHeld(
      new SequentialCommandGroup(
        new AlignToHub(drivetrain, hubTracking)
      )
    );

    // new FeederLaunchCommand(feeder, shooter),
        // new InstantCommand(feeder::zeroFeeder),
        // new WaitCommand(0.5),
        // new ShooterSpinDownCommand(shooter)
    // OI.getOperatorDPadUp().whileActiveContinuous(
    //   new SequentialCommandGroup(
    //     new ShooterTargetCommand(shooter, hubTracking, true, 3.0),
    //     new WaitToLevel(feeder, shooter, 2.0),
    //     new ShooterFeedCommand(feeder, shooter, 2.5),
    //     new ShooterSpinDownCommand(shooter)
    //   )
    // );
    // OI.getOperatorDPadRight().whileActiveContinuous(
    //   new SequentialCommandGroup(
    //     new ShooterTargetCommand(shooter, hubTracking, true, 4.0),
    //     new WaitToLevel(feeder, shooter, 2.0),
    //     new ShooterFeedCommand(feeder, shooter, 2.5),
    //     new ShooterSpinDownCommand(shooter)
    //   )
    // );
    // (new JoystickButton(OI.operatorController,XboxController.Button.kStart.value)).whileHeld(
    //   new SequentialCommandGroup(
    //     new ShooterTargetCommand(shooter, hubTracking, true, 2.17),
    //     new WaitToLevel(feeder, shooter, 2.0),
    //     new ShooterFeedCommand(feeder, shooter, 2.5),
    //     new ShooterSpinDownCommand(shooter)
    //   )
    // );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // if (autoCheckBox.getBoolean(false)) {
    //   //1-ball auto
    // } else {
    //   return null;
    // }
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