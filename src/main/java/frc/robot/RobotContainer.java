// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

  SequentialCommandGroup autoIndexFeedLaunch;
  CommandBase autoCollectSequence;

  SendableChooser<Command> autoChooser;

  SequentialCommandGroup fullAuto;

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
    autoChooser.setDefaultOption("<Select a command>", new InstantCommand());

    DashboardReadoutCommand.resetCounter();

    autoChooser.addOption("Auto-2Ball",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new AbsoluteDriveCommand(drivetrain,
            new Pose2d(
              1.22, 1.67, new Rotation2d(0.58)
            ), 0.1, 0.1
          ),
          new CollectCommand(collector, drivetrain),
          new ShooterTargetCommand(shooter, 2.5)
        ),
        new FeederLaunchCommand(indexer, feeder, shooter),
        new WaitCommand(0.2),
        new ShooterSpinDownCommand(shooter)
      )
    );

    autoChooser.addOption("Auto-1Ball",
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new DriveTranslateCommand(drivetrain, 1.0, 2.57),
          new SequentialCommandGroup(
            new IndexCommand(indexer, shooter),
            new ParallelDeadlineGroup(
              new FeedCommand(feeder, shooter),
              new RunCommand(()->indexer.setPower(0.1),indexer)
            ),
            new InstantCommand(feeder::zeroFeeder)
          ).andThen(new PrintCommand("Finished Feeding")),
          new ShooterTargetCommand(shooter, 2.65).andThen(new PrintCommand("Finished Spinning Up"))
        ),
        new SequentialCommandGroup(
          new FeederLaunchCommand(indexer, feeder, shooter),
          new WaitCommand(0.5)
        ),
        new ShooterSpinDownCommand(shooter)
      )
    );

    autoChooser.addOption("Auto-4Ball",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new ParallelCommandGroup(
            new AbsoluteDriveCommand(drivetrain,
              new Pose2d(
                Units.inchesToMeters(46.0), 0.0, new Rotation2d(Units.degreesToRadians(10.0))
              ), 0.1, 0.1    
            )
            //new IndexCommand(indexer, shooter)
          ),
          new CollectCommand(collector, drivetrain),
          new ShooterTargetCommand(shooter, 2.0)
        ),
        new InstantCommand(DashboardReadoutCommand::resetCounter),
        new DashboardReadoutCommand("Firing first 2 cargo"),
        new FeederLaunchCommand(indexer, feeder, shooter),
        // new ParallelDeadlineGroup(
        //   new SequentialCommandGroup(
        //     new DashboardReadoutCommand("Waiting for falling edge 1"),
        //     new ParallelRaceGroup(
        //       new WaitForFallingEdge(shooter, 2),
        //       new WaitCommand(1.0)
        //     ),
      
        //     new DashboardReadoutCommand("Indexing"),
        //     new IndexCommand(indexer, shooter),

        //     new DashboardReadoutCommand("Waiting for falling edge 2"),
        //     new ParallelRaceGroup(
        //       new WaitForFallingEdge(shooter, 2),
        //       new WaitCommand(1.0)
        //     ),

        //     new DashboardReadoutCommand("Waiting for 0.5 seconds"),
        //     new WaitCommand(0.5)
        //   ),
        //   new SequentialCommandGroup(
        //     new WaitCommand(0.3),
        //     new ContinuousFeedCommand(feeder)          
        //   )
        // ),
        
        new DashboardReadoutCommand("Driving to 3rd ball"),
        new ParallelDeadlineGroup(
          new AbsoluteDriveCommand(drivetrain,
            new Pose2d(
              5.25, -1.22, new Rotation2d(0.229)
            ), 2.4, 0.1, 0.1
          ).andThen(
            new WaitCommand(1.0)
          ),
          new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new DashboardReadoutCommand("Index #1"),              
              new IndexCommand(indexer, shooter),
              new DashboardReadoutCommand("Load"),
              new LoadCommand(indexer, feeder, shooter),
              new DashboardReadoutCommand("Wait"),
              new WaitCommand(1.0),
              new DashboardReadoutCommand("Index #2"),
              new ParallelRaceGroup(
                new WaitCommand(2.0),
                new IndexCommand(indexer, shooter)                  
              )
            ),
            new ShooterTargetCommand(shooter, 4.0),
            new CollectCommand(collector, drivetrain)
          )
        ),

        new DashboardReadoutCommand("Turning to face hub"),
        new AbsoluteDriveCommand(drivetrain,
          new Pose2d(
            5.25, -1.25, new Rotation2d(-0.09)
          ), 0.1, 0.1
        ),
        new DashboardReadoutCommand("Firing second 2 cargo"),
        new FeederLaunchCommand(indexer, feeder, shooter),
        // new DashboardReadoutCommand("Turning to face 5th cargo"),
        // new AbsoluteDriveCommand(drivetrain,
        //   new Pose2d(
        //     5.25, -1.25, new Rotation2d(2.49)
        //   ), 0.1, 0.1
        // ),
        // new DashboardReadoutCommand("Drive"),
        // new AbsoluteDriveCommand(drivetrain,
        //   new Pose2d(
        //     -1.785, 2.068, new Rotation2d(1.346)
        //   ), 0.1, 0.1
        // ),
        new DashboardReadoutCommand("Done!")
      )
    );
    autoChooser.addOption("Turn90", 
      new AbsoluteDriveCommand(drivetrain,
        new Pose2d(
          1.0, 1.0, new Rotation2d(Units.degreesToRadians(90.0))
        ),
        0.1, 0.1
      )
    );
    autoChooser.addOption("AutoIndex", 
      new ParallelCommandGroup(
        new IndexCommand(indexer, shooter),
        new CollectCommand(collector, drivetrain)
      )
    );
    autoChooser.addOption("IndexLoad",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new IndexCommand(indexer, shooter),
          new CollectCommand(collector, drivetrain)
        ),
        new LoadCommand(indexer, feeder, shooter)        
      )
    );

    autoChooser.addOption("Launch",
      new SequentialCommandGroup(
        new ShooterTargetCommand(shooter, 2.0),
        new FeederLaunchCommand(indexer, feeder, shooter)
      )
    );

    autoChooser.addOption("IndexLoadLaunch",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new IndexCommand(indexer, shooter),
          new CollectCommand(collector, drivetrain)
        ),
        new ParallelCommandGroup(
          new LoadCommand(indexer, feeder, shooter),
          new ShooterTargetCommand(shooter, 2.0)
        ),
        new FeederLaunchCommand(indexer, feeder, shooter)
      )
    );

    SmartDashboard.putData("Init/Auto Selector", autoChooser);
    SmartDashboard.putNumber("Init/Auto Delay", 0);

    autoIndexFeedLaunch = new SequentialCommandGroup(
      // new IndexCommand(indexer, shooter),
      new FeederLaunchCommand(indexer, feeder, shooter),
      new InstantCommand(feeder::zeroFeeder),
      new WaitCommand(0.2)
    );

    autoCollectSequence = new ParallelDeadlineGroup(
      new SequentialCommandGroup(
        new IndexCommand(indexer, shooter),
        new LoadCommand(indexer, feeder, shooter)
      )
    );

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
        new ShooterTargetCommand(shooter, 1.0)
    );
    OI.getOperatorDPadLeft().whenActive(
        new ShooterTargetCommand(shooter, 2.0)
    );
    OI.getOperatorDPadUp().whenActive(
        new ShooterTargetCommand(shooter, 3.0)
    );
    OI.getOperatorDPadRight().whenActive(
        new ShooterTargetCommand(shooter, 4.0)
    );
    (new JoystickButton(OI.operatorController, XboxController.Button.kLeftBumper.value)).whenPressed(
      autoCollectSequence
    );
    (new JoystickButton(OI.operatorController, XboxController.Button.kB.value)).whenPressed(
      new SequentialCommandGroup(
        new LoadCommand(indexer, feeder, shooter)
      )
    );
    (new JoystickButton(OI.operatorController,XboxController.Button.kBack.value)).whenPressed(
      new ShooterRangeTargetCommand(shooter, hubTracking)
    );
    (new JoystickButton(OI.operatorController, XboxController.Button.kY.value)).whenPressed(
      new FeederLaunchCommand(indexer, feeder, shooter)
    );
    (new JoystickButton(OI.operatorController, XboxController.Button.kRightBumper.value)).cancelWhenPressed(
      autoCollectSequence
    );
    (new JoystickButton(OI.operatorController, XboxController.Button.kRightBumper.value)).whenPressed(
      new ShooterSpinDownCommand(shooter)
    );
    (new JoystickButton(OI.operatorController, XboxController.Button.kStart.value)).whenPressed(
      new ShooterSpinUpCommand(shooter, 170, (0.25-0.08))
    );

    (new JoystickButton(OI.driverController,9)).whileHeld(
      new SequentialCommandGroup(
        new AlignToHub(drivetrain, hubTracking)
      )
    );

    (new JoystickButton(OI.driverController,9)).whenPressed(
      new SequentialCommandGroup(
        new InstantCommand(()->{
          OI.operatorController.setRumble(RumbleType.kLeftRumble, 0.4);
          OI.operatorController.setRumble(RumbleType.kRightRumble, 0.4);
        }),
        new WaitCommand(0.1),
        new InstantCommand(()->{
          OI.operatorController.setRumble(RumbleType.kLeftRumble, 0);
          OI.operatorController.setRumble(RumbleType.kRightRumble, 0);
        })
      )
    );

    (new JoystickButton(OI.driverController, 9)).whenReleased(
        () -> SmartDashboard.putBoolean("AlignToHub on", false)
    );


    // new FeederLaunchCommand(indexer, feeder, shooter),
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
    fullAuto = (new WaitCommand(SmartDashboard.getNumber("Init/Auto Delay", 0))).andThen(
      autoChooser.getSelected()
    );
    return fullAuto; 
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