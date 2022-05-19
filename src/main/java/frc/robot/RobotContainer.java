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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;

// Import subsystems: Add subsystems here.

import frc.robot.subsystems.*;
import frc.robot.subsystems.HubTracking.HubData;

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
          new SequentialCommandGroup(
            new AbsoluteDriveCommand(
              drivetrain, 
              new Pose2d(
                0.0, 0.0, new Rotation2d(0.902)
              ),
              1.0, 5.5,
              0.1,
              0.06
            ),
            new AbsoluteDriveCommand(
              drivetrain,
              new Pose2d(
                1.23, 1.536, new Rotation2d(0.902)
              ),
              1.0,
              0.1, 0.1
            )
          ),
          new ShooterTargetCommand(shooter, 2.76),
          new CollectCommand(collector, drivetrain)
        ),
        new AbsoluteDriveCommand(
          drivetrain, 
          new Pose2d(
            1.23, 1.536, new Rotation2d(0.605)
          ),
          1.0, 5.5,
          0.1,
          0.05
        ),
        new FeederLaunchCommand(indexer, feeder, shooter),
        new WaitCommand(2.0),
        new ShooterSpinDownCommand(shooter)
      )
    );

    autoChooser.addOption("NEW-2Ball",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new AbsoluteDriveCommand(
              drivetrain,
              new Pose2d(
                1.521, 1.475, new Rotation2d(1.479)
              ),
              1.0,
              0.1, 0.1
            ),
            new AbsoluteDriveCommand(
              drivetrain,
              new Pose2d(
                1.521, 1.48, new Rotation2d(0.500)
              ),
              1.0,
              0.1, 0.1
            )
          ),
          new ShooterTargetCommand(shooter, 2.28),
          new CollectCommand(collector, drivetrain)
        ),
        new FeederLaunchCommand(indexer, feeder, shooter),
        new WaitCommand(2.0),
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
              ), 1.5, 0.1, 0.1
            )
          ),
          new IndexCommand(indexer, shooter),
          new CollectCommand(collector, drivetrain),
          new ShooterTargetCommand(shooter, 1.73)
        ),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new InstantCommand(DashboardReadoutCommand::resetCounter),
            new DashboardReadoutCommand("Firing first 2 cargo"),
            new FeederLaunchCommand(indexer, feeder, shooter, 1.7),
            new WaitCommand(0.2),
            new InstantCommand(
              ()->{
                HubData u = new HubData();
                hubTracking.sampleHubData(u);
                System.out.println("RANGE:"+u.range+"O"+u.area);
              }
            ),
            new DashboardReadoutCommand("Driving to 3rd ball"),
            new AbsoluteDriveCommand(drivetrain,
              new Pose2d(
                5.222, -1.135, new Rotation2d(0.229)
              ), 2.5, 8.0,
              0.07, 0.1
            )
          ),
          new CollectCommand(collector, drivetrain)
        ),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new DashboardReadoutCommand("Index #1"),  
            new IndexCommand(indexer, shooter).withTimeout(2.0),
            new DashboardReadoutCommand("Load"),
            new LoadCommand(indexer, feeder, shooter),
            new DashboardReadoutCommand("Index #2"),
            (new IndexCommand(indexer, shooter)).withTimeout(2.0)
          ),
          new ShooterTargetCommand(shooter, 3.1),
          new HumanPlayerSignalCommand(bling),
          new CollectCommand(collector, drivetrain)
        ),
        new DashboardReadoutCommand("Turning to face hub"),
          // new ShooterRangeTargetCommand(shooter, hubTracking),
        new WaitCommand(0.2),
        new AbsoluteDriveCommand(drivetrain,
          new Pose2d(
            2.30, -1.20, new Rotation2d(-0.048)
          ), 2.0, 5.5,
          0.1, 0.02
        ),
        // new AlignToHub(drivetrain, hubTracking).withTimeout(0.5),
        new WaitCommand(0.3),
        new DashboardReadoutCommand("Firing second 2 cargo"),
        new InstantCommand(
          ()->{
            HubData u = new HubData();
            hubTracking.sampleHubData(u);
            System.out.println("RANGE:" + u.range + "; AZIMUTH: " + u.azimuth + "; AREA: " + u.area);
          }
        ),
        new FeederLaunchCommand(indexer, feeder, shooter),
        new WaitCommand(1.0),
        new DashboardReadoutCommand("Done!"),
        new ShooterSpinDownCommand(shooter)
      )
    );

    autoChooser.addOption("Auto-4Ball-align",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new ParallelCommandGroup(
            new AbsoluteDriveCommand(drivetrain,
              new Pose2d(
                Units.inchesToMeters(46.0), 0.0, new Rotation2d(Units.degreesToRadians(10.0))
              ), 1.5, 0.1, 0.1
            )
          ),
          new IndexCommand(indexer, shooter),
          new CollectCommand(collector, drivetrain),
          new InstantCommand(
            ()->{
              HubData u = new HubData();
              hubTracking.sampleHubData(u);
              System.out.println("RANGE:"+u.range+"O"+u.area);
              new ShooterTargetCommand(shooter, u.range);
            }
          )
        ),

        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new InstantCommand(DashboardReadoutCommand::resetCounter),
            new DashboardReadoutCommand("Firing first 2 cargo"),
            new FeederLaunchCommand(indexer, feeder, shooter, 1.7),
            new WaitCommand(0.2),
            new InstantCommand(
              ()->{
                HubData u = new HubData();
                hubTracking.sampleHubData(u);
                System.out.println("RANGE:"+u.range+"O"+u.area);
              }
            ),
            new DashboardReadoutCommand("Driving to 3rd ball"),
            new AbsoluteDriveCommand(drivetrain,
              new Pose2d(
                5.222, -1.135, new Rotation2d(0.229)
              ), 2.5, 8.0,
              0.07, 0.1
            )
          ),
          new CollectCommand(collector, drivetrain)
        ),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new DashboardReadoutCommand("Index #1"),  
            new IndexCommand(indexer, shooter).withTimeout(2.0),
            new DashboardReadoutCommand("Load"),
            new LoadCommand(indexer, feeder, shooter),
            new DashboardReadoutCommand("Index #2"),
            (new IndexCommand(indexer, shooter)).withTimeout(2.0)
          ),
          new HumanPlayerSignalCommand(bling),
          new CollectCommand(collector, drivetrain)
        ),
        new DashboardReadoutCommand("Turning to face hub"),
          // new ShooterRangeTargetCommand(shooter, hubTracking),
        new WaitCommand(0.2),
        new AbsoluteDriveCommand(drivetrain,
          new Pose2d(
            2.30, -1.20, new Rotation2d(-0.048)
          ), 2.0, 5.5,
          0.1, 0.02
        ),
        new AlignToHub(drivetrain, hubTracking).withTimeout(0.5),
        new WaitCommand(0.3),
        new DashboardReadoutCommand("Firing second 2 cargo"),
        new InstantCommand(
          ()->{
            HubData u = new HubData();
            hubTracking.sampleHubData(u);
            System.out.println("RANGE:" + u.range + "; AZIMUTH: " + u.azimuth + "; AREA: " + u.area);
            new ShooterTargetCommand(shooter, u.range);
          }
        ),
        new WaitCommand(1.0),
        new FeederLaunchCommand(indexer, feeder, shooter),
        new WaitCommand(1.0),
        new DashboardReadoutCommand("Done!"),
        new ShooterSpinDownCommand(shooter)
      )
    );

    autoChooser.addOption("NEW-4Ball",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new ParallelCommandGroup(
            new AbsoluteDriveCommand(drivetrain,
              new Pose2d(
                1.984, 0.098, new Rotation2d(0.3198)
              ), 1.5, 0.1, 0.1
            )
          ),
          new IndexCommand(indexer, shooter),
          new CollectCommand(collector, drivetrain),
          new ShooterTargetCommand(shooter, 2.09)
        ),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new InstantCommand(DashboardReadoutCommand::resetCounter),
            new DashboardReadoutCommand("Firing first 2 cargo"),
            new FeederLaunchCommand(indexer, feeder, shooter, 1.7),
            new InstantCommand(
              ()->{
                HubData u = new HubData();
                hubTracking.sampleHubData(u);
                System.out.println("RANGE:"+u.range+"O"+u.area);
              }
            ),
            new DashboardReadoutCommand("Driving to 3rd ball"),
            new SequentialCommandGroup(
              new AbsoluteDriveCommand(drivetrain,
                new Pose2d(
                  4.511, -0.827, new Rotation2d(0.5108)
                ), 2.5, 8.0,
                0.07, 0.1,
                true
              ),
              new AbsoluteDriveCommand(drivetrain,
                new Pose2d(
                  5.844, -0.068, new Rotation2d(0.6466)
                ), 2.5, 8.0,
                0.15, 0.15
              )
            )
          ),
          new CollectCommand(collector, drivetrain)
        ),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new DashboardReadoutCommand("Index #1"),  
            new IndexCommand(indexer, shooter).withTimeout(1.5),
            new DashboardReadoutCommand("Load"),
            new LoadCommand(indexer, feeder, shooter),
            new DashboardReadoutCommand("Index #2"),
            (new IndexCommand(indexer, shooter)).withTimeout(1.5)
          ),
          new ShooterTargetCommand(shooter, 3.49),
          new HumanPlayerSignalCommand(bling),
          new CollectCommand(collector, drivetrain)
        ),
        new DashboardReadoutCommand("Turning to face hub"),
          // new ShooterRangeTargetCommand(shooter, hubTracking),
        new AbsoluteDriveCommand(drivetrain,
          new Pose2d(
            3.727, -0.733, new Rotation2d(0.2063)
          ), 2.0, 5.5,
          0.1, 0.02
        ),
        new WaitCommand(0.3),
        new DashboardReadoutCommand("Firing second 2 cargo"),
        new InstantCommand(
          ()->{
            HubData u = new HubData();
            hubTracking.sampleHubData(u);
            System.out.println("RANGE:" + u.range + "; AZIMUTH: " + u.azimuth + "; AREA: " + u.area);
          }
        ),
        new FeederLaunchCommand(indexer, feeder, shooter),
        new WaitCommand(1.0),
        new DashboardReadoutCommand("Done!"),
        new ShooterSpinDownCommand(shooter)
      )
    );

    autoChooser.addOption("Reading2Ball",
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new TurnCommand(drivetrain, Units.degreesToRadians(20.0), 1.0).andThen(new PrintCommand("Finished Turning")),
          new ShooterTargetCommand(shooter, 2.4).andThen(new PrintCommand("Finished Spinning Up"))
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(0.4).andThen(new DriveTranslateCommand(drivetrain, 1.5, 0.6)),
          new IndexCommand(indexer, shooter),
          new CollectCommand(collector, drivetrain)
        ),
        new InstantCommand(
          ()->{
            HubData u = new HubData();
            hubTracking.sampleHubData(u);
            System.out.println("[R2B] RANGE:"+u.range+"O"+u.area);
          }
        ),
        new FeederLaunchCommand(indexer, feeder, shooter,1.7),
        new WaitCommand(1.0),
        new ShooterSpinDownCommand(shooter)
      )
    );
    autoChooser.addOption("Auto-1Ball",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new AbsoluteDriveCommand(
            drivetrain,
            new Pose2d(2.0, 0.0, new Rotation2d(0.0)),
            2.0
          ),
          new ShooterTargetCommand(shooter, 2.4)
        ),
        new WaitCommand(0.5),
        new AlignToHub(drivetrain, hubTracking).withTimeout(1.0),
        (new ShooterRangeTargetCommand(shooter, hubTracking) {
          public void end(boolean interruptible) {
            return;
          }
        }).withTimeout(2.0),
        new FeederLaunchCommand(indexer, feeder, shooter, 2.0),
        new WaitCommand(1.0),
        new ShooterSpinDownCommand(shooter)
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
    (new Trigger(()->(OI.operatorController.getRightTriggerAxis()>0.5))).whenActive(
      new InstantCommand(OI::toggleOperatorMode)
    );
    (new Trigger(OI::isChangedToClimberMode)).whenActive(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new RunCommand(()->OI.setRumble(0.6)).withTimeout(0.2),
          new RunCommand(()->OI.setRumble(0.0)).withTimeout(0.2),
          new RunCommand(()->OI.setRumble(0.6)).withTimeout(0.2),
          new RunCommand(()->OI.setRumble(0.0)).withTimeout(0.2),
          new RunCommand(()->OI.setRumble(0.6)).withTimeout(0.2),
          new RunCommand(()->OI.setRumble(0.0)).withTimeout(0.2)
        ),
        new SequentialCommandGroup(
          new ShooterSpinDownCommand(shooter)
        )          
      )
    );
    (new Trigger(OI::isChangedToNormalMode)).whenActive(
      new SequentialCommandGroup(
        new RunCommand(()->OI.setRumble(1.0)).withTimeout(0.125),
        new RunCommand(()->OI.setRumble(0.0)).withTimeout(0.125),
        new RunCommand(()->OI.setRumble(1.0)).withTimeout(0.125),
        new RunCommand(()->OI.setRumble(0.0)).withTimeout(0.125)
      )
    );
    OI.whenNormalMode(OI.getOperatorDPadDown()).whenActive(
        new ShooterTargetCommand(shooter, 2.12)
    );
    OI.whenNormalMode(OI.getOperatorDPadLeft()).whenActive(
        new ShooterTargetCommand(shooter, 2.0)
    );
    OI.whenNormalMode(OI.getOperatorDPadUp()).whenActive(
        new ShooterTargetCommand(shooter, 3.0)
    );
    OI.whenNormalMode(OI.getOperatorDPadRight()).whenActive(
        new ShooterTargetCommand(shooter, 4.0)
    );
    OI.whenNormalMode(new JoystickButton(OI.operatorController, XboxController.Button.kLeftBumper.value)).whenPressed(
      autoCollectSequence
    );
    OI.whenNormalMode(new JoystickButton(OI.operatorController, XboxController.Button.kB.value)).whenPressed(
      new SequentialCommandGroup(
        new LoadCommand(indexer, feeder, shooter)
      )
    );
    OI.whenNormalMode(new JoystickButton(OI.operatorController,XboxController.Button.kBack.value)).whenPressed(
      new ShooterRangeTargetCommand(shooter, hubTracking)
    );
    OI.whenNormalMode(new JoystickButton(OI.operatorController, XboxController.Button.kY.value)).whenPressed(
      new FeederLaunchCommand(indexer, feeder, shooter)
    );
    OI.whenNormalMode(new JoystickButton(OI.operatorController, XboxController.Button.kRightBumper.value)).cancelWhenPressed(
      autoCollectSequence
    );
    OI.whenNormalMode(new JoystickButton(OI.operatorController, XboxController.Button.kRightBumper.value)).whenPressed(
      new ShooterSpinDownCommand(shooter)
    );
    OI.whenNormalMode(new JoystickButton(OI.operatorController, XboxController.Button.kStart.value)).whenPressed(
      new ShooterSpinUpCommand(shooter, 170, (0.25-0.08))
    );
    
    OI.whenNormalMode(OI.getOperatorDPadDown()).whenPressed(new DashboardReadoutCommand("HEY!!!!!!"));
    
    JoystickButton hubAlignButton = new JoystickButton(OI.driverController, 22);

    hubAlignButton.whileActiveOnce(
      new AlignToHub(drivetrain, hubTracking)
    );

    hubAlignButton.whenPressed(
      new SequentialCommandGroup(
        new RunCommand(()->{
          OI.operatorController.setRumble(RumbleType.kLeftRumble, 0.9);
          OI.operatorController.setRumble(RumbleType.kRightRumble, 0.9);
        }).withTimeout(0.1),
        new RunCommand(()->{
          OI.operatorController.setRumble(RumbleType.kLeftRumble, 0);
          OI.operatorController.setRumble(RumbleType.kRightRumble, 0);
        }).withTimeout(0.1)
      )
    );

    hubAlignButton.whenReleased(
        () -> SmartDashboard.putBoolean("AlignToHub on", false)
    );

    SmartDashboard.putData(new HoodIndexCommand(shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    fullAuto = 
    new SequentialCommandGroup(
      new HoodIndexCommand(shooter),
      new WaitCommand(SmartDashboard.getNumber("Init/Auto Delay", 0)),
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
