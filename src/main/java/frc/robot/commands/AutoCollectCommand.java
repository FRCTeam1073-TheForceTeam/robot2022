// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCollectCommand extends ParallelDeadlineGroup 
{
  /** Creates a new AutoCollectCommand. */
  public AutoCollectCommand(Drivetrain drivetrain, Collector collector, Indexer indexer, Shooter shooter) 
  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new WaitCommand(0.4).andThen(new DriveTranslateCommand(drivetrain, 1.5, 0.6)),
      new CollectCommand(collector, drivetrain).alongWith(
        new IndexCommand(indexer, shooter)
      )
    );
  }
}
