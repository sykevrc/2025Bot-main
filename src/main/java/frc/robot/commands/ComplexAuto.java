package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ComplexAuto extends SequentialCommandGroup{
    public ComplexAuto() {
    addCommands(
      new Coral4Command(),
      new DriveDistance(),
      new AutoAlignLeftCommand(),
      new Reverse(),
      new CoralHumanCommand()
    );
  }
}
