// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends Command {
  private DriveSubsystem driveSubsystem;
  private boolean finished = false;
	private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public DriveDistance() {
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    finished = false;
    driveSubsystem.resetEncoders();
    driveSubsystem.driveRobotRelative(0.0, 0.0, 0.0);
  }

  @Override
  public void execute() {
    while(gyro.getWorldLinearAccelX() < 0.05){
    driveSubsystem.driveRobotRelative(0.1, 0.0, 0.0);
    
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}