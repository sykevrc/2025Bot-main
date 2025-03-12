package frc.robot;

import java.util.OptionalLong;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.tools.JoystickUtils;
import frc.robot.tools.Limelight;
import frc.robot.tools.PhotonVision;
import frc.robot.tools.parts.PathBuilder;
import frc.robot.mechanisms.LED;
import frc.robot.mechanisms.LED.LEDStatus;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Algae1Command;
import frc.robot.commands.Algae2Command;
import frc.robot.commands.AlgaeFloorCommand;
import frc.robot.commands.ArmStartCommand;
import frc.robot.commands.AutoAlignLeftCommand;
import frc.robot.commands.AutoAlignRightCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.Coral1Command;
import frc.robot.commands.Coral2Command;
import frc.robot.commands.Coral3Command;
import frc.robot.commands.Coral4Command;
import frc.robot.commands.CoralHumanCommand;
import frc.robot.commands.DriveBackwardsCommand;
import frc.robot.commands.EjectAlgaeCommand;
import frc.robot.commands.EjectCoralNoCheck;
import frc.robot.commands.EjectCoralReverse;
import frc.robot.commands.ElevatorStartCommand;
import frc.robot.commands.EndEffectorStopCommand;
import frc.robot.commands.IntakeNoWait;
import frc.robot.commands.ResetElevatorEncoder;
import frc.robot.commands.FailsafeCoralCommand;
import frc.robot.commands.ResetPositionCommand;
import frc.robot.commands.autonomous.AutoAlignLeftAutoCommand;
import frc.robot.commands.autonomous.AutoAlignRightAutoCommand;
import frc.robot.commands.autonomous.DelayCommand;
import frc.robot.commands.autonomous.EjectCoralCommand;
import frc.robot.commands.autonomous.IntakeCoralWaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

	public static final Field2d field = new Field2d();
	public static final PhotonVision photonVision = new PhotonVision();
	public static final Limelight limelight = new Limelight();
	public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public static final ArmSubsystem armSubsystem = new ArmSubsystem();
	public static final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
	public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	public static final LED led1 = new LED(0);

	// This is required by pathplanner
	// public final static PathBuilder autoBuilder = new PathBuilder();

	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandXboxController operatorController = new CommandXboxController(1);
	private final Command m_simpleAuto = new DriveDistance(
			AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed, m_robotDrive);

	// A complex auto routine that drives forward, drops a hatch, and then drives
	// backward.
	private final Command m_complexAuto = new ComplexAuto();

	// A chooser for autonomous commands
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer(boolean isSim) {

		// Configure the trigger bindings
		configureBindings();
		m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
		m_chooser.addOption("Complex Auto", m_complexAuto);
	
		// Put the chooser on the dashboard
		Shuffleboard.getTab("Autonomous").add(m_chooser);
	}

	private void configureBindings() {

		if (RobotBase.isReal()) {
			// Real, not a simulation

			// Coral stuck on battery command - repeat command if coral is still stuck.
			driverController.button(2).whileTrue(new FailsafeCoralCommand());

			// Coral Commands
			operatorController.button(3).onTrue(new SequentialCommandGroup(
					new ArmStartCommand(),
					new WaitCommand(0.15),
					new Coral1Command()));
			operatorController.button(4).onTrue(new SequentialCommandGroup(
					new ArmStartCommand(),
					new WaitCommand(0.15),
					new Coral2Command()));
			operatorController.button(2).whileTrue(new Coral3Command());
			operatorController.button(10).onTrue(new SequentialCommandGroup(
					new ArmStartCommand(),
					new WaitCommand(0.15),
					new Coral4Command()));

			// Intake coral from human element
			operatorController.button(8).whileTrue(new CoralHumanCommand());

			// Algae Commands
			// This is for the floor algae, press again to stop
			operatorController.button(9).whileTrue(new AlgaeFloorCommand());

			// Press again to stop
			operatorController.leftTrigger().whileTrue(new EjectAlgaeCommand());

			/*
			 * operatorController.leftTrigger().whileTrue(new SequentialCommandGroup(
			 * new ArmStartCommand()
			 * ));
			 */

			// these are to get the algae off of the reef
			operatorController.povDown().whileTrue(new Algae1Command());
			operatorController.povUp().whileTrue(new Algae2Command());

			// Driver to reset field oriented drive
			driverController.button(8).whileTrue(new ResetPositionCommand());
			operatorController.button(7).whileTrue(new ResetElevatorEncoder());

			// Auto align for the coral
			driverController.leftBumper().whileTrue(new AutoAlignLeftCommand());
			driverController.rightBumper().whileTrue(new AutoAlignRightCommand());

			// driverController.button(3).whileTrue(new StartCommand());
			operatorController.button(1).onTrue(new SequentialCommandGroup(
					new EndEffectorStopCommand(),
					new ElevatorStartCommand(),
					new WaitCommand(0.05),
					new ArmStartCommand()

			));

			// driverController.rightBumper().whileTrue(new RunCommand(() -> new
			// SequentialCommandGroup(new IntakeNoWait(), new StopIntake()).execute()));

			// operatorController.rightTrigger().onTrue(getAutonomousCommand())

			operatorController.rightTrigger().whileTrue(// new SequentialCommandGroup(
					new EjectCoralNoCheck(0.6)
			// new EjectCoralCommand()//,
			// new DriveBackwardsCommand()
			// new ArmStartCommand(), // this will retract the arm and stop end effector
			// new StartCommand() // this will retract the arm and move the elevator down
			// )
			);

			// Climber Stuff
			operatorController.rightBumper().whileTrue(new ClimberUpCommand());
			operatorController.leftBumper().whileTrue(new ClimberDownCommand());

			// Move the end effector wheels freely
			/*
			 * operatorController.axisGreaterThan(1, 0.2).whileTrue(
			 * new EjectCoralNoCheck(operatorController.getRightX())
			 * );
			 */

			// driverController.rightTrigger().whileFalse(new StopEjectCoralCommand());

			// driverController.button(1).whileTrue(new RunCommand(() -> new
			// ResetPositionCommand()));

			// driverController.button(1).whileTrue(new RunCommand(() ->
			// sliderSubsystem.setDesiredState(ElevatorSubsystem.ElevatorState.Start)));

			// Swerve Drive method is set as default for drive subsystem
			driveSubsystem.setDefaultCommand(

					new RunCommand(() -> driveSubsystem.drive(
							JoystickUtils.processJoystickInput(driverController.getLeftY()),
							JoystickUtils.processJoystickInput(driverController.getLeftX()),
							JoystickUtils.processJoystickInput(-driverController.getRightX())),
							driveSubsystem));

		} else {

			// This is for simulation

			// driverController.button(1).whileTrue(new AutoAlignRightCommand());
			driverController.button(1).whileTrue(
					// new EjectCoralNoCheck(0.0)
					// new Algae1Command()
					new SequentialCommandGroup(
							// new RunCommand(() ->
							// endEffectorSubsystem.setDesiredState(EndEffectorState.IntakeHoldAlgae)),
							new EndEffectorStopCommand(),
							new ArmStartCommand(),
							new ElevatorStartCommand()));

			// Swerve Drive method is set as default for drive subsystem
			driveSubsystem.setDefaultCommand(

					// new RunCommand(() -> driveSubsystem.drive(
					new RunCommand(() -> driveSubsystem.driveRobotRelative(
							JoystickUtils.processJoystickInput(driverController.getLeftY()),
							JoystickUtils.processJoystickInput(driverController.getLeftX()),
							JoystickUtils.processJoystickInput(-driverController.getRawAxis(2))),
							driveSubsystem));
		}
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}