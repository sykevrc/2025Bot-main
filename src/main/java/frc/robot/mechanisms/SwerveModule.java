// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.tools.parts.PIDGains;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SwerveModule {
	/** Creates a new SwerveModule. */
	//private final CANBus kCANBus = new CANBus();

	private final SparkFlex driveMotor;
	private SparkFlexSim driveFlexSim = null;
	private final SparkMax turningMotor;
	private SparkMaxSim turningMaxSim = null;
	private final CANcoder cancoder;
	private final RelativeEncoder driveEncoder;

	private final SparkClosedLoopController sparkDrivePID;
	private final ProfiledPIDController m_turningPIDController;

	public final double angleZero;

	private final String moduleName;
	private Rotation2d _simulatedAbsoluteEncoderRotation2d = null;

	private double m_moduleAngleRadians;
	private Rotation2d m_moduleAngleRotation2d = new Rotation2d();
	//private SwerveModuleState optimizedState;
	private double angularPIDOutput;
	private double angularFFOutput;
	private double turnOutput;
	private boolean isSim = false;
	private ShuffleboardTab swerveTab = null;
	//private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

	SparkMaxConfig turnConfig = null;
	SparkFlexConfig driveConfig = null;
	
	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.ksTurning, ModuleConstants.kvTurning);

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			PIDGains angularPID,
			PIDGains drivePID,
			boolean invertTurningMotor,
			boolean invertDriveMotor
			) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		if(RobotBase.isReal()) {
			isSim = false;
		} else {
			isSim = true;
			_simulatedAbsoluteEncoderRotation2d = new Rotation2d(0.0);
		}

		// Initialize the motors
		driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);

		if(isSim) {
			driveFlexSim = new SparkFlexSim(driveMotor, DCMotor.getNeoVortex(1));
		}
		
		turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

		if(isSim) {
			turningMaxSim = new SparkMaxSim(turningMotor, DCMotor.getNEO(1));
		}

		cancoder = new CANcoder(absoluteEncoderPort, Constants.kCanivoreCANBusName);
		//cancoder = new CANcoder(absoluteEncoderPort);
		cancoder.clearStickyFaults();

		driveConfig = new SparkFlexConfig();

		driveConfig
            .inverted(invertDriveMotor)
            .idleMode(IdleMode.kCoast);

        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(
				drivePID.kP,
				drivePID.kI,
				drivePID.kD
			);

        driveConfig.signals.primaryEncoderPositionPeriodMs(5);

		// Set the gear ratio
		driveConfig.encoder
		.positionConversionFactor(
			ModuleConstants.kdriveGearRatioL3 * ModuleConstants.kwheelCircumference
			//Divide the circumference by the gear ratio to get your conversion calculation
			//ModuleConstants.kwheelCircumference / ModuleConstants.kdriveGearRatioL3 
		)
		.velocityConversionFactor(
			ModuleConstants.kdriveGearRatioL3
			* ModuleConstants.kwheelCircumference
			//ModuleConstants.kwheelCircumference / ModuleConstants.kdriveGearRatioL3 
			* (1d / 60d)
		);

        driveMotor.configure(
			driveConfig, 
			ResetMode.kResetSafeParameters, 
			PersistMode.kPersistParameters
		);

		this.sparkDrivePID = driveMotor.getClosedLoopController();

		driveEncoder = driveMotor.getEncoder();

		turnConfig = new SparkMaxConfig();

		turnConfig
            .inverted(invertTurningMotor)
            .idleMode(IdleMode.kCoast);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(
				Constants.DriveConstants.kGyroTurningGains.kP, 
				Constants.DriveConstants.kGyroTurningGains.kI, 
				Constants.DriveConstants.kGyroTurningGains.kD
			);
        turnConfig.signals.primaryEncoderPositionPeriodMs(5);

        turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		m_turningPIDController = new ProfiledPIDController(
			angularPID.kP,
			angularPID.kI,
			angularPID.kD,
			new TrapezoidProfile.Constraints( // radians/s?
					2 * Math.PI * 600, // theoretical is 5676 RPM -> 94*2pi
					2 * Math.PI * 1200));

		m_turningPIDController.enableContinuousInput(0, Math.toRadians(360));

		if(Constants.kDebugDriveTrain == true) {
			// Swerve tab stuff
			swerveTab = Shuffleboard.getTab("Swerve");
			swerveTab.addDouble(moduleName + " Absolute", this::getAbsoluteHeading);
			swerveTab.addDouble(moduleName + " Meters", this::getDistanceMeters);
			swerveTab.addString(moduleName + " Abs. Status", this::getStatus);
		}
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return (cancoder.getAbsolutePosition().refresh().getValueAsDouble() * 360);
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		m_moduleAngleRadians = Math.toRadians(cancoder.getAbsolutePosition(true).getValueAsDouble() * 360.0);

		if(isSim){
			m_moduleAngleRotation2d = _simulatedAbsoluteEncoderRotation2d;
			return new SwerveModulePosition(driveEncoder.getPosition(), _simulatedAbsoluteEncoderRotation2d);
		}

		m_moduleAngleRotation2d = Rotation2d.fromDegrees(cancoder.getAbsolutePosition(true).getValueAsDouble() * 360.0);

		return new SwerveModulePosition(driveEncoder.getPosition(), m_moduleAngleRotation2d);
	}

	// Sets the position of the swerve module
	public void setDesiredState(SwerveModuleState desiredState) {	

		if(isSim) {
			m_moduleAngleRadians = Math.toRadians(desiredState.angle.getDegrees());
			_simulatedAbsoluteEncoderRotation2d = desiredState.angle;
		}

		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		desiredState.optimize(m_moduleAngleRotation2d);

		angularPIDOutput = m_turningPIDController.calculate(m_moduleAngleRadians,
			desiredState.angle.getRadians());

		angularFFOutput = turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity);

		turnOutput = angularPIDOutput + angularFFOutput;

		turningMotor.setVoltage(turnOutput);
		
		if(isSim) {
			sparkDrivePID.setReference(
				desiredState.speedMetersPerSecond,
				ControlType.kVelocity
			);

			driveFlexSim.iterate(
				desiredState.speedMetersPerSecond,
        		RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        		0.02
			);

		} else {
			sparkDrivePID.setReference(
				desiredState.speedMetersPerSecond,
				ControlType.kVelocity
				//ControlType.kMAXMotionVelocityControl
			);
		}

		if(Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("Motors/DriveMotorCurrentOutput_" + moduleName, driveMotor.getOutputCurrent());
			Logger.recordOutput("Motors/DriveMotorTemp_" + moduleName, driveMotor.getMotorTemperature());
			Logger.recordOutput("Motors/TurnMotorCurrentOutput_" + moduleName, turningMotor.getOutputCurrent());
			Logger.recordOutput("Motors/TurnMotorTemp_" + moduleName, turningMotor.getMotorTemperature());
		}
	}

	public void resetEncoders() {
		
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turningMotor.stopMotor();
	}

	double getAngleZero() {
		return this.angleZero;
	}

	String getStatus() {
		return cancoder.getMagnetHealth().getValue().name();
	}

	public void setTurningPID(double p, double i, double d) {
		m_turningPIDController.setPID(p, i, d);
	}

	public void simulatePeriodic() {
		
	}

	public void setDrivePID(double p, double i, double d) {
		driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(
				p,
				i,
				d
			);
	}
}
