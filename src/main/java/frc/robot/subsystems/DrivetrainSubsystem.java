// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
	public static final double kMaxSpeed = 3.0; // 3 meters per second
	public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

	private final Translation2d m_frontLeftLocation = new Translation2d(0.417, 0.417);
	private final Translation2d m_frontRightLocation = new Translation2d(0.417, -0.417);
	private final Translation2d m_backLeftLocation = new Translation2d(-0.417, 0.417);
	private final Translation2d m_backRightLocation = new Translation2d(-0.417, -0.417);

	private final SwerveModule m_frontLeft = new SwerveModule(1, 2);
	private final SwerveModule m_frontRight = new SwerveModule(3, 4);
	private final SwerveModule m_backLeft = new SwerveModule(5, 6);
	private final SwerveModule m_backRight = new SwerveModule(7, 8);

	private final AHRS m_gyro = new AHRS(Port.kMXP);

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
			m_kinematics,
			m_gyro.getRotation2d());

	private SwerveModuleState[] m_swerveModuleStates = new SwerveModuleState[] {};

	public DrivetrainSubsystem() {
		m_gyro.reset();
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates = m_kinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		m_swerveModuleStates = swerveModuleStates;

		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);
	}

	public void driveVoltage(double voltage) {
		m_frontLeft.driveVoltage(voltage);
		m_frontRight.driveVoltage(voltage);
		m_backLeft.driveVoltage(voltage);
		m_backRight.driveVoltage(voltage);
	}

	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		m_odometry.update(
				m_gyro.getRotation2d(),
				m_swerveModuleStates);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("FRA-Actual", m_frontRight.getTurnEncoderDistance());
		SmartDashboard.putNumber("FLA-Actual", m_frontLeft.getTurnEncoderDistance());
		SmartDashboard.putNumber("BRA-Actual", m_backRight.getTurnEncoderDistance());
		SmartDashboard.putNumber("BLA-Actual", m_backLeft.getTurnEncoderDistance());

		SmartDashboard.putNumber("FRA-Setpoint", m_frontRight.getAngleSetpoint());
		SmartDashboard.putNumber("FLA-Setpoint", m_frontLeft.getAngleSetpoint());
		SmartDashboard.putNumber("BRA-Setpoint", m_backRight.getAngleSetpoint());
		SmartDashboard.putNumber("BLA-Setpoint", m_backLeft.getAngleSetpoint());
	}
}