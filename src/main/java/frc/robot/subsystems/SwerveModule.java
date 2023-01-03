// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.SwerveModulePosition;

public class SwerveModule {
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = 2048;
    // TODO: Consider gear ratio's?
    private static final double kEncoderConstant = (1 / kEncoderResolution) * kWheelRadius * Math.PI;

    private static final double kModuleMaxAngularVelocity = DrivetrainSubsystem.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_turningMotor;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel      PWM output for the drive motor.
     * @param turningMotorChannel    PWM output for the turning motor.
     * @param driveEncoderChannelA   DIO input for the drive encoder channel A
     * @param driveEncoderChannelB   DIO input for the drive encoder channel B
     * @param turningEncoderChannelA DIO input for the turning encoder channel A
     * @param turningEncoderChannelB DIO input for the turning encoder channel B
     */
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            int driveEncoderChannelA,
            int driveEncoderChannelB,
            int turningEncoderChannelA,
            int turningEncoderChannelB) {
        m_driveMotor = new WPI_TalonFX(driveMotorChannel);
        m_turningMotor = new WPI_TalonFX(turningMotorChannel);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.

        // Set the distance (in this case, angle) in radians per pulse for the turning
        // encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        // TODO: m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveEncoderDistance(), new Rotation2d(getTurnEncoderDistance()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDriveEncoderDistance(), new Rotation2d(getTurnEncoderDistance()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(getTurnEncoderDistance()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(getDriveEncoderRate(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(getTurnEncoderDistance(),
                state.angle.getRadians());

        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    }

    public void driveVoltage(double voltage) {
        m_driveMotor.setVoltage(voltage);
        m_turningMotor.setVoltage(voltage);
    }

    /**
     * Get the drive motor encoder's rate
     * 
     * @return the drive motor encoder's rate
     */
    private double getDriveEncoderRate() {
        return m_driveMotor.getSelectedSensorVelocity(0) * kEncoderConstant * 10;
    }

    /**
     * Get the drive motor encoder's distance
     * 
     * @return the drive motor encoder's distance
     */
    private double getDriveEncoderDistance() {
        return m_driveMotor.getSelectedSensorPosition(0) * kEncoderConstant;
    }

    /**
     * Get the turn motor angle
     * 
     * @return get the turn motor angle
     */
    private double getTurnEncoderDistance() {
        return m_turningMotor.getSelectedSensorPosition(0) * kEncoderConstant;
    }
}