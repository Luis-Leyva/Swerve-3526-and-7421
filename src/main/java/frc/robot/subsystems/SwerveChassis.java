// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveChassis extends SubsystemBase {

	// Gyroscope
	Pigeon2 gyro = new Pigeon2(13);

	// Swerve Modules
	SwerveModule frontLeft = new SwerveModule(1, 2, 3, 0.0, "Front Left", "CANivore");
	SwerveModule frontRight = new SwerveModule(4, 5, 6, 0.0, "Front Right", "CANivore");
	SwerveModule backLeft = new SwerveModule(7, 8, 9, 0.0, "Back Left", "CANivore");
	SwerveModule backRight = new SwerveModule(10, 11, 12, 0.0, "Back Right", "CANivore");

	// Swerve Kinematics
	SwerveDriveKinematics kinematics;

	// Swerve Odometry
	SwerveDrivePoseEstimator odometry;

	// Swerve Module Positions
	SwerveModulePosition[] modulePositions;

	// Desired Speeds
	ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	/** Creates a new SwerveChassis. */
	public SwerveChassis() {
		kinematics = new SwerveDriveKinematics(Constants.SwerveChassisConstants.kModuleLocation);

		modulePositions = new SwerveModulePosition[] {
				frontLeft.getModulePosition(),
				frontRight.getModulePosition(),
				backLeft.getModulePosition(),
				backRight.getModulePosition()
		};

		odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), modulePositions, new Pose2d());
	}

	// GetModulePosition
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				frontLeft.getModulePosition(),
				frontRight.getModulePosition(),
				backLeft.getModulePosition(),
				backRight.getModulePosition()
		};
	}

	// GetModuleStates
	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
				frontLeft.getActualState(),
				frontRight.getActualState(),
				backLeft.getActualState(),
				backRight.getActualState()
		};
	}

	// Get the current pose of the robot
	public Pose2d getPose() {
		return odometry.getEstimatedPosition();
	}

	// Reset the odometry to a specified pose
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
	}

	// Reset odometry angle
	public void resetAngle(double angle) {
		Pose2d actualPose = getPose();
		Pose2d newPose = new Pose2d(actualPose.getTranslation(), new Rotation2d(angle));

		resetOdometry(newPose);
	}

	// Add vision measurements to the pose estimator
	public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
		odometry.addVisionMeasurement(visionPose, timestamp);
	}

	// Drive Robot Relative
	public void driveRobotRelative(ChassisSpeeds speeds) {
		desiredSpeeds = speeds;
	}

	// Drive Robot Field Relative
	public void driveFieldRelative(ChassisSpeeds speeds) {
		ChassisSpeeds tempSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
		ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.discretize(tempSpeeds, 0.20);

		driveRobotRelative(fieldRelativeSpeeds);
	}

	// Get robot relative speeds
	public ChassisSpeeds getRobotRelativeSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	// Set the desired states of the swerve modules
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		backLeft.setDesiredState(desiredStates[2]);
		backRight.setDesiredState(desiredStates[3]);
	}

	// Update Odometry
	public void updateOdometry() {
		odometry.update(gyro.getRotation2d(), getModulePositions());
	}

	public void updateShuffleboard() {

	}

	@Override
	public void periodic() {
		updateOdometry();

		SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(desiredSpeeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveModuleConstants.kMaxSpeed);

		setModuleStates(desiredStates);

		updateShuffleboard();

	}
}
