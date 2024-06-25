package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

	public static final class SwerveModuleConstants {

		// Max speed of the drive motor
		public static final double kMaxSpeed = 4.0; // meters per second

		// Ratio drive motor and turn motor
		public static final double kDriveGearRatio = 6.12;
		public static final double kTurningGearRatio = 12.8;

		// Wheel diameter
		public static final double kWheelDiameter = Units.inchesToMeters(4.0);

		// Physical conversion constants
		public static final double kRotationToMeter = Math.PI * kWheelDiameter / kDriveGearRatio;
		public static final double kRPMToMeterPerSecond = kRotationToMeter / 60.0;

		public static final double kRotationToDegree = 360.0 / kTurningGearRatio;
		public static final double kRPMToDegreePerSecond = kRotationToDegree / 60.0;

	}

	public static final class SwerveChassisConstants {

		public static final double kTrackWidth = Units.inchesToMeters(23.0);
		public static final double kWheelBase = Units.inchesToMeters(23.0);

		// Kinematics
		public static final Translation2d[] kModuleLocation = {
				new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // Front left
				new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // Front right
				new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) };
	}

}
