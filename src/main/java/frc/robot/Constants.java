// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /*Constants for physical aspects of the modules, plus PID loops constants*/
    public final static class ModuleConstants{
        // Physical wheel constants
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * (WHEEL_DIAMETER_METERS / 2);
        // public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(12);

        // Gear ratio
        public static final double TURNING_RATIO = (50.0 / 14.0) * (60.0 / 10.0);
        public static final double DRIVE_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);

        // PID constants
        public static final double MODULE_KP = 0.16;
        public static final double MODULE_KD = 3;
        public static final double POSITION_CONVERSION_FACTOR = (Math.PI * 2) / TURNING_RATIO;
    }

    public static final class DriveConstants{
        // Can ID ports
        public static final int[] MOD_FR_CANS = {3, 4, 5};
        public static final int[] MOD_FL_CANS = {6, 7, 8};
        public static final int[] MOD_BL_CANS = {9, 10, 11};
        public static final int[] MOD_BR_CANS = {12, 13, 14};
        public static final int GYRO_CAN = 15;

        //Thanos Offsets
        public static final double MOD_FR_OFFSET = CURRENT_MODE == Mode.THANOS ? 198.8: 357.803;//360 - 160.400;
        public static final double MOD_FL_OFFSET = CURRENT_MODE == Mode.THANOS ? 145.9 : 349.629;//360 - 215.508;
        public static final double MOD_BR_OFFSET = CURRENT_MODE == Mode.THANOS ? 263.1 : 180 + 46.143;//360 - 105.820; 70.488
        public static final double MOD_BL_OFFSET = CURRENT_MODE == Mode.THANOS ? 254.1 : 180 + 70.488;//360 - 97.119; 96.943 149.6
        // Competition Offsets
        // TODO competition offsets

        // Kinematics
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(20.733);
        
        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(20.733);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, kTrackWidth / 2),
        new Translation2d(WHEEL_BASE / 2, -kTrackWidth / 2),
        new Translation2d(-WHEEL_BASE / 2, kTrackWidth / 2),
        new Translation2d(-WHEEL_BASE / 2, -kTrackWidth / 2));
    }

    public static class IntakeConstants{
        public static final int WRIST_ID = 19;
        public static final int INTAKE_ID = 20;

        public static final int LIMIT_SWTICH_PORT = 2;
        public static final int WRIST_ANGLE_PORT = 3;
    }

    public static class ArmConstants{
        public static final int ANGLE_MASTER_ID = 16;
        public static final int ANGLE_FOLLOWER_ID = 17;
        public static final int EXTENTION_ID = 18;
    }

    public static class AutoConstants{
        //Trajectory following values
        public static final double MAX_VELOCITY_PERCENT_OUTPUT = 0.5;
        public static final double MAX_ACCELERATION_PERCENT_OUTPUT = 0.5;

        public static final Constraints THETA_CONSTRAINTS = 
            new Constraints(MAX_VELOCITY_PERCENT_OUTPUT, MAX_ACCELERATION_PERCENT_OUTPUT);

        public static final ProfiledPIDController THETA_CONTROLLER = 
            new ProfiledPIDController(0.0, 0.0, 0.0, THETA_CONSTRAINTS);
        
        public static final PIDController CONTROLLER_X =
            new PIDController(0.5, 0, 0);
        public static final PIDController CONTROLLER_Y =
            new PIDController(0.5, 0, 0);
        
        //Auto balance constants
        public static final double BALANCE_P = 0.5;
        public static final double DESIRED_BALANCE_ANGLE = 1;
    }

    public static class ArmConstants{
        public static final int ArmExID = 17;
        public static final int ArmAngleID = 16;

        public static final double kAngleConversionFactor = (0.5/3462.87) * 360 * 8192;

        public static final double kPAngle = 0.13;
        public static final double kIAngle = 0.005;
        public static final double kDAngle = 0.0075;

        // public static final double kVAngle = 0.0;
        // public static final double kGAngle = 0.15;
        
        public static final double kReverseLimit = 40;
        public static final double kForwardLimit = 280;
    }

    public static final Mode currentMode = Mode.THANOS;
    public static final int driverPort = 0;

    public static enum Mode {
        /** Running on the test bot */
        THANOS,

        /**Running on the competition bot */
        ALPHA,

        /** Running a physics simulator */
        SIM,

        /**Replaying from a log file */
        REPLAY
    }
}
