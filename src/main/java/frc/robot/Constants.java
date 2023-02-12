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
    public static final class ModuleConstants{
        // Physical wheel constants
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * (WHEEL_DIAMETER_METERS / 2);

        // Gear ratio
        public static final double TURNING_RATIO = (50.0 / 14.0) * (60.0 / 10.0);
        public static final double DRIVE_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);

        // PID constants
        public static final double MODULE_KP = 0.26;
        public static final double MODULE_KD = 3;
        public static final double POSITION_CONVERSION_FACTOR = (Math.PI * 2) / TURNING_RATIO;
    }

    public static final class DriveConstants {
        // Can ID ports
        public static final int[] MOD_FR_CANS = {3, 4, 5};
        public static final int[] MOD_FL_CANS = {6, 7, 8};
        public static final int[] MOD_BL_CANS = {9, 10, 11};
        public static final int[] MOD_BR_CANS = {12, 13, 14};
        public static final int GYRO_CAN = 15;

        //Thanos Offsets
        public static final double MOD_FR_OFFSET = CURRENT_MODE == Mode.THANOS ? 198.8 : -115.6;
        public static final double MOD_FL_OFFSET = CURRENT_MODE == Mode.THANOS ? 145.9 : 180 + 5.4;
        public static final double MOD_BR_OFFSET = CURRENT_MODE == Mode.THANOS ? 263.1 : 161.0;
        public static final double MOD_BL_OFFSET = CURRENT_MODE == Mode.THANOS ? 254.1 : 180 + 177.8;
        // Competition Offset

        // Kinematics
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(20.733);
        
        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(20.733);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
    }

    public static class WristConstants {
        public static final int WRIST_ID = 19;
        public static final int INTAKE_ID = 20;

        public static final int LIMIT_SWITCH_PORT = 2;
        public static final int WRIST_ANGLE_PORT = 21;
        
        public static final double WRIST_PIVOT_RATIO = 2.6666;

        // Plus 80 degrees
        public static final double WRIST_LOWER_LIMIT = 0;

        public static final double WRIST_UPPER_LIMIT = 125.0;
        public static final int TOF_PORT = 23;
        public static final double WRIST_KP = 0.05;
        public static final double WRIST_KI = 0.0;
        public static final double WRIST_KD = 0.0;
    }


    public static class AutoConstants {
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

    public static final Mode CURRENT_MODE = Mode.HELIOS;

    public static class ArmConstants {
        public static final int ARM_EXTENSION_ID = 18;
        public static final int ARM_ANGLE_ID_MASTER = 16;
        public static final int ARM_ANGLE_ID_FOLLOWER = 17;
        public static final int LIMIT_SWITCH_PORT = 3;

        public static final double KP_ANGLE = 0.23;
        public static final double KI_ANGLE = 0.005;
        public static final double KD_ANGLE = 0.0075;

        public static final double ARM_EXT_KP = 0.1;
        public static final double ARM_EXT_KI = 0;
        public static final double ARM_EXT_KD = 0;
        
        public static final double K_REVERSE_LIMIT = 45;
        public static final double K_FORWARD_LIMIT = 300;

        public static final int ENCODER_PORT = 1;

        public static final double SPROCKET_DIAMETER = 1.99;
        public static final double EXTENSION_RATIO = (1.0/25.0) * (SPROCKET_DIAMETER * Math.PI);

        public static final double EXT_LOWER_LIMIT = 0.5;
        public static final double EXT_HIGHER_LIMIT = 25;

    }

    public static final int DRIVER_PORT = 0;

    public enum Mode {
        /** Running on the test bot */
        THANOS,

        /**Running on the competition bot */
        HELIOS,

        /** Running a physics simulator */
        SIM,

        /**Replaying from a log file */
        REPLAY
    }
}
