// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.gos.lib.properties.GosDoubleProperty;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.supersystems.ArmPose;


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
        public static final double POSITION_CONVERSION_FACTOR = ((Math.PI * 2) / TURNING_RATIO);

        public static final double MODULE_KS = 0.0;
        public static final double MODULE_KV = 1.75;
        public static final double MODULE_KA = 0.0;
        public static final double MODULE_DRIVE_KP = 0.5;

        public static final double MAX_SPEED_MPS = 3.048;
    }

    public static final class DriveConstants {
        // Can ID ports
        public static final int[] MOD_FR_CANS = {3, 4, 5};
        public static final int[] MOD_FL_CANS = {6, 7, 8};
        public static final int[] MOD_BL_CANS = {9, 10, 11};
        public static final int[] MOD_BR_CANS = {12, 13, 14};
        public static final int GYRO_CAN = 15;

        //Thanos Offsets
        public static final double MOD_FR_OFFSET = 110.8;
        public static final double MOD_FL_OFFSET = 173.6 ;
        public static final double MOD_BR_OFFSET = -163.0;
        public static final double MOD_BL_OFFSET = 0.610;
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

        // Camera constants
        public static final Transform3d FRONT_CAM_POSE = new Transform3d(
                new Translation3d(Units.inchesToMeters(12.0), 0.0, Units.inchesToMeters(8.00)),
                new Rotation3d(Math.PI, 0.0, 0.0));
        public static final Transform3d LEFT_CAM_POSE = new Transform3d(
                new Translation3d(Units.inchesToMeters(4.0), Units.inchesToMeters(6.0), Units.inchesToMeters(25.0)),
                new Rotation3d(Units.degreesToRadians(90.0), 0.0, Units.degreesToRadians(90.0)));
        public static final String FRONT_CAM_NAME = "FrontPiCam";
        public static final String LEFT_CAM_NAME = "LeftWebCam";

        public static final GosDoubleProperty CAM_AMBIGUITY_THRESHOLD
                = new GosDoubleProperty(false, "Camera ambiguity threshold", 0.3);
        public static final GosDoubleProperty CAM_DISTANCE_THRESHOLD
                = new GosDoubleProperty(false, "Camera distance threshold", 7);
    }

    public static class WristConstants {
        public static final int WRIST_ID = 20;
        public static final int INTAKE_ID = 19;

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

        public static final double GROUND_OFFSET = 0.2;

        public static final double INTAKE_LENGTH = 0;
    }


    public static class AutoConstants {
        //Trajectory following values
        public static final double MAX_VELOCITY_PERCENT_OUTPUT = 2.0;
        public static final double MAX_ACCELERATION_PERCENT_OUTPUT = 1.0;

        public static final PIDController THETA_CONTROLLER =
                new PIDController(1.0, 0.0, 0.0);

        public static final PIDController CONTROLLER_X =
            new PIDController(2.5, 0, 0);
        public static final PIDController CONTROLLER_Y =
            new PIDController(2.5, 0, 0);

        public static final PIDConstants CONSTANTS_X =
                new PIDConstants(2.5, 0, 0);

        public static final PIDConstants THETA_CONSTANTS =
                new PIDConstants(1.0, 0.0, 0.0);
        
        //Auto balance constants
        public static final double BALANCE_P = -0.2;
        public static final double DESIRED_BALANCE_ANGLE = 1;
        public static double Balance_D = 0.1;

        public static final Transform2d CENTER_TRANSLATION = new Transform2d(
                new Translation2d(0.6, 0.0),
                new Rotation2d(0.0)
        );

        public static final Transform2d LEFT_TRANSLATION = new Transform2d(
                new Translation2d(0.6, 0.6),
                new Rotation2d()
        );

        public static final Transform2d RIGHT_TRANSLATION = new Transform2d(
                new Translation2d(0.6, -0.6),
                new Rotation2d()
        );
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

        public static final GosDoubleProperty ARM_EXT_KP = new GosDoubleProperty(false, "Arm extension kP", 0.1);
        public static final GosDoubleProperty ARM_EXT_KI = new GosDoubleProperty(false, "Arm extension kI", 0);
        public static final GosDoubleProperty ARM_EXT_KD = new GosDoubleProperty(false, "Arm extension kD", 0);

        public static final double ARM_OFFSET = 165;

        public static final int ENCODER_PORT = 1;

        public static final double SPROCKET_DIAMETER = 1.99;
        public static final double EXTENSION_RATIO = (1.0 / 25.0) * (SPROCKET_DIAMETER * Math.PI);


        public static final double EXT_PID_TOLERANCE = 1;

        // Used for dynamic limit calculations, unit is inches
        public static final double PIVOT_HIGHT = 5;
    }

    public static class LimitConstants {
        private LimitConstants() {}

        // Arm Extension limits for Piecewise Function
        public static final GosDoubleProperty ARM_EXT_STOW =
                new GosDoubleProperty(false, "Arm Extension Stow Limit", 0.5);
        public static final GosDoubleProperty ARM_EXT_SCORE_LOWER =
                new GosDoubleProperty(false, "Arm Extension Score Lower Limit", 0);
        public static final GosDoubleProperty ARM_EXT_SCORE_UPPER =
                new GosDoubleProperty(false, "Arm Extension Score Upper Limit", 25);

        // Arm Angle limits for Piecewise Function
        public static final GosDoubleProperty ARM_ANGLE_LOWER =
                new GosDoubleProperty(false, "Arm Angle Lower Limit", 13);
        public static final GosDoubleProperty ARM_ANGLE_UPPER =
                new GosDoubleProperty(false, "Arm Angle Upper Limit", 325);

        // Wrist limits for Piecewise Function
        public static final GosDoubleProperty WRIST_STOW =
                new GosDoubleProperty(false, "Wrist Stow Limit", 1);
        public static final GosDoubleProperty WRIST_SCORE_LOWER =
                new GosDoubleProperty(false, "Wrist Score Lower Limit", 0);
        public static final GosDoubleProperty WRIST_SCORE_UPPER =
                new GosDoubleProperty(false, "Wrist Score Upper Limit", 200);

        //Arm angle zones for piecewise intervals
        public static final GosDoubleProperty STOW_ZONE =
                new GosDoubleProperty(false, "Stow Zone Lower Bound", 45);
        public static final GosDoubleProperty SCORE_ZONE =
                new GosDoubleProperty(false, "Score Zone Lower Bound", 200);

        // Worry about ground at angle 302
        public static final GosDoubleProperty GROUND_ZONE =
                new GosDoubleProperty(false,"Ground Zone Lower Bound", 280);

        // Actual max limit is 324
        public static final GosDoubleProperty MAX_MOVEMENT =
                new GosDoubleProperty(false, "Max Movement Bound", 325);
    }

    public static class ArmSetpoints{
        public static final ArmPose STOW_POSITION = new ArmPose(0.0, 18, 0.0);
        public static final ArmPose VERT_STOW_POSE = new ArmPose(0.0, 180, 0.0);

        public static final ArmPose INTAKE_CUBE = new ArmPose(6.6, 325.1, 170.6);
        public static final ArmPose INTAKE_CONE = new ArmPose(1.0, 328.0, 177.0);
        public static final ArmPose HUMAN_PLAYER_STATION = new ArmPose(0.0, 237.6, 86.0);

        public static final ArmPose MIDDLE_GOAL_NON_STOW = new ArmPose(0.0, 252.1, 99.7);
        public static final ArmPose MIDDLE_GOAL_STOW = new ArmPose(0.0, 0.0, 0.0);
        public static final ArmPose HIGH_GOAL = new ArmPose(24.3, 247.0, 95.3);
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
