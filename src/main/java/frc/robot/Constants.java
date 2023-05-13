// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.signals.InvertedValue;
import com.gos.lib.properties.GosDoubleProperty;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.supersystems.ArmLimits;
import frc.robot.supersystems.ArmPose;
import lib.utils.piecewise.PiecewiseInterval;
import lib.utils.piecewise.Range;
import lib.utils.piecewise.RangedPiecewise;

import java.util.List;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final Mode CURRENT_MODE = Mode.HELIOS_V2;

    /*Constants for physical aspects of the modules, plus PID loops constants*/
    public static final class ModuleConstants {
        private ModuleConstants() {
            throw new IllegalStateException("Utility Class");
        }
        // Physical wheel constants
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * (WHEEL_DIAMETER_METERS / 2);

        // Gear ratio
        public static final double TURNING_RATIO = (50.0 / 14.0) * (60.0 / 10.0);
        public static final double DRIVE_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);

        // PID constants
        public static final double MODULE_KP = 0.8;
        public static final double MODULE_KD = 0;
        public static final double POSITION_CONVERSION_FACTOR = ((Math.PI * 2) / TURNING_RATIO);

        public static final double MODULE_KS = 0.0;
        public static final double MODULE_KV = 1.75;
        public static final double MODULE_KA = 0.0;
        public static final double MODULE_DRIVE_KP = 0.5;

        public static final double MAX_SPEED_L2_MPS = 3.657;
        public static final GosDoubleProperty MAX_SPEED_FPS = new GosDoubleProperty(false, "Max Drive Speed", 17);
        public static final double MAX_SPEED_L3_MPS = Units.feetToMeters(MAX_SPEED_FPS.getValue());

        /** Constants for the Phoenix Pro Modules using Falcon 500s **/
        public static final double L3_GEAR_RATIO = 6.12;

        public static final double FALCON_AZIMUTH_KP = 3.50;
        public static final double FALCON_AZIMUTH_KI = 0.05;
        public static final double FALCON_AZIMUTH_KD = 0.00;

        public static final double FALCON_DRIVE_KV = 0.75;
        public static final double FALCON_DRIVE_KS = 0.1;
        public static final double FALCON_DRIVE_KA = 0.0;
        public static final double FALCON_DRIVE_KP = 0.01;

        public static final InvertedValue FALCON_AZIMUTH_INVERT = InvertedValue.Clockwise_Positive;
    }

    public static final class DriveConstants {
        private DriveConstants() {
            throw new IllegalStateException("Utility Class");
        }
        // Can ID ports
        public static final int[] MOD_FL_CANS = {3, 4, 5};
        public static final int[] MOD_FR_CANS = {6, 7, 8};
        public static final int[] MOD_BR_CANS = {9, 10, 11};
        public static final int[] MOD_BL_CANS = {12, 13, 14};
        public static final int GYRO_CAN = 15;

        //Thanos Offsets
        public static final double MOD_FR_OFFSET = 141.35 + 180;
        public static final double MOD_FL_OFFSET = -23.555 + 180;
        public static final double MOD_BR_OFFSET = -142.207 + 180;
        public static final double MOD_BL_OFFSET = 140.625;
        // Competition Offset
        public static final double MOD_FL_OFFSET_V2 = 262.8 + 180;
        public static final double MOD_FR_OFFSET_V2 = 255.893 + 180;
        public static final double MOD_BL_OFFSET_V2 = 173.4 + 180;
        public static final double MOD_BR_OFFSET_V2 = 169.4 + 180;

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
                new Rotation3d(Units.degreesToRadians(-2.2), 0.0, 0.0));
        public static final Transform3d LEFT_CAM_POSE = new Transform3d(
                new Translation3d(Units.inchesToMeters(2.0), Units.inchesToMeters(6.0), Units.inchesToMeters(24.0)),
                new Rotation3d(Units.degreesToRadians(180.0), 0.0, Units.degreesToRadians(90.0)));
        public static final String FRONT_CAM_NAME = "FrontPiCam";
        public static final String LEFT_CAM_NAME = "LeftWebCam";

        public static final GosDoubleProperty CAM_AMBIGUITY_THRESHOLD
                = new GosDoubleProperty(false, "Camera ambiguity threshold", 0.2);
        public static final GosDoubleProperty CAM_DISTANCE_THRESHOLD
                = new GosDoubleProperty(false, "Camera distance threshold", 4);
    }

    public static class WristConstants {
        private WristConstants() {
            throw new IllegalStateException("Utility Class");
        }

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
        private AutoConstants() {
            throw new IllegalStateException("Utility Class");
        }

        //Trajectory following values
        public static final double MAX_VELOCITY_MPS_AUTO = Units.feetToMeters(16);
        public static final double MAX_ACCELERATION_MPS_AUTO = MAX_VELOCITY_MPS_AUTO / 2.0;

        public static final PIDController THETA_CONTROLLER =
                new PIDController(3.25, 0.1, 0.3);

        public static final PIDController CONTROLLER_X =
            new PIDController(3.2, 0.03, 0.3);
        public static final PIDController CONTROLLER_Y =
            new PIDController(3.2, 0.03, 0.3);

        public static final PIDConstants CONSTANTS_X =
                new PIDConstants(4.0, 0.005, 0.0);

        public static final PIDConstants THETA_CONSTANTS =
                new PIDConstants(3.2, 0.0, 0.0);
        
        //Auto balance constants
        public static final double BALANCE_P = -0.04;
        public static final double DESIRED_BALANCE_ANGLE = 0;
        public static final double ACCEPTABLE_BALANCE_ANGLE = 1.5;
        public static final double BALANCE_D = 0.1;

        public static final Transform2d CENTER_TRANSLATION = new Transform2d(
                new Translation2d(0.65, 0.0),
                new Rotation2d(0.0)
        );

        public static final Transform2d LEFT_TRANSLATION = new Transform2d(
                new Translation2d(0.65, 0.525),
                new Rotation2d()
        );

        public static final Transform2d RIGHT_TRANSLATION = new Transform2d(
                new Translation2d(0.65, -0.525),
                new Rotation2d()
        );

        public static final Transform2d HUMAN_PLAYER_RIGHT_TRANSLATION = new Transform2d(
                new Translation2d(0.5, 0.75),
                new Rotation2d()
        );

        public static final Transform2d HUMAN_PLAYER_LEFT_TRANSLATION = new Transform2d(
                new Translation2d(0.5, -0.75),
                new Rotation2d()
        );


    }

    public static class ArmConstants {

        private ArmConstants() {
            throw new IllegalStateException("Utility Class");
        }

        public static final int ARM_EXTENSION_ID = 18;
        public static final int ARM_ANGLE_ID_MASTER = 16;
        public static final int ARM_ANGLE_ID_FOLLOWER = 17;
        public static final int LIMIT_SWITCH_PORT = 3;

        public static final double KP_ANGLE = CURRENT_MODE == Mode.HELIOS_V1 ? 0.53 : 0.227;
        public static final double KI_ANGLE = 0.007;
        public static final double KD_ANGLE = 0.08;

        public static final GosDoubleProperty ARM_EXT_KP = new GosDoubleProperty(true, "Arm extension kP", 0.5);
        public static final GosDoubleProperty ARM_EXT_KI = new GosDoubleProperty(false, "Arm extension kI", 0);
        public static final GosDoubleProperty ARM_EXT_KD = new GosDoubleProperty(false, "Arm extension kD", 0);

        public static final double ARM_OFFSET = CURRENT_MODE == Mode.HELIOS_V1 ? 165.0 : 294;

        public static final int ENCODER_PORT = 4;

        public static final double SPROCKET_DIAMETER = 1.99;
        public static final double EXTENSION_RATIO = 0.3532;

        public static final double PIVOT_HEIGHT = CURRENT_MODE == Mode.HELIOS_V1 ? 33.0 : 33.0;


        public static final double EXT_PID_TOLERANCE = 1.0;

        public static final double EXT_AGRESSION = 12.0;
        public static final double ANGLE_AGRESSION = 20.0;
        public static final double MAX_ANGLE_SPEED = 1.0;
        public static final double MAX_EXT = 1.0;
    }
    public static class LimitConstants {

        private LimitConstants() {
            throw new IllegalStateException("Utility Class");
        }



        // Arm Extension limits for Piecewise Function
        public static final GosDoubleProperty ARM_EXT_STOW =
                new GosDoubleProperty(false, "Arm Extension Stow Limit", 0.0);
        public static final GosDoubleProperty ARM_EXT_SCORE_LOWER =
                new GosDoubleProperty(false, "Arm Extension Score Lower Limit", 0);
        public static final GosDoubleProperty ARM_EXT_SCORE_UPPER =
                new GosDoubleProperty(true, "Arm Extension Score Upper Limit", 15 * 1.4);

        // Arm Angle limits for Piecewise Function
        public static final GosDoubleProperty ARM_ANGLE_LOWER =
                new GosDoubleProperty(true, "Arm Angle Lower Limit", 40);
        public static final GosDoubleProperty ARM_ANGLE_UPPER =
                new GosDoubleProperty(false, "Arm Angle Upper Limit", 325);

        // Wrist limits for Piecewise Function
        public static final GosDoubleProperty WRIST_STOW =
                new GosDoubleProperty(false, "Wrist Stow Limit", 1);
        public static final GosDoubleProperty WRIST_SCORE_LOWER =
                new GosDoubleProperty(false, "Wrist Score Lower Limit", 0);
        public static final GosDoubleProperty WRIST_SCORE_UPPER =
                new GosDoubleProperty(true, "Wrist Score Upper Limit", 130);

        //Arm angle zones for piecewise intervals
        public static final GosDoubleProperty STOW_ZONE =
                new GosDoubleProperty(false, "Stow Zone Lower Bound", 45);
        public static final GosDoubleProperty INTAKE_ZONE =
                new GosDoubleProperty(true, "Intake Zone Lower Bound", 50);
        public static final GosDoubleProperty INTAKE_ZONE_UPPER =
                new GosDoubleProperty(true, "Intake Upper Bound", 65);
        public static final GosDoubleProperty SCORE_ZONE =
                new GosDoubleProperty(true, "Score Zone Lower Bound", 180);

        // Worry about ground at angle 302
        public static final GosDoubleProperty GROUND_ZONE =
                new GosDoubleProperty(false,"Ground Zone Lower Bound", 280);

        // Actual max limit is 324
        public static final GosDoubleProperty MAX_MOVEMENT =
                new GosDoubleProperty(false, "Max Movement Bound", 325);

        // ArmLimit objects kept as constants
        public static final ArmLimits STOW_LIMIT = new ArmLimits(
                WRIST_STOW.getValue(), WRIST_STOW.getValue(),
                ARM_EXT_STOW.getValue(), ARM_EXT_STOW.getValue(),
                ARM_ANGLE_LOWER.getValue(), ARM_ANGLE_UPPER.getValue());

        public static final ArmLimits INTAKE_LIMIT = new ArmLimits(
                WRIST_SCORE_LOWER.getValue(), WRIST_SCORE_UPPER.getValue(),
                ARM_EXT_STOW.getValue(), ARM_EXT_SCORE_UPPER.getValue(),
                ARM_ANGLE_LOWER.getValue(), ARM_ANGLE_UPPER.getValue()
        );

        public static final ArmLimits NO_EXTENTION = new ArmLimits(
                WRIST_STOW.getValue(), WRIST_SCORE_UPPER.getValue(),
                ARM_EXT_STOW.getValue(), ARM_EXT_STOW.getValue(),
                ARM_ANGLE_LOWER.getValue(), ARM_ANGLE_UPPER.getValue());

        public static final ArmLimits FULL_RANGE_LIMIT = new ArmLimits(
                WRIST_SCORE_LOWER.getValue(), WRIST_SCORE_UPPER.getValue(),
                ARM_EXT_SCORE_LOWER.getValue(), ARM_EXT_SCORE_UPPER.getValue(),
                ARM_ANGLE_LOWER.getValue(), ARM_ANGLE_UPPER.getValue());

        public static final ArmLimits GROUND_LIMIT = new ArmLimits(
                WRIST_SCORE_LOWER.getValue(), WRIST_SCORE_UPPER.getValue(),
                ARM_EXT_SCORE_LOWER.getValue(), 6.1,
                ARM_ANGLE_LOWER.getValue(), ARM_ANGLE_UPPER.getValue());

        // Piecewise function for limiting drive speed based off arm angle
        private static final Range BACK_RANGE = new Range(
                STOW_ZONE.getValue(),
                true,
                360 - (180 + STOW_ZONE.getValue()),
                false
        );

        private static final Range FORWARD_RANGE = new Range(
                360 - (180 + STOW_ZONE.getValue()),
                true,
                MAX_MOVEMENT.getValue(),
                false
        );

        public static final GosDoubleProperty SPEED_LIMIT_RAMP = new GosDoubleProperty(false, "Speed Limit Ramp", 3.5);
        public static final GosDoubleProperty SPEED_LIMIT_CAP = new GosDoubleProperty(false, "Speed Limit Cap", 0.5);

        private static final PiecewiseInterval<Double> BACK_SPEED = new PiecewiseInterval<>(
                BACK_RANGE,
                angle ->
                        (MathUtil.clamp(
                                ((1 - ((angle - STOW_ZONE.getValue()) / 180)) * SPEED_LIMIT_RAMP.getValue()),
                                SPEED_LIMIT_CAP.getValue(),
                                1)
        ));

        private static final PiecewiseInterval<Double> FORWARD_SPEED = new PiecewiseInterval<>(
                FORWARD_RANGE,
                angle -> (MathUtil.clamp(
                        1 - ((angle - (360 - (180 + STOW_ZONE.getValue()))) * SPEED_LIMIT_RAMP.getValue()),
                        SPEED_LIMIT_CAP.getValue(),
                        1))
        );

        public static final RangedPiecewise<Double> DRIVE_SPEED_PIECEWISE = new RangedPiecewise<>(
                new Range(STOW_ZONE.getValue(), true, MAX_MOVEMENT.getValue(), false),
                List.of(
                        BACK_SPEED,
                        FORWARD_SPEED
                )
        );
    }
    public static class ArmSetpoints {

        private ArmSetpoints() {
            throw new IllegalStateException("Utility Class");
        }

        public static final GosDoubleProperty HUMAN_HEIGHT = new GosDoubleProperty(false, "HUMAN HIEGHT", 233.6);
        public static final GosDoubleProperty HUMAN_WRIST = new GosDoubleProperty(false, "HUMAN WRIST", 78.0);

        public static final ArmPose STOW_POSITION = new ArmPose(0.0, 40, 0.0);
        public static final ArmPose VERT_STOW_POSE = new ArmPose(0.0, 180, 0.0);

        public static final ArmPose INTAKE_CUBE = new ArmPose(4.7, 325.1, 165.6);
        public static final ArmPose INTAKE_CONE = new ArmPose(1.5, 328.0, 170.0);
        public static final ArmPose INTAKE_BATTERY = new ArmPose(11.9, 58.0, 145.0);
        public static final ArmPose HUMAN_PLAYER_STATION = new ArmPose(0.0, 230.6, 78.0);

        public static final ArmPose MIDDLE_GOAL = new ArmPose(0.0, 252.1, 99.7);
        public static final ArmPose HIGH_GOAL = new ArmPose(20, 240.0, 95.3);
    }

    public static final int DRIVER_PORT = 0;


    public enum Mode {
        /** Running on the test bot */
        HELIOS_V2,

        /**Running on the competition bot */
        HELIOS_V1,

        /** Running a physics simulator */
        SIM,

        /**Replaying from a log file */
        REPLAY;
    }
}
