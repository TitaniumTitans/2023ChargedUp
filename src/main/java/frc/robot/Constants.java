// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

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
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kWheelCircumference = 2 * Math.PI * (kWheelDiameterMeters / 2);
        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(120);

        // Gear ratio
        public static final double kTurningRatio = (50.0 / 14.0) * (60.0 / 10.0);
        public static final double kDriveRatio = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);

        // PID constants
        public static final double kModuleKp = 0.16;
        public static final double kModuleKd = 3;
    }

    public static final class DriveConstants{
        // Can ID ports
        public static final int[] kMod0Cans = {3, 4, 5};
        public static final int[] kMod1Cans = {6, 7, 8};
        public static final int[] kMod2Cans = {9, 10, 11};
        public static final int[] kMod3Cans = {12, 13, 14};
        public static final int kGyroCan = 15;

        //Thanos Offsets
        public static final double kMod0Offset = currentMode == Mode.THANOS ? 160.400 : 357.803;//360 - 160.400;
        public static final double kMod1Offset = currentMode == Mode.THANOS ? 215.508 : 349.629;//360 - 215.508;
        public static final double kMod3Offset = currentMode == Mode.THANOS ? 96.943 : 180 + 46.143;//360 - 105.820; 70.488
        public static final double kMod2Offset = currentMode == Mode.THANOS ? 105.381 : 180 + 70.488;//360 - 97.119; 96.943
        // Competition Offsets
        // TODO competition offsets

        // Kinematics
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(20.733);
        
        // Distance between front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(20.733);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }

    public static final class AutoConstants{
        public static final double kMaxVelocityMPS = 5.0;
        public static final double kMaxAccelerationMPS = 5.0;

        public static final double kThetaP  = 0.2;
        public static final double kXP = 0.3;
        public static final double kYP = 0.3;

        public static final PIDController kControllerX = new PIDController(kXP, 0.0, 0.0);
        public static final PIDController kControllerY = new PIDController(kYP, 0.0, 0.0);

        public static final double kMaxThetaVelocity = 5.0;
        public static final double kMaxThetaAcceleration = 5.0;
        public static final TrapezoidProfile.Constraints kThetaConstraints = 
            new TrapezoidProfile.Constraints(kMaxThetaVelocity, kMaxThetaAcceleration);

        public static final ProfiledPIDController kThetaController =
            new ProfiledPIDController(kThetaP, 0.0, 0.0, kThetaConstraints);
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
