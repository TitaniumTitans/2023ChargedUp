package frc.robot.supersystems;

import frc.robot.subsystems.arm.ArmAngleSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import lib.utils.piecewise.PiecewiseInterval;
import lib.utils.piecewise.Range;
import lib.utils.piecewise.RangedPiecewise;

import java.util.List;

public class ArmSupersystem {

    private final Range fullArmRange = new Range(30, true, 300, false);

    private final Range stowRange = new Range(30, true, 200, false);
    private final Range fullRangeZone = new Range(200, true, 280, false);
    private final Range floorCollisionZone = new Range(280, true, 300, false);
    private final List<PiecewiseInterval<Double>> armRegion = List.of(
            new PiecewiseInterval<>(stowRange, ignored -> 0.0),
            new PiecewiseInterval<>(fullRangeZone, ignored -> 0.0),
            new PiecewiseInterval<>(floorCollisionZone, ignored -> 0.0)
    );
    //private static final RangedPiecewise PIECEWISE = new RangedPiecewise(fullArmRange, );
    private final ArmAngleSubsystem angleArmSubsystem;
    private final WristSubsystem wristSubsystem;


    //todo add ability for extension on arm with different subsystem
    //private final [INSERT EX SUBSYSTEM] extensionArmSubsystem;

    public ArmSupersystem(ArmAngleSubsystem armSubsystem, WristSubsystem wristSubsystem) {
        this.angleArmSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;


    }


}
