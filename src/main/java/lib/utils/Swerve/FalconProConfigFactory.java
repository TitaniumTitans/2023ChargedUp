package lib.utils.Swerve;

import com.ctre.phoenixpro.hardware.TalonFX;

public class FalconProConfigFactory {
    public static final int FASTEST_CYCLE = 4;
    public static final int SLOWEST_CYCLE = 2;
    public static void setStatusFrames(TalonFX talon) {
        // Set faults to 500 hertz
        talon.getFault_ReverseHardLimit().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_DeviceTemp().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_BootDuringEnable().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_Hardware().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_ProcTemp().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_ForwardHardLimit().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_ForwardSoftLimit().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_FusedSensorOutOfSync().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_MissingRemoteSensor().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_OverSupplyV().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_ReverseSoftLimit().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_StatorCurrLimit().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_Undervoltage().setUpdateFrequency(SLOWEST_CYCLE * 2);
        talon.getFault_UnstableSupplyV().setUpdateFrequency(SLOWEST_CYCLE * 2);

        // Set sticky faults to 1000 hertz
        talon.getStickyFault_ReverseHardLimit().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_DeviceTemp().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_BootDuringEnable().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_Hardware().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_ProcTemp().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_ForwardHardLimit().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_ForwardSoftLimit().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_FusedSensorOutOfSync().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_MissingRemoteSensor().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_OverSupplyV().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_ReverseSoftLimit().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_StatorCurrLimit().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_Undervoltage().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getStickyFault_UnstableSupplyV().setUpdateFrequency(SLOWEST_CYCLE);

        // Misc. Status Signals
        talon.getAppliedRotorPolarity().setUpdateFrequency(10);
        talon.getDeviceTemp().setUpdateFrequency(10);
        talon.getControlMode().setUpdateFrequency(10);
        talon.getMotionMagicIsRunning().setUpdateFrequency(5);
        talon.getDeviceEnable().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getProcessorTemp().setUpdateFrequency(SLOWEST_CYCLE);
        talon.getForwardLimit().setUpdateFrequency(50);
        talon.getReverseLimit().setUpdateFrequency(50);
        talon.getDutyCycle().setUpdateFrequency(90);

        // Closed loop configs
        talon.getClosedLoopError().setUpdateFrequency(90);
        talon.getClosedLoopFeedForward().setUpdateFrequency(90);
        talon.getClosedLoopOutput().setUpdateFrequency(90);
        talon.getClosedLoopReference().setUpdateFrequency(90);
        talon.getClosedLoopSlot().setUpdateFrequency(90);
        talon.getClosedLoopDerivativeOutput().setUpdateFrequency(90);
        talon.getClosedLoopIntegratedOutput().setUpdateFrequency(90);
        talon.getClosedLoopProportionalOutput().setUpdateFrequency(90);
        talon.getClosedLoopReferenceSlope().setUpdateFrequency(90);
    }
}
