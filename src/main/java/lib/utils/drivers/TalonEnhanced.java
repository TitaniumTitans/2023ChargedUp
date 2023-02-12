package lib.utils.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * This is my second attempt to create a wrapper class for CTRE motor
 * controllers and add error checking, "lazy" setters (that only run when a
 * value changes), and some other convenience methods.
 * 
 * At the moment, it's not <em>technically</em> a drop-in replacement for
 * TalonFX or TalonSRX (some of those class's methods that aren't in
 * IMotorControllerEnhanced are missing), but it's pretty close.
 */
public class TalonEnhanced {
    private final IMotorControllerEnhanced talon;

    private static final int TIMEOUT_MS = 100;
    private static final int FAST_FRAME_MS = 45;
    private static final int SLOW_FRAME_MS = 255;

    public TalonEnhanced(IMotorControllerEnhanced talon) {
        this.talon = talon;
    }

    public TalonEnhanced autoRetry(CTREUtil.ConfigCall talonConfigCall) {
        CTREUtil.autoRetry(talonConfigCall);
        return this;
    }

    private double lastDemand0 = Double.NaN;
    private ControlMode lastControlMode = null;

    /**
     * Same idea as 254's "LazyTalonFX" class. The
     * {@link IMotorControllerEnhanced}'s set method is only called when demand0 is
     * changed.
     */
    public TalonEnhanced set(ControlMode controlMode, double demand0) {
        if (controlMode != lastControlMode || demand0 != lastDemand0) {
            talon.set(controlMode, demand0);
            lastControlMode = controlMode;
            lastDemand0 = demand0;
        }
        return this;
    }

    private DemandType lastDemandType;
    private double lastDemand1;

    /**
     * Same idea as 254's "LazyTalonFX" class. The
     * {@link IMotorControllerEnhanced}'s set method is only called when demand0 is
     * changed.
     */
    public TalonEnhanced set(ControlMode controlMode, double demand0, DemandType demandType, double demand1) {
        if (controlMode != lastControlMode
                || demand0 != lastDemand0
                || demandType != lastDemandType
                || demand1 != lastDemand1) {
            talon.set(controlMode, demand0, demandType, demand1);
            lastControlMode = controlMode;
            lastDemand0 = demand0;
            lastDemandType = demandType;
            lastDemand1 = demand1;
        }
        return this;
    }

    public TalonEnhanced neutralOutput() {
        talon.neutralOutput();
        return this;
    }

    public TalonEnhanced setNeutralMode(NeutralMode neutralMode) {
        talon.setNeutralMode(neutralMode);
        return this;
    }

    public TalonEnhanced setSensorPhase(boolean sensorPhase) {
        talon.setSensorPhase(sensorPhase);
        return this;
    }

    public TalonEnhanced setInverted(boolean invert) {
        talon.setInverted(invert);

        return this;
    }

    public TalonEnhanced setInverted(InvertType invertType) {
        talon.setInverted(invertType);

        return this;
    }

    public boolean getInverted() {
        return talon.getInverted();
    }

    public TalonEnhanced configOpenLoopRamp(double secondsFromNeutralToFull) {
        return this.autoRetry(() -> talon.configOpenloopRamp(secondsFromNeutralToFull, TIMEOUT_MS));
    }

    public TalonEnhanced configClosedLoopRamp(double secondsFromNeutralToFull) {
        return this.autoRetry(() -> talon.configClosedloopRamp(secondsFromNeutralToFull, TIMEOUT_MS));
    }

    public TalonEnhanced configPeakOutputForward(double percentOut) {
        return this.autoRetry(() -> talon.configPeakOutputForward(percentOut, TIMEOUT_MS));
    }

    public TalonEnhanced configPeakOutputReverse(double percentOut) {
        return this.autoRetry(() -> talon.configPeakOutputReverse(percentOut, TIMEOUT_MS));
    }

    public TalonEnhanced configNominalOutputForward(double percentOut) {
        return this.autoRetry(() -> talon.configNominalOutputForward(percentOut, TIMEOUT_MS));
    }

    public TalonEnhanced configNominalOutputReverse(double percentOut) {
        return this.autoRetry(() -> talon.configNominalOutputReverse(percentOut, TIMEOUT_MS));
    }

    public TalonEnhanced configNeutralDeadband(double percentDeadband) {
        return this.autoRetry(() -> talon.configNeutralDeadband(percentDeadband, TIMEOUT_MS));
    }

    public TalonEnhanced configVoltageCompSaturation(double voltage) {
        return this.autoRetry(() -> talon.configVoltageCompSaturation(voltage, TIMEOUT_MS));
    }

    public TalonEnhanced configVoltageMeasurementFilter(int filterWindowSamples) {
        return this.autoRetry(() -> talon.configVoltageMeasurementFilter(filterWindowSamples, TIMEOUT_MS));
    }

    public TalonEnhanced enableVoltageCompensation(boolean enable) {
        talon.enableVoltageCompensation(enable);
        return this;
    }

    public double getBusVoltage() {
        return talon.getBusVoltage();
    }

    public double getMotorOutputPercent() {
        return talon.getMotorOutputPercent();
    }

    public double getMotorOutputVoltage() {
        return talon.getMotorOutputVoltage();
    }

    public double getTemperature() {
        return talon.getTemperature();
    }

    public TalonEnhanced configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx) {
        return this.autoRetry(() -> talon.configSelectedFeedbackSensor(feedbackDevice, pidIdx, TIMEOUT_MS));
    }

    public TalonEnhanced configSelectedFeedbackCoefficient(double coefficient, int pidIdx) {
        return this.autoRetry(() -> talon.configSelectedFeedbackCoefficient(coefficient, pidIdx, TIMEOUT_MS));
    }

    public TalonEnhanced configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource,
            int remoteOrdinal) {
        return this.autoRetry(
                () -> talon.configRemoteFeedbackFilter(deviceID, remoteSensorSource, remoteOrdinal, TIMEOUT_MS));
    }

    public TalonEnhanced configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal) {
        return this.autoRetry(() -> talon.configRemoteFeedbackFilter(canCoderRef, remoteOrdinal, TIMEOUT_MS));
    }

    public TalonEnhanced configRemoteFeedbackFilter(BaseTalon talonRef, int remoteOrdinal) {
        return this.autoRetry(() -> talon.configRemoteFeedbackFilter(talonRef, remoteOrdinal, TIMEOUT_MS));
    }

    public TalonEnhanced configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice) {
        return this.autoRetry(() -> talon.configSensorTerm(sensorTerm, feedbackDevice, TIMEOUT_MS));
    }

    public double getSelectedSensorPosition(int pidIdx) {
        return talon.getSelectedSensorPosition(pidIdx);
    }

    public double getSelectedSensorVelocity(int pidIdx) {
        return talon.getSelectedSensorVelocity(pidIdx);
    }

    public TalonEnhanced setSelectedSensorPosition(double sensorPos, int pidIdx) {
        return this.autoRetry(() -> talon.setSelectedSensorPosition(sensorPos, pidIdx, TIMEOUT_MS));
    }

    public TalonEnhanced setControlFramePeriod(ControlFrame frame, int periodMs) {
        return this.autoRetry(() -> talon.setControlFramePeriod(frame, periodMs));
    }

    public TalonEnhanced setStatusFramePeriod(StatusFrame frame, int periodMs) {
        return this.autoRetry(() -> talon.setStatusFramePeriod(frame, periodMs, TIMEOUT_MS));
    }

    public int getStatusFramePeriod(StatusFrame frame) {
        return talon.getStatusFramePeriod(frame, TIMEOUT_MS);
    }

    public TalonEnhanced configForwardLimitSwitchSource(RemoteLimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose, int deviceID) {
        return this
                .autoRetry(() -> talon.configForwardLimitSwitchSource(type, normalOpenOrClose, deviceID, TIMEOUT_MS));
    }

    public TalonEnhanced configReverseLimitSwitchSource(RemoteLimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose, int deviceID) {
        return this
                .autoRetry(() -> talon.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID, TIMEOUT_MS));
    }

    public TalonEnhanced overrideLimitSwitchesEnable(boolean enable) {
        talon.overrideLimitSwitchesEnable(enable);
        return this;
    }

    public TalonEnhanced configForwardSoftLimitThreshold(double forwardSensorLimit) {
        return this.autoRetry(() -> talon.configForwardSoftLimitThreshold(forwardSensorLimit, TIMEOUT_MS));
    }

    public TalonEnhanced configReverseSoftLimitThreshold(double reverseSensorLimit) {
        return this.autoRetry(() -> talon.configReverseSoftLimitThreshold(reverseSensorLimit, TIMEOUT_MS));
    }

    public TalonEnhanced configForwardSoftLimitEnable(boolean enable) {
        return this.autoRetry(() -> talon.configForwardSoftLimitEnable(enable, TIMEOUT_MS));
    }

    public TalonEnhanced configReverseSoftLimitEnable(boolean enable) {
        return this.autoRetry(() -> talon.configReverseSoftLimitEnable(enable, TIMEOUT_MS));
    }

    public TalonEnhanced overrideSoftLimitsEnable(boolean enable) {
        talon.overrideSoftLimitsEnable(enable);
        return this;
    }


    private double last_kP = Double.NaN;

    public TalonEnhanced config_kP(int slotIdx, double value) {
        if (last_kP != value) {
            last_kP = value;
            return this.autoRetry(() -> talon.config_kP(slotIdx, value, TIMEOUT_MS));
        } else {
            return this;
        }
    }

    private double last_kI = Double.NaN;

    public TalonEnhanced config_kI(int slotIdx, double value) {
        if (last_kI != value) {
            last_kI = value;
            return this.autoRetry(() -> talon.config_kI(slotIdx, value, TIMEOUT_MS));
        } else {
            return this;
        }
    }

    private double last_kD = Double.NaN;

    public TalonEnhanced config_kD(int slotIdx, double value) {
        if (last_kD != value) {
            last_kD = value;
            return this.autoRetry(() -> talon.config_kD(slotIdx, value, TIMEOUT_MS));
        } else {
            return this;
        }
    }

    private double last_kF = Double.NaN;

    public TalonEnhanced config_kF(int slotIdx, double value) {
        if (last_kF != value) {
            last_kF = value;
            return this.autoRetry(() -> talon.config_kF(slotIdx, value, TIMEOUT_MS));
        } else {
            return this;
        }
    }

    private double lastIZone = Double.NaN;

    public TalonEnhanced configIntegralZone(int slotIdx, double iZone) {
        if (lastIZone != iZone) {
            lastIZone = iZone;
            return this.autoRetry(() -> talon.config_IntegralZone(slotIdx, iZone, TIMEOUT_MS));
        } else {
            return this;
        }
    }

    private double lastAllowableCloseLoopError = Double.NaN;

    public TalonEnhanced configAllowableClosedLoopError(int slotIdx, double allowableCloseLoopError) {
        if (lastAllowableCloseLoopError != allowableCloseLoopError) {
            lastAllowableCloseLoopError = allowableCloseLoopError;
            return this.autoRetry(
                    () -> talon.configAllowableClosedloopError(slotIdx, allowableCloseLoopError, TIMEOUT_MS));
        } else {
            return this;
        }
    }

    private double lastMaxIAccumulator = Double.NaN;

    public TalonEnhanced configMaxIntegralAccumulator(int slotIdx, double maxIAccumulator) {
        if (lastMaxIAccumulator != maxIAccumulator) {
            lastMaxIAccumulator = maxIAccumulator;
            return this.autoRetry(() -> talon.configMaxIntegralAccumulator(slotIdx, maxIAccumulator, TIMEOUT_MS));
        } else {
            return this;
        }
    }

    public TalonEnhanced configClosedLoopPeakOutput(int slotIdx, double percentOut) {
        return this.autoRetry(() -> talon.configClosedLoopPeakOutput(slotIdx, percentOut, TIMEOUT_MS));
    }

    public TalonEnhanced configClosedLoopPeriod(int slotIdx, int loopTimeMs) {
        return this.autoRetry(() -> talon.configClosedLoopPeriod(slotIdx, loopTimeMs, TIMEOUT_MS));
    }

    public TalonEnhanced configAuxPIDPolarity(boolean invert) {
        return this.autoRetry(() -> talon.configAuxPIDPolarity(invert, TIMEOUT_MS));
    }

    public TalonEnhanced setIntegralAccumulator(double iAccumulator, int pidIdx) {
        return this.autoRetry(() -> talon.setIntegralAccumulator(iAccumulator, pidIdx, TIMEOUT_MS));
    }

    public double getClosedLoopError(int pidIdx) {
        return talon.getClosedLoopError(pidIdx);
    }

    public double getIntegralAccumulator(int pidIdx) {
        return talon.getIntegralAccumulator(pidIdx);
    }

    public double getErrorDerivative(int pidIdx) {
        return talon.getErrorDerivative(pidIdx);
    }

    public TalonEnhanced selectProfileSlot(int slotIdx, int pidIdx) {
        talon.selectProfileSlot(slotIdx, pidIdx);
        return this;
    }

    public double getClosedLoopTarget(int pidIdx) {
        return talon.getClosedLoopTarget(pidIdx);
    }

    public double getActiveTrajectoryPosition() {
        return talon.getActiveTrajectoryPosition();
    }

    public double getActiveTrajectoryVelocity() {
        return talon.getActiveTrajectoryVelocity();
    }

    private double lastCruiseVelocity = Double.NaN;

    public TalonEnhanced configMotionCruiseVelocity(double cruiseVelocity) {
        if (lastCruiseVelocity != cruiseVelocity) {
            lastCruiseVelocity = cruiseVelocity;
            return this.autoRetry(() -> talon.configMotionCruiseVelocity(cruiseVelocity, TIMEOUT_MS));
        } else {
            return this;
        }
    }

    private double lastAccel = Double.NaN;

    public TalonEnhanced configMotionAcceleration(double accel) {
        if (lastAccel != accel) {
            lastAccel = accel;
            return this.autoRetry(() -> talon.configMotionAcceleration(accel, TIMEOUT_MS));
        } else {
            return this;
        }
    }

    public TalonEnhanced configMotionSCurveStrength(int curveStrength) {
        return this.autoRetry(() -> talon.configMotionSCurveStrength(curveStrength, TIMEOUT_MS));
    }

    public TalonEnhanced configMotionProfileTrajectoryPeriod(int baseTrajectoryDurationMS) {
        return this.autoRetry(() -> talon.configMotionProfileTrajectoryPeriod(baseTrajectoryDurationMS, TIMEOUT_MS));
    }

    public TalonEnhanced clearMotionProfileTrajectories() {
        return this.autoRetry(talon::clearMotionProfileTrajectories);
    }

    public int getMotionProfileTopLevelBufferCount() {
        return talon.getMotionProfileTopLevelBufferCount();
    }

    public TalonEnhanced pushMotionProfileTrajectory(TrajectoryPoint trajectoryPoint) {
        return this.autoRetry(() -> talon.pushMotionProfileTrajectory(trajectoryPoint));
    }

    public boolean isMotionProfileTopLevelBufferFull() {
        return talon.isMotionProfileTopLevelBufferFull();
    }

    public TalonEnhanced processMotionProfileBuffer() {
        talon.processMotionProfileBuffer();

        return this;
    }

    public TalonEnhanced getMotionProfileStatus(MotionProfileStatus statusToFill) {
        return this.autoRetry(() -> talon.getMotionProfileStatus(statusToFill));
    }

    public TalonEnhanced clearMotionProfileHasUnderRun() {
        return this.autoRetry(() -> talon.clearMotionProfileHasUnderrun(TIMEOUT_MS));
    }

    public TalonEnhanced changeMotionControlFramePeriod(int periodMs) {
        return this.autoRetry(() -> talon.changeMotionControlFramePeriod(periodMs));
    }

    public ErrorCode getLastError() {
        return talon.getLastError();
    }

    public TalonEnhanced getFaults(Faults toFill) {
        return this.autoRetry(() -> talon.getFaults(toFill));
    }

    public TalonEnhanced getStickyFaults(StickyFaults toFill) {
        return this.autoRetry(() -> talon.getStickyFaults(toFill));
    }

    public TalonEnhanced clearStickyFaults() {
        return this.autoRetry(() -> talon.clearStickyFaults(TIMEOUT_MS));
    }

    public int getFirmwareVersion() {
        return talon.getFirmwareVersion();
    }


    public boolean hasResetOccurred() {
        return talon.hasResetOccurred();
    }

    public TalonEnhanced configSetCustomParam(int newValue, int paramIndex) {
        return this.autoRetry(() -> talon.configSetCustomParam(newValue, paramIndex, TIMEOUT_MS));
    }

    public int configGetCustomParam(int paramIndex) {
        return talon.configGetCustomParam(paramIndex, TIMEOUT_MS);
    }

    public TalonEnhanced configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
        return this.autoRetry(() -> talon.configSetParameter(param, value, subValue, ordinal, TIMEOUT_MS));
    }

    public TalonEnhanced configSetParameter(int param, double value, int subValue, int ordinal) {
        return this.autoRetry(() -> talon.configSetParameter(param, value, subValue, ordinal, TIMEOUT_MS));
    }

    public double configGetParameter(ParamEnum paramEnum, int ordinal) {
        return talon.configGetParameter(paramEnum, ordinal, TIMEOUT_MS);
    }

    public double configGetParameter(int paramEnum, int ordinal) {
        return talon.configGetParameter(paramEnum, ordinal, TIMEOUT_MS);
    }

    public int getBaseID() {
        return talon.getBaseID();
    }

    public int getDeviceID() {
        return talon.getDeviceID();
    }

    public ControlMode getControlMode() {
        return talon.getControlMode();
    }

    public TalonEnhanced follow(IMotorController masterToFollow) {
        talon.follow(masterToFollow);

        return this;
    }

    public TalonEnhanced follow(TalonEnhanced masterToFollow) {
        return this.follow(masterToFollow.talon);
    }

    public TalonEnhanced configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx) {
        return this.autoRetry(() -> talon.configSelectedFeedbackSensor(feedbackDevice, pidIdx, TIMEOUT_MS));
    }

    public TalonEnhanced configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg) {
        return this.autoRetry(() -> talon.configSupplyCurrentLimit(currLimitCfg, TIMEOUT_MS));
    }

    public TalonEnhanced setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs) {
        return this.autoRetry(() -> talon.setStatusFramePeriod(frame, periodMs, TIMEOUT_MS));
    }

    public int getStatusFramePeriod(StatusFrameEnhanced frame) {
        return talon.getStatusFramePeriod(frame, TIMEOUT_MS);
    }

    public TalonEnhanced setFeedbackIntervals(int intervalMs) {
        return this
                .autoRetry(() -> talon.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_2_Feedback0, intervalMs, TIMEOUT_MS))
                .autoRetry(() -> talon.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_Brushless_Current, intervalMs, TIMEOUT_MS));
    }

    public TalonEnhanced setControlIntervals(int intervalMs) {
        return this.autoRetry(() -> talon.setControlFramePeriod(ControlFrame.Control_3_General, intervalMs));
    }

    public TalonEnhanced setAllStatusIntervals(int intervalMs) {
        for (StatusFrameEnhanced frame : StatusFrameEnhanced.values()) {
            this.autoRetry(() -> talon.setStatusFramePeriod(frame, intervalMs, TIMEOUT_MS));
        }
        return this;
    }

    public TalonEnhanced defaultFrameIntervals() {
        return this.setAllStatusIntervals(SLOW_FRAME_MS).setControlIntervals(FAST_FRAME_MS);
    }

    public double getOutputCurrent() {
        return talon.getMotorOutputVoltage();
    }

    public TalonEnhanced configVelocityMeasurementPeriod(SensorVelocityMeasPeriod period) {
        return this.autoRetry(() -> talon.configVelocityMeasurementPeriod(period, TIMEOUT_MS));
    }

    public TalonEnhanced configVelocityMeasurementWindow(int windowSize) {
        return this.autoRetry(() -> talon.configVelocityMeasurementWindow(windowSize, TIMEOUT_MS));
    }

    public TalonEnhanced configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
        return this.autoRetry(() -> talon.configForwardLimitSwitchSource(type, normalOpenOrClose, TIMEOUT_MS));
    }

    public TalonEnhanced configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
        return this.autoRetry(() -> talon.configReverseLimitSwitchSource(type, normalOpenOrClose, TIMEOUT_MS));
    }
}
