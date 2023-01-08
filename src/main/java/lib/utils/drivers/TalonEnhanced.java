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
import lib.utils.drivers.CTREUtil.*;

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
    private IMotorControllerEnhanced m_talon;

    // TODO Consider moving thsese constants somewhere else
    final static private int kTimeoutMs = 100;
    final static private int kFastFrameMs = 45;
    final static private int kSlowFrameMs = 255;

    // TODO Set common parameters in constructor, add more
    // TODO overloaded constructor methods
    public TalonEnhanced(IMotorControllerEnhanced talon) {
        m_talon = talon;
    }

    // TODO Change to handleError and change behavior depending on error type
    public TalonEnhanced autoRetry(ConfigCall talonConfigCall) {
        CTREUtil.autoRetry(() -> talonConfigCall.run());
        return this;
    }

    private double m_lastDemand0 = Double.NaN;
    private ControlMode m_lastControlMode = null;

    /**
     * Same idea as 254's "LazyTalonFX" class. The
     * {@link IMotorControllerEnhanced}'s set method is only called when demand0 is
     * changed.
     */
    public TalonEnhanced set(ControlMode controlMode, double demand0) {
        if (controlMode != m_lastControlMode || demand0 != m_lastDemand0) {
            m_talon.set(controlMode, demand0);
            m_lastControlMode = controlMode;
            m_lastDemand0 = demand0;
        }
        return this;
    }

    private DemandType m_lastDemandType;
    private double m_lastDemand1;

    /**
     * Same idea as 254's "LazyTalonFX" class. The
     * {@link IMotorControllerEnhanced}'s set method is only called when demand0 is
     * changed.
     */
    public TalonEnhanced set(ControlMode controlMode, double demand0, DemandType demandType, double demand1) {
        if (controlMode != m_lastControlMode ||
                demand0 != m_lastDemand0 ||
                demandType != m_lastDemandType ||
                demand1 != m_lastDemand1) {
            m_talon.set(controlMode, demand0, demandType, demand1);
            m_lastControlMode = controlMode;
            m_lastDemand0 = demand0;
            m_lastDemandType = demandType;
            m_lastDemand1 = demand1;
        }
        return this;
    }

    public TalonEnhanced neutralOutput() {
        m_talon.neutralOutput();
        return this;
    }

    public TalonEnhanced setNeutralMode(NeutralMode neutralMode) {
        m_talon.setNeutralMode(neutralMode);
        return this;
    }

    public TalonEnhanced setSensorPhase(boolean PhaseSensor) {
        m_talon.setSensorPhase(PhaseSensor);
        return this;
    }

    public TalonEnhanced setInverted(boolean invert) {
        m_talon.setInverted(invert);

        return this;
    }

    public TalonEnhanced setInverted(InvertType invertType) {
        m_talon.setInverted(invertType);

        return this;
    }

    public boolean getInverted() {
        return m_talon.getInverted();
    }

    public TalonEnhanced configOpenloopRamp(double secondsFromNeutralToFull) {
        return this.autoRetry(() -> m_talon.configOpenloopRamp(secondsFromNeutralToFull, kTimeoutMs));
    }

    public TalonEnhanced configClosedloopRamp(double secondsFromNeutralToFull) {
        return this.autoRetry(() -> m_talon.configClosedloopRamp(secondsFromNeutralToFull, kTimeoutMs));
    }

    public TalonEnhanced configPeakOutputForward(double percentOut) {
        return this.autoRetry(() -> m_talon.configPeakOutputForward(percentOut, kTimeoutMs));
    }

    public TalonEnhanced configPeakOutputReverse(double percentOut) {
        return this.autoRetry(() -> m_talon.configPeakOutputReverse(percentOut, kTimeoutMs));
    }

    public TalonEnhanced configNominalOutputForward(double percentOut) {
        return this.autoRetry(() -> m_talon.configNominalOutputForward(percentOut, kTimeoutMs));
    }

    public TalonEnhanced configNominalOutputReverse(double percentOut) {
        return this.autoRetry(() -> m_talon.configNominalOutputReverse(percentOut, kTimeoutMs));
    }

    public TalonEnhanced configNeutralDeadband(double percentDeadband) {
        return this.autoRetry(() -> m_talon.configNeutralDeadband(percentDeadband, kTimeoutMs));
    }

    public TalonEnhanced configVoltageCompSaturation(double voltage) {
        return this.autoRetry(() -> m_talon.configVoltageCompSaturation(voltage, kTimeoutMs));
    }

    public TalonEnhanced configVoltageMeasurementFilter(int filterWindowSamples) {
        return this.autoRetry(() -> m_talon.configVoltageMeasurementFilter(filterWindowSamples, kTimeoutMs));
    }

    public TalonEnhanced enableVoltageCompensation(boolean enable) {
        m_talon.enableVoltageCompensation(enable);
        return this;
    }

    public double getBusVoltage() {
        return m_talon.getBusVoltage();
    }

    public double getMotorOutputPercent() {
        return m_talon.getMotorOutputPercent();
    }

    public double getMotorOutputVoltage() {
        return m_talon.getMotorOutputVoltage();
    }

    public double getTemperature() {
        return m_talon.getTemperature();
    }

    public TalonEnhanced configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx) {
        return this.autoRetry(() -> m_talon.configSelectedFeedbackSensor(feedbackDevice, pidIdx, kTimeoutMs));
    }

    public TalonEnhanced configSelectedFeedbackCoefficient(double coefficient, int pidIdx) {
        return this.autoRetry(() -> m_talon.configSelectedFeedbackCoefficient(coefficient, pidIdx, kTimeoutMs));
    }

    public TalonEnhanced configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource,
            int remoteOrdinal) {
        return this.autoRetry(
                () -> m_talon.configRemoteFeedbackFilter(deviceID, remoteSensorSource, remoteOrdinal, kTimeoutMs));
    }

    public TalonEnhanced configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal) {
        return this.autoRetry(() -> m_talon.configRemoteFeedbackFilter(canCoderRef, remoteOrdinal, kTimeoutMs));
    }

    public TalonEnhanced configRemoteFeedbackFilter(BaseTalon talonRef, int remoteOrdinal) {
        return this.autoRetry(() -> m_talon.configRemoteFeedbackFilter(talonRef, remoteOrdinal, kTimeoutMs));
    }

    public TalonEnhanced configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice) {
        return this.autoRetry(() -> m_talon.configSensorTerm(sensorTerm, feedbackDevice, kTimeoutMs));
    }

    public double getSelectedSensorPosition(int pidIdx) {
        return m_talon.getSelectedSensorPosition(pidIdx);
    }

    public double getSelectedSensorVelocity(int pidIdx) {
        return m_talon.getSelectedSensorVelocity(pidIdx);
    }

    public TalonEnhanced setSelectedSensorPosition(double sensorPos, int pidIdx) {
        return this.autoRetry(() -> m_talon.setSelectedSensorPosition(sensorPos, pidIdx, kTimeoutMs));
    }

    public TalonEnhanced setControlFramePeriod(ControlFrame frame, int periodMs) {
        return this.autoRetry(() -> m_talon.setControlFramePeriod(frame, periodMs));
    }

    public TalonEnhanced setStatusFramePeriod(StatusFrame frame, int periodMs) {
        return this.autoRetry(() -> m_talon.setStatusFramePeriod(frame, periodMs, kTimeoutMs));
    }

    public int getStatusFramePeriod(StatusFrame frame) {
        return m_talon.getStatusFramePeriod(frame, kTimeoutMs);
    }

    public TalonEnhanced configForwardLimitSwitchSource(RemoteLimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose, int deviceID) {
        return this
                .autoRetry(() -> m_talon.configForwardLimitSwitchSource(type, normalOpenOrClose, deviceID, kTimeoutMs));
    }

    public TalonEnhanced configReverseLimitSwitchSource(RemoteLimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose, int deviceID) {
        return this
                .autoRetry(() -> m_talon.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID, kTimeoutMs));
    }

    public TalonEnhanced overrideLimitSwitchesEnable(boolean enable) {
        m_talon.overrideLimitSwitchesEnable(enable);
        return this;
    }

    public TalonEnhanced configForwardSoftLimitThreshold(double forwardSensorLimit) {
        return this.autoRetry(() -> m_talon.configForwardSoftLimitThreshold(forwardSensorLimit, kTimeoutMs));
    }

    public TalonEnhanced configReverseSoftLimitThreshold(double reverseSensorLimit) {
        return this.autoRetry(() -> m_talon.configReverseSoftLimitThreshold(reverseSensorLimit, kTimeoutMs));
    }

    public TalonEnhanced configForwardSoftLimitEnable(boolean enable) {
        return this.autoRetry(() -> m_talon.configForwardSoftLimitEnable(enable, kTimeoutMs));
    }

    public TalonEnhanced configReverseSoftLimitEnable(boolean enable) {
        return this.autoRetry(() -> m_talon.configReverseSoftLimitEnable(enable, kTimeoutMs));
    }

    public TalonEnhanced overrideSoftLimitsEnable(boolean enable) {
        m_talon.overrideLimitSwitchesEnable(enable);
        return this;
    }

    // TODO Create some PIDF convenience functions

    private double m_last_kP = Double.NaN;

    public TalonEnhanced config_kP(int slotIdx, double value) {
        if (m_last_kP != value) {
            m_last_kP = value;
            return this.autoRetry(() -> m_talon.config_kP(slotIdx, value, kTimeoutMs));
        } else
            return this;
    }

    private double m_last_kI = Double.NaN;

    public TalonEnhanced config_kI(int slotIdx, double value) {
        if (m_last_kI != value) {
            m_last_kI = value;
            return this.autoRetry(() -> m_talon.config_kI(slotIdx, value, kTimeoutMs));
        } else
            return this;
    }

    private double m_last_kD = Double.NaN;

    public TalonEnhanced config_kD(int slotIdx, double value) {
        if (m_last_kD != value) {
            m_last_kD = value;
            return this.autoRetry(() -> m_talon.config_kD(slotIdx, value, kTimeoutMs));
        } else
            return this;
    }

    private double m_last_kF = Double.NaN;

    public TalonEnhanced config_kF(int slotIdx, double value) {
        if (m_last_kF != value) {
            m_last_kF = value;
            return this.autoRetry(() -> m_talon.config_kF(slotIdx, value, kTimeoutMs));
        } else
            return this;
    }

    private double m_lastIzone = Double.NaN;

    public TalonEnhanced config_IntegralZone(int slotIdx, double izone) {
        if (m_lastIzone != izone) {
            m_lastIzone = izone;
            return this.autoRetry(() -> m_talon.config_IntegralZone(slotIdx, izone, kTimeoutMs));
        } else
            return this;
    }

    private double m_lastAllowableCloseLoopError = Double.NaN;

    public TalonEnhanced configAllowableClosedloopError(int slotIdx, double allowableCloseLoopError) {
        if (m_lastAllowableCloseLoopError != allowableCloseLoopError) {
            m_lastAllowableCloseLoopError = allowableCloseLoopError;
            return this.autoRetry(
                    () -> m_talon.configAllowableClosedloopError(slotIdx, allowableCloseLoopError, kTimeoutMs));
        } else
            return this;
    }

    private double m_lastMaxIaccum = Double.NaN;

    public TalonEnhanced configMaxIntegralAccumulator(int slotIdx, double maxIaccum) {
        if (m_lastMaxIaccum != maxIaccum) {
            m_lastMaxIaccum = maxIaccum;
            return this.autoRetry(() -> m_talon.configMaxIntegralAccumulator(slotIdx, maxIaccum, kTimeoutMs));
        } else
            return this;
    }

    public TalonEnhanced configClosedLoopPeakOutput(int slotIdx, double percentOut) {
        return this.autoRetry(() -> m_talon.configClosedLoopPeakOutput(slotIdx, percentOut, kTimeoutMs));
    }

    public TalonEnhanced configClosedLoopPeriod(int slotIdx, int loopTimeMs) {
        return this.autoRetry(() -> m_talon.configClosedLoopPeriod(slotIdx, loopTimeMs, kTimeoutMs));
    }

    public TalonEnhanced configAuxPIDPolarity(boolean invert) {
        return this.autoRetry(() -> m_talon.configAuxPIDPolarity(invert, kTimeoutMs));
    }

    public TalonEnhanced setIntegralAccumulator(double iaccum, int pidIdx) {
        return this.autoRetry(() -> m_talon.setIntegralAccumulator(iaccum, pidIdx, kTimeoutMs));
    }

    public double getClosedLoopError(int pidIdx) {
        return m_talon.getClosedLoopError(pidIdx);
    }

    public double getIntegralAccumulator(int pidIdx) {
        return m_talon.getIntegralAccumulator(pidIdx);
    }

    public double getErrorDerivative(int pidIdx) {
        return m_talon.getErrorDerivative(pidIdx);
    }

    public TalonEnhanced selectProfileSlot(int slotIdx, int pidIdx) {
        m_talon.selectProfileSlot(slotIdx, pidIdx);
        return this;
    }

    public double getClosedLoopTarget(int pidIdx) {
        return m_talon.getClosedLoopTarget(pidIdx);
    }

    public double getActiveTrajectoryPosition() {
        return m_talon.getActiveTrajectoryPosition();
    }

    public double getActiveTrajectoryVelocity() {
        return m_talon.getActiveTrajectoryVelocity();
    }

    // TODO Write motion magic convenience methods.

    private double m_lastCruiseVelocity = Double.NaN;

    public TalonEnhanced configMotionCruiseVelocity(double cruiseVelocity) {
        if (m_lastCruiseVelocity != cruiseVelocity) {
            return this.autoRetry(() -> m_talon.configMotionCruiseVelocity(cruiseVelocity, kTimeoutMs));
        } else
            return this;
    }

    private double m_lastAccel = Double.NaN;

    public TalonEnhanced configMotionAcceleration(double accel) {
        if (m_lastAccel != accel) {
            m_lastAccel = accel;
            return this.autoRetry(() -> m_talon.configMotionAcceleration(accel, kTimeoutMs));
        } else
            return this;
    }

    public TalonEnhanced configMotionSCurveStrength(int curveStrength) {
        return this.autoRetry(() -> m_talon.configMotionSCurveStrength(curveStrength, kTimeoutMs));
    }

    public TalonEnhanced configMotionProfileTrajectoryPeriod(int baseTrajDurationMs) {
        return this.autoRetry(() -> m_talon.configMotionProfileTrajectoryPeriod(baseTrajDurationMs, kTimeoutMs));
    }

    public TalonEnhanced clearMotionProfileTrajectories() {
        return this.autoRetry(() -> m_talon.clearMotionProfileTrajectories());
    }

    public int getMotionProfileTopLevelBufferCount() {
        return m_talon.getMotionProfileTopLevelBufferCount();
    }

    public TalonEnhanced pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        return this.autoRetry(() -> m_talon.pushMotionProfileTrajectory(trajPt));
    }

    public boolean isMotionProfileTopLevelBufferFull() {
        return m_talon.isMotionProfileTopLevelBufferFull();
    }

    public TalonEnhanced processMotionProfileBuffer() {
        m_talon.processMotionProfileBuffer();

        return this;
    }

    public TalonEnhanced getMotionProfileStatus(MotionProfileStatus statusToFill) {
        return this.autoRetry(() -> m_talon.getMotionProfileStatus(statusToFill));
    }

    public TalonEnhanced clearMotionProfileHasUnderrun() {
        return this.autoRetry(() -> m_talon.clearMotionProfileHasUnderrun(kTimeoutMs));
    }

    public TalonEnhanced changeMotionControlFramePeriod(int periodMs) {
        return this.autoRetry(() -> m_talon.changeMotionControlFramePeriod(periodMs));
    }

    public ErrorCode getLastError() {
        return m_talon.getLastError();
    }

    // TODO Implement automatic fault monitoring/handling. Perhaps divide faults
    // TODO into "categories" such as limits/soft limits, hardware/api issues, and
    // TODO non-critical "warning" faults.
    public TalonEnhanced getFaults(Faults toFill) {
        return this.autoRetry(() -> m_talon.getFaults(toFill));
    }

    public TalonEnhanced getStickyFaults(StickyFaults toFill) {
        return this.autoRetry(() -> m_talon.getStickyFaults(toFill));
    }

    public TalonEnhanced clearStickyFaults() {
        return this.autoRetry(() -> m_talon.clearStickyFaults(kTimeoutMs));
    }

    public int getFirmwareVersion() {
        return m_talon.getFirmwareVersion();
    }

    // TODO Automatically re-instate config when this happens. Perhaps save a live
    // TODO "copy" of the config in a varialbe?
    public boolean hasResetOccurred() {
        return m_talon.hasResetOccurred();
    }

    public TalonEnhanced configSetCustomParam(int newValue, int paramIndex) {
        return this.autoRetry(() -> m_talon.configSetCustomParam(newValue, paramIndex, kTimeoutMs));
    }

    public int configGetCustomParam(int paramIndex) {
        return m_talon.configGetCustomParam(paramIndex, kTimeoutMs);
    }

    public TalonEnhanced configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
        return this.autoRetry(() -> m_talon.configSetParameter(param, value, subValue, ordinal, kTimeoutMs));
    }

    public TalonEnhanced configSetParameter(int param, double value, int subValue, int ordinal) {
        return this.autoRetry(() -> m_talon.configSetParameter(param, value, subValue, ordinal, kTimeoutMs));
    }

    public double configGetParameter(ParamEnum paramEnum, int ordinal) {
        return m_talon.configGetParameter(paramEnum, ordinal, kTimeoutMs);
    }

    public double configGetParameter(int paramEnum, int ordinal) {
        return m_talon.configGetParameter(paramEnum, ordinal, kTimeoutMs);
    }

    public int getBaseID() {
        return m_talon.getBaseID();
    }

    public int getDeviceID() {
        return m_talon.getDeviceID();
    }

    public ControlMode getControlMode() {
        return m_talon.getControlMode();
    }

    public TalonEnhanced follow(IMotorController masterToFollow) {
        m_talon.follow(masterToFollow);

        return this;
    }

    public TalonEnhanced follow(TalonEnhanced masterToFollow) {
        return this.follow(masterToFollow.m_talon);
    }

    public TalonEnhanced configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx) {
        return this.autoRetry(() -> m_talon.configSelectedFeedbackSensor(feedbackDevice, pidIdx, kTimeoutMs));
    }

    public TalonEnhanced configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg) {
        return this.autoRetry(() -> m_talon.configSupplyCurrentLimit(currLimitCfg, kTimeoutMs));
    }

    public TalonEnhanced setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs) {
        // TODO Create custom frame enum which better represents common use cases
        return this.autoRetry(() -> m_talon.setStatusFramePeriod(frame, periodMs, kTimeoutMs));
    }

    public int getStatusFramePeriod(StatusFrameEnhanced frame) {
        // TODO Create custom frame enum which better represents common use cases
        return m_talon.getStatusFramePeriod(frame, kTimeoutMs);
    }

    // TODO Refactor these methods using custom frame enum
    public TalonEnhanced setFeedbackIntervals(int intervalMs) {
        return this
                .autoRetry(() -> m_talon.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_2_Feedback0, intervalMs, kTimeoutMs))
                .autoRetry(() -> m_talon.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_Brushless_Current, intervalMs, kTimeoutMs));
    }

    public TalonEnhanced setControlIntervals(int intervalMs) {
        return this.autoRetry(() -> m_talon.setControlFramePeriod(ControlFrame.Control_3_General, intervalMs));
    }

    public TalonEnhanced setAllStatusIntervals(int intervalMs) {
        for (StatusFrameEnhanced frame : StatusFrameEnhanced.values()) {
            this.autoRetry(() -> m_talon.setStatusFramePeriod(frame, intervalMs, kTimeoutMs));
        }
        return this;
    }

    public TalonEnhanced defaultFrameIntervals() {
        return this.setAllStatusIntervals(kSlowFrameMs).setControlIntervals(kFastFrameMs);
    }

    public double getOutputCurrent() {
        // TODO check the subtype of the IMotorControllerEnhanced passed into the
        // TODO constructor, and implement getSupplyCurrent/getStatorCurrent
        // TODO methods in TalonEnhanced.
        return m_talon.getOutputCurrent();
    }

    public TalonEnhanced configVelocityMeasurementPeriod(SensorVelocityMeasPeriod period) {
        return this.autoRetry(() -> m_talon.configVelocityMeasurementPeriod(period, kTimeoutMs));
    }

    public TalonEnhanced configVelocityMeasurementWindow(int windowSize) {
        return this.autoRetry(() -> m_talon.configVelocityMeasurementWindow(windowSize, kTimeoutMs));
    }

    public TalonEnhanced configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
        return this.autoRetry(() -> m_talon.configForwardLimitSwitchSource(type, normalOpenOrClose, kTimeoutMs));
    }

    public TalonEnhanced configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
        return this.autoRetry(() -> m_talon.configReverseLimitSwitchSource(type, normalOpenOrClose, kTimeoutMs));
    }
}