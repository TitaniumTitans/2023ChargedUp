package frc.robot.subsystems.candle;


import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle m_CANdle;
    private Animation m_toAnim = null;

    public enum AnimationType {
        ColorFlow,
        Fire,
        Rainbow,
        Strobe,
        Twinkle,
        SetAll
    }

    private AnimationType m_currentAnim;

    public CANdleSubsystem() {
        m_CANdle = new CANdle(Constants.CANdleConstants.CANDLE_ID);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = CANdle.LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
        m_CANdle.configAllSettings(configAll, 100);
    }

    @Override
    public void periodic() {
        m_CANdle.animate(m_toAnim);
    }

    public void changeAnimation(AnimationType anim) {
        m_currentAnim = anim;

        switch (anim) {
            case ColorFlow: m_toAnim = Constants.CANdleConstants.COLOR_FLOW_ANIM; break;
            case Fire: m_toAnim = Constants.CANdleConstants.FIRE_ANIM; break;
            case Strobe: m_toAnim = Constants.CANdleConstants.STROBE; break;
            case Twinkle: m_toAnim = Constants.CANdleConstants.TWINKLE_ANIM; break;
            case Rainbow: m_toAnim = Constants.CANdleConstants.RAINBOW_ANIM; break;
            case SetAll: m_toAnim = Constants.CANdleConstants.SET_ALL_ANIM; break;
        }
    }
    public void incrementAnim() {
        switch (m_currentAnim) {
            case ColorFlow: changeAnimation(AnimationType.Fire); break;
            case Fire: changeAnimation(AnimationType.Strobe); break;
            case Strobe: changeAnimation(AnimationType.Twinkle); break;
            case Twinkle: changeAnimation(AnimationType.Rainbow); break;
            case Rainbow: changeAnimation(AnimationType.ColorFlow); break;
        }
    }

    public void decrementAnim() {
        switch (m_currentAnim)
        {
            case ColorFlow: changeAnimation(AnimationType.Rainbow); break;
            case Fire: changeAnimation(AnimationType.ColorFlow); break;
            case Strobe: changeAnimation(AnimationType.Fire); break;
            case Twinkle: changeAnimation(AnimationType.Strobe); break;
            case Rainbow: changeAnimation(AnimationType.Twinkle); break;
        }
    }

    public double getVBat() { return m_CANdle.getBusVoltage(); }
    public double get5V() { return m_CANdle.get5VRailVoltage(); }
    public double getCurrent() { return m_CANdle.getCurrent(); }
    public double getTemperature() { return m_CANdle.getTemperature(); }
    public void configBrightness(double percent) { m_CANdle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_CANdle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(CANdle.LEDStripType type) { m_CANdle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_CANdle.configStatusLedState(offWhenActive, 0); }

    public Command setAnimationFactory (AnimationType anim) {
        return runOnce(() -> changeAnimation(anim));
    }
    public Command incrementAnimationFactory () {
        return runOnce(() -> incrementAnim());
    }
    public Command decrementAnimationFactory () {
        return runOnce(() -> decrementAnim());
    }
}

