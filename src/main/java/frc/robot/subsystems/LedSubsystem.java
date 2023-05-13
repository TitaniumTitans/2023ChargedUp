package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final CANdle m_candle;
    private final int m_numLED = 400;
    private Animations m_currentAnimation = Animations.RAINBOW;

    private final Animation m_rainbow = new RainbowAnimation(1.0, 0.5, m_numLED);
    private final Animation m_red = new ColorFlowAnimation(255, 0, 0, 0, 0.5, m_numLED, Direction.Forward);
    private final Animation m_blue = new ColorFlowAnimation(0, 255, 0, 0, 0.5, m_numLED, Direction.Forward);
    private final Animation m_fire = new FireAnimation(1.0, 0.5, m_numLED, 0.25, 0.25);
    private final Animation m_larson = new LarsonAnimation(255, 0, 0, 0, 0.5, m_numLED, BounceMode.Front, 20);

    public enum Animations {
        RAINBOW,
        RED,
        BLUE,
        FIRE,
        LARSON
    }

    public LedSubsystem(int id) {
        m_candle = new CANdle(id);
        m_candle.animate(m_rainbow);
    }

    public void nextAnimation() {
        m_currentAnimation = Animations.values()[m_currentAnimation.ordinal() + 1 < 5 ? 0 : m_currentAnimation.ordinal() + 1];
    }

    public void setAnimation(Animations animation) {
        m_currentAnimation = animation;
    }

    public Animations getAnimation() {
        return m_currentAnimation;
    }

    @Override
    public void periodic() {
        switch(m_currentAnimation){
            case BLUE:
                m_candle.animate(m_blue);
                break;
            case FIRE:
                m_candle.animate(m_fire);
                break;
            case LARSON:
                m_candle.animate(m_larson);
                break;
            case RAINBOW:
                m_candle.animate(m_rainbow);
                break;
            case RED:
                m_candle.animate(m_red);
                break;
            default:
                m_candle.setLEDs(255, 0, 255);
                break;
            
        }
    }
    
}
