package lib.controllers;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import jdk.jfr.Event;

public class FootPedal extends CommandGenericHID {
    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public FootPedal(int port) {
        super(port);
    }

    public enum Pedals {
        kBlankValue,
        kLeft,
        kMiddle,
        kRight
    }

    public boolean getLeftPedal() {
        return getHID().getRawButtonPressed(Pedals.kLeft.ordinal());
    }

    public boolean getMiddlePedal() {
        return getHID().getRawButtonPressed(Pedals.kMiddle.ordinal());
    }

    public boolean getRightPedal() {
        return getHID().getRawButtonPressed(Pedals.kRight.ordinal());
    }

    public Trigger leftPedal(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftPedal).castTo(Trigger::new);
    }

    public Trigger leftPedal() {
        return leftPedal(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger middlePedal(EventLoop loop) {
        return new BooleanEvent(loop, this::getMiddlePedal).castTo(Trigger::new);
    }

    public Trigger middlePedal() {
        return middlePedal(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightPedal(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightPedal).castTo(Trigger::new);
    }

    public Trigger rightPedal() {
        return rightPedal(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
}
