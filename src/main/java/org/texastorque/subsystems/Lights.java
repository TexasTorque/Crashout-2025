package org.texastorque.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueStatelessSubsystem;
import org.texastorque.torquelib.util.TorqueUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;


public final class Lights extends TorqueStatelessSubsystem implements Subsystems {

    public static class Solid extends LightAction {
        private final Supplier<Color> color;

        public Solid(final Supplier<Color> color) {
            this.color = color;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            for (int i = 0; i < buff.getLength(); i++)
                buff.setLED(i, color.get());
        }
    }

    public static class Blink extends LightAction {
        private final Supplier<Color> color1, color2;
        private final double hertz;

        public Blink(final Supplier<Color> color1, final double hertz) {
            this(color1, () -> Color.kBlack, hertz);
        }

        public Blink(final Supplier<Color> color1, final Supplier<Color> color2, final double hertz) {
            this.color1 = color1;
            this.color2 = color2;
            this.hertz = hertz;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            final double timestamp = TorqueUtil.time();
            final boolean on = (Math.floor(timestamp * hertz) % 2 == 1);
            for (int i = 0; i < buff.getLength(); i++)
                buff.setLED(i, on ? color1.get() : color2.get());
        }
    }

    public static class Rainbow extends LightAction {
        private int rainbowFirstPixelHue = 0;

        @Override
        public void run(AddressableLEDBuffer buff) {
            for (var i = 0; i < buff.getLength(); i++) {
                final int hue = (rainbowFirstPixelHue + (i * 180 / buff.getLength())) % 180;
                buff.setHSV(i, hue, 255, 128);
            }
            rainbowFirstPixelHue += 3;
            rainbowFirstPixelHue %= 180;
        }
    }


    private static abstract class LightAction {
        public abstract void run(AddressableLEDBuffer buff);
    }

    private static volatile Lights instance;

    private static final int LENGTH = 23;

    public static final synchronized Lights getInstance() {
        return instance == null ? instance = new Lights() : instance;
    }

    private final List<AddressableLED> lights;
    private final AddressableLEDBuffer buff;

    public static final double HERTZ = 15;

    private LightAction alliance = new Solid(() -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? Color.kRed : Color.kBlue),
            green = new Solid(() -> Color.kGreen),
            yellow = new Solid(() -> Color.kYellow),
            white = new Solid(() -> Color.kWhite),
            blinkGreen = new Blink(() -> Color.kGreen, HERTZ),
            rainbow = new Rainbow();

    private Lights() {
        lights = new ArrayList<>();
        buff = new AddressableLEDBuffer(LENGTH);

        createStrips(Ports.LIGHTS);
    }


    private void createStrips(int... ports) {
        for (int i = 0; i < buff.getLength(); i++) {
            buff.setLED(i, Color.kRed);
        }

        for (int port : ports) {
            final AddressableLED strip = new AddressableLED(port);
            strip.setLength(LENGTH);
            lights.add(strip);
        }

        setData(buff);
    }


    private void setData(final AddressableLEDBuffer buff) {
        for (final AddressableLED strip : lights) {
            strip.setData(buff);
        }
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        for (final AddressableLED strip : lights) {
            strip.start(); 
        }
    }

    public final LightAction getColor(final TorqueMode mode) {
        if (drivebase.getState() == Drivebase.State.ALIGN) return blinkGreen;
        if (elevator.getState() == Elevator.State.CLIMB) return rainbow;
        if (DriverStation.isDisabled()) return white;
        if (perception.seesTag()) return green;
        if (claw.hasCoral()) return yellow;

        return alliance;
    }

    @Override
    public final void update(final TorqueMode mode) {
        getColor(mode).run(buff);
        setData(buff);
    }

    @Override
    public void clean(TorqueMode mode) {}
}