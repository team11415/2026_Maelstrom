package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Subsystem for controlling LED lights through a CTRE CANdle.
 * 
 * Think of the CANdle as a "translator" — it sits on the CAN bus
 * and converts commands into the precise signal patterns that
 * individually addressable LEDs understand.
 */
public class LEDs extends SubsystemBase {

    // The CANdle hardware device — ID 7 on the CANivore bus
    private final CANdle candle = new CANdle(7, Constants.kCANBus);

    // LED index range:
    // 0-7 = the 8 onboard LEDs on the CANdle itself
    // 8-167 = the 160 LEDs on your external strip
    // So the full range is 0 (first onboard) to 167 (last strip LED)
    private static final int FIRST_LED = 0;
    private static final int LAST_LED = 167;  // 8 onboard + 160 strip - 1

    // Our team color: #008E85 (teal)
    private static final RGBWColor TEAM_COLOR = new RGBWColor(0, 142, 133, 0);

    // ---- Control Requests ----
    // These work just like DutyCycleOut for motors — you create them once,
    // then reuse them by passing them to setControl().

    // Chase animation: lights flow down the strip one LED at a time
    // Constructor takes (startIndex, endIndex) — the range of LEDs to control
    private final ColorFlowAnimation chaseAnimation = new ColorFlowAnimation(FIRST_LED, LAST_LED)
        .withColor(TEAM_COLOR)                             // Set our teal color
        .withSlot(0)                                       // Use animation slot 0
        .withFrameRate(25)                                 // 25 frames per second
        .withDirection(AnimationDirectionValue.Forward);    // Chase moves forward

    // Solid color: all LEDs show the same color at once
    private final SolidColor solidColor = new SolidColor(FIRST_LED, LAST_LED)
        .withColor(TEAM_COLOR);

    // Empty animation: clears the animation in slot 0 (turns LEDs off)
    private final EmptyAnimation offAnimation = new EmptyAnimation(0);

    public LEDs() {
        // Configure the CANdle's settings
        CANdleConfiguration config = new CANdleConfiguration();

        // Set the LED strip type to match your WS2812B RGB strip
        config.LED.StripType = StripTypeValue.RGB;

        // Set brightness (0.0 to 1.0) — start at 75%
        config.LED.BrightnessScalar = 0.75;

        // Apply the configuration to the CANdle
        candle.getConfigurator().apply(config);
    }

    /** Runs the chase/color flow animation in our team color */
    public void runChase() {
        candle.setControl(chaseAnimation);
    }

    /** Sets all LEDs to a solid team color */
    public void runSolid() {
        candle.setControl(solidColor);
    }

    /** Sets all LEDs to a custom solid color */
    public void runSolidColor(int r, int g, int b) {
        candle.setControl(solidColor.withColor(new RGBWColor(r, g, b, 0)));
    }

    /** Turns off all LEDs */
    public void turnOff() {
        candle.setControl(offAnimation);
    }
}