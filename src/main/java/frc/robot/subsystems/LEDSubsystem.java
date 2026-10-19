package frc.robot.subsystems;

import java.io.ObjectInput;
import java.lang.reflect.Array;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;


/*public class LEDSubsystem extends SubsystemBase {
    private final Spark blinkin;
    
    // Blinkin pattern values from REV documentation
    // These are the PWM values (-1.0 to 1.0) that map to preset patterns
    public static final double RAINBOW = -0.99;
    public static final double CHASE_BLUE = -0.71;
    public static final double STROBE_GOLD = -0.07;
    public static final double SOLID_RED = 0.61;
    public static final double SOLID_GREEN = 0.77;
    public static final double SOLID_GOLD = 0.67;
    public static final double SOLID_BLUE = 0.87;
    public static final double COLOR1_LARSON = -0.35; // scanner/chase
    public static final double HEARTBEAT_RED = -0.25;
    public static final double BREATH_BLUE = -0.15;

    // Pattern constants
    private static final double SEARCHING     = -0.99; // rainbow = scanning
    private static final double LOW_CONF      = 0.61;  // solid red = sees tag, low confidence
    private static final double MED_CONF      = 0.67;  // solid gold = getting there
    private static final double HIGH_CONF     = 0.77;  // solid green = locked and confident
    private static final double LOCKED_FLASH  = -0.11; // strobe to celebrate lock
    private static final double STROBE_RED   = -0.11;    

    //public void LEDblinkin() {
    //    blinkin = new Spark(0); // PWM port number
    //}
    
    public void setPattern(double value) {
        blinkin.set(value);
    }
    private final NetworkTable limelight;
    
    // State tracking
    private int consecutiveFrames = 0;
    private final int REQUIRED_FRAMES = 5; // debounce

    private final Timer timer;




    //public LEDSubsystem() {
    //    blinkin = new Spark(0);
    //    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    //}

    private void updateLEDState() {
        double tv = limelight.getEntry("tv").getDouble(0);
        double ta = limelight.getEntry("ta").getDouble(0);

        // Pull MegaTag2 pose stddevs for confidence
        double[] botpose = limelight.getEntry("botpose_orb_wpiblue")
                                    .getDoubleArray(new double[11]);

        double xStdDev = (botpose.length > 7) ? botpose[6] : 999;
        double yStdDev = (botpose.length > 7) ? botpose[7] : 999;

        if (tv == 0) {
            consecutiveFrames = 0;
            blinkin.set(SEARCHING);
            return;
        }

        // Score confidence 0.0 - 1.0
        double taScore    = Math.min(ta / 8.0, 1.0);
        double stdScore   = Math.max(0, 1.0 - ((xStdDev + yStdDev) / 2.0));
        double confidence = (taScore * 0.4) + (stdScore * 0.6); // weight stddev more

        if (confidence < 0.33) {
            consecutiveFrames = 0;
            blinkin.set(LOW_CONF);

        } else if (confidence < 0.66) {
            consecutiveFrames = 0;
            blinkin.set(MED_CONF);

        } else {
            consecutiveFrames++;

            if (consecutiveFrames >= REQUIRED_FRAMES) {
                blinkin.set(LOCKED_FLASH); // fully locked, strobe to signal drivers
            } else {
                blinkin.set(HIGH_CONF); // almost there
            }
        }
    }

    // Call this from your command or robot container if needed
    public boolean isLocked() {
        return consecutiveFrames >= REQUIRED_FRAMES;
    }


double timeRemaining = 10.0 - timer.get();
int totalLEDs = 8;
int litLEDs = (int)((timeRemaining / 10.0) * totalLEDs);

/*for (int i = 0; i < totalLEDs; i++) {
    Object ledBuffer;
    if (i < litLEDs) {
        // shift color red to green as countdown progresses
        int r = (int)(255 * (timeRemaining / 10.0));
        int g = (int)(255 * (1.0 - timeRemaining / 10.0));
        ledBuffer.setRGB(i, r, g, 0);
    } else {
        ledBuffer.setRGB(i, 0, 0, 0)

    private boolean countdownActive = false;

    // How many seconds before end to start blinking red
    private static final double WARNING_TIME = 3.0; // last 3 seconds
    private static final double COUNTDOWN    = 10.0;

    public void startCountdown() {
        timer.reset();
        timer.start();
        countdownActive = true;
    }

    public void stopCountdown() {
        timer.stop();
        countdownActive = false;
    }

    @Override
    public void periodic() {
        if (!countdownActive) {
            blinkin.set(SEARCHING);
            return;
        }

        double elapsed = timer.get();
        double remaining = COUNTDOWN - elapsed;

        if (remaining <= 0) {
            // Countdown finished
            stopCountdown();
            blinkin.set(STROBE_RED);
            return;
        }

        if (remaining <= WARNING_TIME) {
            blinkin.set(STROBE_RED); // last 3 seconds, blink red
        } else {
            blinkin.set(SOLID_GREEN); // solid green while time is good
        }
    }

    public boolean isCountdownDone() {
        return timer.get() >= COUNTDOWN;
    }
} */