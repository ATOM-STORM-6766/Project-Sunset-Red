package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LED extends SubsystemBase {

    AddressableLED mLED;
    AddressableLEDBuffer mLEDBuffer;

    // private DigitalInput mIntakeEnterOmron;

    private enum LEDState {
            Disabled,
            Idle,
            Intaking, 
            Shooting_Spinning, 
            Shooting,
            Outtake,
            HasRing
    }
    private final Intake sIntake;

    private LEDState mLEDState = LEDState.Disabled;


    private int mRainbowFirstPixelHue = 0;
    private final int LED_LENGTH = 60;

    public LED(Intake intake) {
        sIntake =intake;
        mLED = new AddressableLED(Constants.LED_PORT);
        mLEDBuffer = new AddressableLEDBuffer(LED_LENGTH);
        mLED.setLength(LED_LENGTH);
        mLED.setData(mLEDBuffer);
        mLED.start();
        // mIntakeEnterOmron = new DigitalInput(Constants.INTAKER_ENTER_OMRON_ID);
    }


    
    @Override
    public void periodic() {
        determineState();
        SmartDashboard.putString("LED: State", mLEDState.toString());
        switch (mLEDState) {
            case Disabled:
                setTransition(Color.kMagenta, Color.kCyan);
                break;
            case Idle:
                setSolidColor(new Color(256,0,0));
                break;
            case HasRing:
                setSolidColor(new Color(0,256,0));
                break;
            default:
                break;
        }

        mLED.setData(mLEDBuffer);
    }
    

    
    private synchronized void determineState() {
        boolean disabled = DriverStation.isDisabled();
        if (disabled) {
            mLEDState = LEDState.Disabled;
        }else if(sIntake.isOmronDetected()){
            mLEDState = LEDState.HasRing;
        }else{
            mLEDState = LEDState.Idle;
        }
    }

    private synchronized void setRainbow() {
        // int temp = Math.min(LED_LENGTH-1,(int)(LED_LENGTH * Shooter.getInstance().getSpinningPercentage()));
        // if(temp >= 42){
        //     mRainbowFirstPixelHue = (mRainbowFirstPixelHue + 1) % 180;
        //     for (int i = 0; i < LED_LENGTH; i++) {
        //         final int hue = (mRainbowFirstPixelHue + (i * 180 / LED_LENGTH)) % 180;
        //         mLEDBuffer.setHSV(i, hue, 255, 128);
        //     }
        //     mRainbowFirstPixelHue += 3;
        //     mRainbowFirstPixelHue %= 180;
        // } else {
        //     try{
        //         for (int i = 0; i < temp; i++) {
        //             final int hue = (i * 180 / LED_LENGTH) % 180;
        //             mLEDBuffer.setHSV(i, hue, 255, 128);
        //         }
        //         for (int i = temp; i < LED_LENGTH; i++) {
        //             mLEDBuffer.setRGB(i, 0, 0, 0);
        //         }
        //     } catch (Exception e) {
        //         e.printStackTrace();
        //     }
        // }
    }


    private void setTransition(Color from, Color to) { // int[] sorted by r, g, b
        Color8Bit from8bit = new Color8Bit(from);
        Color8Bit to8bit = new Color8Bit(to);
        int length = LED_LENGTH;
        for (int i = 0; i < length; i ++) {
            mLEDBuffer.setRGB(
                i,
                (int) (from8bit.red * (length - i) + to8bit.red * i) / length,
                (int) (from8bit.green * (length - i) + to8bit.green * i) / length,
                (int) (from8bit.blue * (length - i) + to8bit.blue * i) / length
            );
        }
    }

    private int brightPosition = 0;
    private final double TAIL_LENGTH = 30.0;

    public synchronized void setRisingColor(Color color) {
        Color8Bit color8bit = new Color8Bit(color);
        brightPosition += 1;
        if (brightPosition >= LED_LENGTH)
            brightPosition = 0;
        for (int i = 0; i < LED_LENGTH; i++) {
            mLEDBuffer.setRGB(i, 0, 0, 0);
        }
        for (int i = 0; i < TAIL_LENGTH; i++) {
            mLEDBuffer.setRGB((brightPosition + LED_LENGTH + i)%LED_LENGTH, (int) (color8bit.red / TAIL_LENGTH * i),
                    (int) (color8bit.green / TAIL_LENGTH * i), (int) (color8bit.blue / TAIL_LENGTH * i));
        }
    }
    
    public void setBlinkingColor(Color color) {
        if (Timer.getFPGATimestamp() % 1 > 0.5) {
            for (int i = 0; i < LED_LENGTH; i++) {
                mLEDBuffer.setLED(i, color);
            }

        } else {
            for (int i = 0; i < LED_LENGTH; i++) {
                mLEDBuffer.setLED(i, Color.kBlack);
            }
        }
    }

    public void setSolidColor(Color color) {
        for (int i = 0; i < LED_LENGTH; i++){
            mLEDBuffer.setLED(i, color);
        }
    }


    
}
