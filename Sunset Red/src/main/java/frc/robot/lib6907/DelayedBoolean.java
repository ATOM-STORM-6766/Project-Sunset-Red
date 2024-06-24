package frc.robot.lib6907;

public class DelayedBoolean {
  private boolean mLastValue;
  private double mTransitionTimestamp;
  private final double mDelay;

  public DelayedBoolean(double timestamp, double delay) {
    mTransitionTimestamp = timestamp;
    mLastValue = false;
    mDelay = delay;
  }

  public boolean get(){
    return mLastValue;
  }

  public boolean update(double timestamp, boolean value) {
    boolean result = false;

    if (value && !mLastValue) {
      mTransitionTimestamp = timestamp;
    }

    // If we are still true and we have transitioned.
    if (value && (timestamp - mTransitionTimestamp > mDelay)) {
      result = true;
    }

    mLastValue = value;
    return result;
  }
}
