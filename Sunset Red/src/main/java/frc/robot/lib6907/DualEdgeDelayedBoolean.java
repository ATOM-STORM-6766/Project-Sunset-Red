package frc.robot.lib6907;

public class DualEdgeDelayedBoolean {
  private boolean mLastValue;
  private double mRisingEdgeTimestamp;
  private double mFallingEdgeTimestamp;
  private final double mDelay;
  private final EdgeType mEdgeType;
  private boolean mState;

  public enum EdgeType {
    RISING,
    FALLING
  }

  public DualEdgeDelayedBoolean(double timestamp, double delay, EdgeType edgeType) {
    mRisingEdgeTimestamp = timestamp;
    mFallingEdgeTimestamp = timestamp;
    mLastValue = false;
    mDelay = delay;
    mEdgeType = edgeType;
    mState = false;
  }

  public boolean get() {
    return mState;
  }

  public boolean update(double timestamp, boolean value) {
    if (value && !mLastValue) {
      mRisingEdgeTimestamp = timestamp;
    } else if (!value && mLastValue) {
      mFallingEdgeTimestamp = timestamp;
    }

    switch (mEdgeType) {
      case RISING:
        mState = value && (timestamp - mRisingEdgeTimestamp > mDelay);
        break;
      case FALLING:
        mState = value || (timestamp - mFallingEdgeTimestamp < mDelay);
        break;
    }

    mLastValue = value;
    return mState;
  }
}
