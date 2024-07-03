package frc.robot.lib6907;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import java.util.Collections;
import java.util.NavigableMap;
import java.util.Objects;
import java.util.TreeMap;

public class CircularInterpolatingTreeMap<K, V> {
  private final NavigableMap<K, V> buffer;
  private final int capacity;
  private final InverseInterpolator<K> inverseInterpolator;
  private final Interpolator<V> interpolator;

  public CircularInterpolatingTreeMap(
      int maxSize, InverseInterpolator<K> inverseInterpolator, Interpolator<V> interpolator) {
    if (maxSize <= 0) {
      throw new IllegalArgumentException("Capacity must be positive");
    }
    this.capacity = maxSize;
    this.buffer = Collections.synchronizedNavigableMap(new TreeMap<>());
    this.inverseInterpolator =
        Objects.requireNonNull(inverseInterpolator, "inverseInterpolator must not be null");
    this.interpolator = Objects.requireNonNull(interpolator, "interpolator must not be null");
  }

  public void put(K key, V value) {
    if (buffer.size() >= capacity) {
      buffer.pollFirstEntry();
    }
    buffer.put(key, value);
  }

  public V get(K key) {
    if (buffer.isEmpty()) {
      return null;
    }
    V val = buffer.get(key);
    if (val == null) {
      K ceilingKey = buffer.ceilingKey(key);
      K floorKey = buffer.floorKey(key);

      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return buffer.get(floorKey);
      }
      if (floorKey == null) {
        return buffer.get(ceilingKey);
      }
      V floor = buffer.get(floorKey);
      V ceiling = buffer.get(ceilingKey);

      return interpolator.interpolate(
          floor, ceiling, inverseInterpolator.inverseInterpolate(floorKey, ceilingKey, key));
    } else {
      return val;
    }
  }

  public void clear() {
    buffer.clear();
  }
}
