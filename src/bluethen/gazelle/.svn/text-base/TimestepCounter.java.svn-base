package bluethen.gazelle;

/*
 * Timestep Counter
 * Tracks the amount of constant timesteps to use
 */

public class TimestepCounter {
	
	int leftOverDeltaTime;
	int timestepCount;
	int fixedTimestepSizeMS;
	float fixedTimestepSizeSeconds;
	int timestepCap;

	TimestepCounter() {
		this(15);
	}

	TimestepCounter(int fixedTimestepSize) {
		this(fixedTimestepSize, 4);
	}

	TimestepCounter(int fixedTimestepSize, int timestepCap) {
		this.fixedTimestepSizeMS = fixedTimestepSize;
		fixedTimestepSizeSeconds = fixedTimestepSizeMS / 1000.0f;
		this.timestepCap = timestepCap;
	}

	void update(int elapsedTime) {
		timestepCount += (int) ((float) (elapsedTime + leftOverDeltaTime) / (float) fixedTimestepSizeMS);
		leftOverDeltaTime += elapsedTime
				- (timestepCount * fixedTimestepSizeMS);
		if (timestepCount > timestepCap) {
			// System.out.println("Timestep count capped: " + timestepCount +
			// " -> " + timestepCap);
			timestepCount = timestepCap;
		}
	}

	boolean hasNext() {
		if (timestepCount > 0)
			return true;
		return false;
	}

	void decrement() {
		timestepCount--;
	}

	float getTimestep() {
		return fixedTimestepSizeSeconds;
	}
}
