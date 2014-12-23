package bluethen.gazelle.util;

public class PMath {
	
	public static int constraint(int val, int min, int max) {
		return Math.min(Math.max(val, min), max);
	}

	public static float dist(float x1, float y1, float x2, float y2) {
		return (float) Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
	}

	public static float dotP(float x1, float y1, float x2, float y2) {
		return x1 * x2 + y1 * y2;
	}
	
}
