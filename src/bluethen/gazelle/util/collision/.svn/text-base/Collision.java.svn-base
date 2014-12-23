package bluethen.gazelle.util.collision;

import bluethen.gazelle.Body;
import bluethen.gazelle.util.PMath;
import bluethen.gazelle.Renderer;
import bluethen.gazelle.constraints.Circle;
import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.constraints.Polygon;

/*
 * Utility class that contains methods for checking for collisions between different shapes
 */
public class Collision {
	/*
	 * Checks to see if two line segments overlap if they do, returns a float
	 * array of where they overlap in the form of new float[]{x,y,t} (t is
	 * scalar between 0 and 1, showing how far along the line the collision is)
	 * 
	 * Returns null if none is found
	 * 
	 * based off the implementation @
	 * http://wiki.processing.org/w/Line-Line_intersection
	 */
	public static float[] checkSegmentToSegment(float x1, float y1, float x2, float y2, // first line
			float x3, float y3, float x4, float y4) { // second line

		float bx = x2 - x1;
		float by = y2 - y1;
		float dx = x4 - x3;
		float dy = y4 - y3;
		float b_dot_d_perp = bx * dy - by * dx;
		if (b_dot_d_perp == 0)
			return null;
		float cx = x3 - x1;
		float cy = y3 - y1;
		float t = (cx * dy - cy * dx) / b_dot_d_perp;
		if (t < 0 || t > 1)
			return null;
		float u = (cx * by - cy * bx) / b_dot_d_perp;
		if (u < 0 || u > 1)
			return null;
		return new float[] { x1 + t * bx, y1 + t * by, t };
	}

	public static boolean checkPolygonToPolygonAABB(Polygon a, Polygon b) {
		float aR = a.getBoundingRadius();
		float bR = b.getBoundingRadius();
		
		float rSum = aR + bR;
		
		if (Math.abs(a.getX() - b.getX()) > rSum)
			return false;
		if (Math.abs(a.getY() - b.getY()) > rSum)
			return false;

		return true;
	}

	/*
	 * Tries to find the min/max points from the axis Returns a new min and max
	 * in the form of new float[]{min,max}
	 */
	public static float[] projectToAxis(Polygon b, float axisX, float axisY) {
		float min = 0, max = 0;
		float axisLenSq = axisX * axisX + axisY * axisY;

		// dot Product of first vertex for reference
		float dotP = PMath.dotP(b.getVertex(0).getX(), b.getVertex(0).getY(), axisX, axisY) / axisLenSq;

		min = dotP;
		max = dotP;

		for (int i = 0; i < b.getVertexCount(); i++) {
			// Project the rest of the vertices onto the axis and extend
			// the interval to the left/right if necessary
			dotP = PMath.dotP(b.getVertex(i).getX(), b.getVertex(i).getY(), axisX, axisY) / axisLenSq;

			min = Math.min(dotP, min);
			max = Math.max(dotP, max);
		}

		return new float[] { min, max };
	}

	public static float intervalDistance(float minA, float maxA, float minB, float maxB) {
		if (minA < minB)
			return minB - maxA;
		return minA - maxB;
	}
}
