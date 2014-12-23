package bluethen.gazelle.util.collision;

import bluethen.gazelle.Body;
import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.constraints.Polygon;

/* Collision Info
 * Used for passing information about collisions between 2 bodies.
 */
public class CollisionInfo {
	public CollisionInfo() {}

	public boolean intersects = false;

	public float depth;
	public float normalX;
	public float normalY;

	public Polygon edgeParent;
	public Edge edge;

	public Polygon pointParent;
	public Body point;
}