package bluethen.gazelle.util.collision;

import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.constraints.Polygon;
import bluethen.gazelle.util.PMath;

public class PolygonToPolygon {
	/*
	 * Check to see if 2 Rigid Bodies overlaps 
	 * Returns a new CollisionInfo object. 
	 * intersects boolean should return true/false on whether or
	 * not they intersect
	 */
	public static CollisionInfo check (Polygon a, Polygon b) {
		CollisionInfo collision = new CollisionInfo();
		collision.intersects = false;
		
//		// this slows things down a lot :( 
//		// first do AABB checking 
//		if (!Collision.checkPolygonToPolygonAABB(a,b))
//			return collision; 
		 
		float minDist = Float.MAX_VALUE;

		// loop through each edge of both shapes
		for (int i = 0; i < a.getEdgeCount() + b.getEdgeCount(); i++) {

			Edge edge;

			Polygon edgeBody;
			Polygon pointBody;

			if (i < a.getEdgeCount()) { // Use aShape
				edgeBody = a;
				pointBody = b;
				edge = edgeBody.getEdge(i);
			} else { // use bShape
				edgeBody = b;
				pointBody = a;
				edge = edgeBody.getEdge(i - a.getEdgeCount());
			}

			// Find the axis the edge is aligned to
			// This axis is perpendicular to the edge
			float sepAxisX = edge.getBody1().getY() - edge.getBody2().getY(); 
			float sepAxisY = edge.getBody2().getX() - edge.getBody1().getX(); 

			// Normalize sepAxis
			//float sepAxisDistSq = sepAxisX * sepAxisX + sepAxisY * sepAxisY;
			float sepAxisDist = (float) Math.sqrt(sepAxisX * sepAxisX + sepAxisY * sepAxisY);
			sepAxisX /= sepAxisDist; // edge.getRestingDistance();
			sepAxisY /= sepAxisDist; // edge.getRestingDistance();

			// Project both shapes to axis
			float[] intervalsA = Collision.projectToAxis(a, sepAxisX, sepAxisY);
			float[] intervalsB = Collision.projectToAxis(b, sepAxisX, sepAxisY);
			float minA = intervalsA[0];
			float maxA = intervalsA[1];
			float minB = intervalsB[0];
			float maxB = intervalsB[1];

			float distance = Collision.intervalDistance(minA, maxA, minB, maxB);

			if (distance > 0)
				return collision; // No collision
			else if (Math.abs(distance) < minDist) {
//				float sepAxisDist = (float) Math.sqrt(sepAxisX * sepAxisX + sepAxisY * sepAxisY);
//				sepAxisX /= sepAxisDist; // edge.getRestingDistance();
//				sepAxisY /= sepAxisDist; // edge.getRestingDistance();
				
				minDist = Math.abs(distance);
				collision.normalX = sepAxisX;
				collision.normalY = sepAxisY;

				collision.edge = edge;
				collision.edgeParent = edgeBody;
				collision.pointParent = pointBody;
			}
		}
		if (collision.edgeParent == null)
			return collision; // no collision

		collision.depth = minDist;

		Polygon edgeParent = collision.edgeParent;
		Polygon pointParent = collision.pointParent;

		float edgeParentCenterX = edgeParent.getX();
		float edgeParentCenterY = edgeParent.getY();
		
		float pointParentCenterX = pointParent.getX();
		float pointParentCenterY = pointParent.getY();
		
		// Make sure the collision normal is pointed towards Body a
		// Use the "line formula" (find the dot product of normal and
		// (centerA-centerB))
		float line = PMath.dotP(collision.normalX, collision.normalY, 
				pointParentCenterX - edgeParentCenterX, pointParentCenterY - edgeParentCenterY);

		if (line < 0) {
			collision.normalX = -collision.normalX;
			collision.normalY = -collision.normalY;
		}

		// Find the vertex that intersects edge's body
		float smallestDist = Float.MAX_VALUE;
		for (int i = 0; i < pointParent.getVertexCount(); i++) {
			// Find the distance of the vertex of the line using the line
			// equation
			float distance = collision.normalX * (pointParent.getVertex(i).getX() - edgeParentCenterX) + 
				collision.normalY * (pointParent.getVertex(i).getY() - edgeParentCenterY);

			// If the distance is smaller than smallest distance so far,
			// set the smallest distance and collision vertex
			if (distance < smallestDist) {
				smallestDist = distance;
				collision.point = pointParent.getVertex(i);
			}
		}
		collision.intersects = true;
		return collision;
	}
}
