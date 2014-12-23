package bluethen.gazelle.util.collision;

import bluethen.gazelle.Body;
import bluethen.gazelle.constraints.Circle;
import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.constraints.Polygon;

public class PolygonToCircle {
	public static boolean resolve(Polygon poly, Circle circle, boolean preserveVelocity) {
		float damping = 0.95f;
//		float slidingFriction = 0.3f;
		
		float oldCircleVX = 0, oldCircleVY = 0;
		if (preserveVelocity) {
			oldCircleVX = circle.getBody().getVelX();
			oldCircleVY = circle.getBody().getVelY();
		}
		
		// find point on Circle closest to polygon
		// circleX and Y is the point on circle closest to polygon
		// circleX2 and Y2 is the point on circle farthest from polygon
		float circleX = poly.getX() - circle.getX();
		float circleY = poly.getY() - circle.getY();
		float dist = (float)Math.sqrt(circleX * circleX + circleY * circleY);
		circleX /= dist;
		circleY /= dist;
		
		circleX *= circle.getRadius();
		circleY *= circle.getRadius();
		
		float circleX2 = circle.getX() - circleX;
	    float circleY2 = circle.getY() - circleY;
			
		circleX += circle.getX();
		circleY += circle.getY();
		
		float minDist = Float.MAX_VALUE;
		boolean collision = false;
		float collisionNormalX = 0, collisionNormalY = 0;
		Edge collisionEdge = null;
		for (int i = 0; i < poly.getEdgeCount(); i++) {
			Edge edge = poly.getEdge(i);

			// Find the axis perpendicular to the edge
			float sepAxisX = edge.getBody1().getY() - edge.getBody2().getY(); 
			float sepAxisY = edge.getBody2().getX() - edge.getBody1().getX();
					
			// Normalize sepAxis
			float sepAxisDist = (float) Math.sqrt(sepAxisX * sepAxisX + sepAxisY * sepAxisY);
			sepAxisX /= sepAxisDist;
			sepAxisY /= sepAxisDist;
			
			// project circle onto sepAxis
			float cDotSep = sepAxisX * circleX + sepAxisY * circleY;
			float cDotSep2 = sepAxisX * circleX2 + sepAxisY * circleY2;
			
			// project polygon onto sepAxis
			float [] pDotSep = Collision.projectToAxis(poly, sepAxisX, sepAxisY);
			
			float minA = Math.min(cDotSep, cDotSep2);
			float maxA = Math.max(cDotSep, cDotSep2);
			float minB = pDotSep[0];
			float maxB = pDotSep[1];
			
			float distance = Collision.intervalDistance(minA, maxA, minB, maxB);
			
			if (distance > 0) {
				collision = false;
				break; // no collision
			}
			else if (Math.abs(distance) < minDist) {
				minDist = Math.abs(distance);
				collision = true;
				collisionNormalX = sepAxisX;
				collisionNormalY = sepAxisY;
				collisionEdge = edge;
			}
		}
		if (collision && collisionEdge != null) {
			// Make sure the collision normal is pointed towards circle
			// Use the "line formula" (find the dot product of normal and
			// (centerA-centerB))
			float line = collisionNormalX * (circle.getX() - poly.getX()) + collisionNormalY * (circle.getY() - poly.getY());
			if (line < 0) {
				collisionNormalX = -collisionNormalX;
				collisionNormalY = -collisionNormalY;
			}
			
			float oldEdgeV1X = 0, oldEdgeV1Y = 0, oldEdgeV2X = 0, oldEdgeV2Y = 0;
			if (preserveVelocity) {
				oldEdgeV1X = collisionEdge.getBody1().getVelX();
				oldEdgeV1Y = collisionEdge.getBody1().getVelY();

				oldEdgeV2X = collisionEdge.getBody2().getVelX();
				oldEdgeV2Y = collisionEdge.getBody2().getVelY();
			}

			
			float collisionX = collisionNormalX * minDist;
			float collisionY = collisionNormalY * minDist;
			
			float t;
			if (Math.abs(collisionEdge.getBody2().getX() - collisionEdge.getBody1().getX()) > Math.abs(collisionEdge.getBody2().getY() - collisionEdge.getBody1().getY()))
				t = (circle.getX() - collisionX - collisionEdge.getBody1().getX()) / (collisionEdge.getBody2().getX() - collisionEdge.getBody1().getX());
			else
				t = (circle.getY() - collisionY - collisionEdge.getBody1().getY()) / (collisionEdge.getBody2().getY() - collisionEdge.getBody1().getY());

			if (Float.isInfinite(t)) {
				t = -1;
				t = (circle.getY() - collisionY - collisionEdge.getBody1().getY());
			}
			float lambda = 1f / (t * t + (1 - t) * (1 - t));
		
			float inverseMass1 = 1f / circle.getBody().getMass();
			float inverseMass2 = 1f / poly.getMass();
			float correction1 = (inverseMass1 / (inverseMass1 + inverseMass2));
			float correction2 = 1-correction1;
			
			// correct edge's position
			collisionEdge.getBody1().setX(collisionEdge.getBody1().getX() - collisionX * (1 - t) * correction2 * lambda);
			collisionEdge.getBody1().setY(collisionEdge.getBody1().getY() - collisionY * (1 - t) * correction2 * lambda);

			collisionEdge.getBody2().setX(collisionEdge.getBody2().getX() - collisionX * t * correction2 * lambda);
			collisionEdge.getBody2().setY(collisionEdge.getBody2().getY() - collisionY * t * correction2 * lambda);
		
			circle.getBody().setX(circle.getX() + collisionX * correction1);
			circle.getBody().setY(circle.getY() + collisionY * correction1);

			// Preserve velocity
			if (preserveVelocity) {
				PreserveVelocity.apply(circle.getBody(), collisionEdge, 
						oldCircleVX, oldCircleVY, 
						oldEdgeV1X, oldEdgeV1Y, oldEdgeV2X, oldEdgeV2Y, t,
						collisionNormalX, collisionNormalY,
						damping);
			}
			
			// Apply sliding friction
//			SlidingFriction.apply(circle.getBody(), collisionNormalX, collisionNormalY, slidingFriction);
//			SlidingFriction.apply(collisionEdge, -collisionNormalX, -collisionNormalY, slidingFriction);
		}
		// see if any of poly's points are inside of circle
		for (int i = 0; i < poly.getVertexCount(); i++) {
			Body point = poly.getVertex(i);
			float diffX = circle.getX() - point.getX();
			float diffY = circle.getY() - point.getY();
			float diffSquared = (float) (Math.pow(diffX, 2) + Math.pow(diffY, 2));
			
			if (diffSquared < circle.getRadius() * circle.getRadius()) {
				float oldPointVX = 0, oldPointVY = 0;
				if (preserveVelocity) {
					oldPointVX = point.getVelX();
					oldPointVY = point.getVelY();
				}
				
			    dist = (float)Math.sqrt(diffSquared);
				float diff = (circle.getRadius() - dist) / dist;
				
				float inverseMass1 = 1f / circle.getBody().getMass();
				float inverseMass2 = 1f / poly.getMass();
				float correction1 = (inverseMass1 / (inverseMass1 + inverseMass2));
				float correction2 = 1-correction1;
				
				circle.setX(circle.getX() + diffX * diff * correction1);
				circle.setY(circle.getY() + diffY * diff * correction1);
				
				point.setX(point.getX() - diffX * diff * correction2);
				point.setY(point.getY() - diffY * diff * correction2);
				
				float normX = diffX / dist;
				float normY = diffY / dist;
				
				if (preserveVelocity) {
					PreserveVelocity.apply(circle.getBody(), point, 
							oldCircleVX, oldCircleVY, oldPointVX, oldPointVY, 
							normX, normY, damping);
				}
				
//				SlidingFriction.apply(circle.getBody(), normX, normY, slidingFriction);
//				SlidingFriction.apply(point, -normX, -normY, slidingFriction);
			}
		}
		return collision;
	}
}
