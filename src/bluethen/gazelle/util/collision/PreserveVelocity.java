package bluethen.gazelle.util.collision;

import bluethen.gazelle.Body;
import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.util.PMath;
/*
 * http://codeflow.org/entries/2010/nov/29/verlet-collision-with-impulse-preservation/ 
 * f1 and f2
 * are velocities projected on the collision normal
 * we subtract the velocity along the collision normal of one ball from the other ball
 * and vice versa
 */
public class PreserveVelocity {
	public static boolean enabled = true;
	public static void apply (Body a, Body b, 
			float aVX, float aVY, float bVX, float bVY, 
			float collisionNormalX, float collisionNormalY,
			float damping) {
		if (enabled) {
			// project that onto the collision normal to see how
			// much of it is towards point
			float f1 = damping * PMath.dotP(aVX, aVY, collisionNormalX, collisionNormalY);
			float f2 = damping * PMath.dotP(bVX, bVY, collisionNormalX, collisionNormalY);
	
			float newAVX = aVX + f2 * collisionNormalX - f1 * collisionNormalX;
			float newAVY = aVY + f2 * collisionNormalY - f1 * collisionNormalY;
	
			float newBVX = bVX + f1 * collisionNormalX - f2 * collisionNormalX;
			float newBVY = bVY + f1 * collisionNormalY - f2 * collisionNormalY;
	
			a.setLastX(a.getX() - newAVX);
			a.setLastY(a.getY() - newAVY);
	
			b.setLastX(b.getX() - newBVX);
			b.setLastY(b.getY() - newBVY);
		}
	}
	
	public static void apply (Body body, Edge edge, 
			float bodyVX, float bodyVY, 
			float edgeVXA, float edgeVYA, float edgeVXB, float edgeVYB, float t,
			float collisionNormalX, float collisionNormalY,
			float damping) {
		if (enabled) {
			// find edge's velocity at impact point
			float edgeVX = edgeVXA * (1 - t) + edgeVXB * t;
			float edgeVY = edgeVYA * (1 - t) + edgeVYB * t;
	
			// project that onto the collision normal to see how
			// much of it is towards point
			float f1 = PMath.dotP(collisionNormalX, collisionNormalY, bodyVX, bodyVY);
			float f2 = PMath.dotP(collisionNormalX, collisionNormalY, edgeVX, edgeVY);
	
			f1 *= damping;
			f2 *= damping;
			
			float newPointVX = bodyVX + f2 * collisionNormalX - f1 * collisionNormalX;
			float newPointVY = bodyVY + f2 * collisionNormalY - f1 * collisionNormalY;
	
			float edgeVXDiff = f1 * collisionNormalX - f2 * collisionNormalX;
			float edgeVYDiff = f1 * collisionNormalY - f2 * collisionNormalY;
	
			body.setLastX(body.getX() - newPointVX);
			body.setLastY(body.getY() - newPointVY);
	
			edge.getBody1().setLastX(edge.getBody1().getX() - (edgeVXA + edgeVXDiff * (1 - t)));
			edge.getBody1().setLastY(edge.getBody1().getY() - (edgeVYA + edgeVYDiff * (1 - t)));
			edge.getBody2().setLastX(edge.getBody2().getX() - (edgeVXB + edgeVXDiff * t));
			edge.getBody2().setLastY(edge.getBody2().getY() - (edgeVYB + edgeVYDiff * t));
		}
	}
}
