package bluethen.gazelle.constraints;

import java.util.List;

import bluethen.gazelle.Body;
import bluethen.gazelle.PhysicsMediator;
import bluethen.gazelle.util.collision.Collision;

public class Edge extends Link {
	// partOfRigidBody tells whether or not this edge is part of a larger body
	// if it is, then we do not solve rigid-constraints,
	// and instead assume that it will be taken care of in RigidBody
	boolean partOfRigidBody;

	public Edge(Body body1, Body body2, float restingDistance) {
		super(body1, body2, restingDistance, 1);
		body1.addEdge(this);
		body2.addEdge(this);
		
		body1.removeLink(this); // because super() will try and add this to the bodies' link lists
		body2.removeLink(this);
		
		partOfRigidBody = false;
	}

	public void solveConstraint(List<Body> bodies, boolean preserveVelocity) {
		// Only solve body-edge constraints if the edge is not a part of rigid
		// body
		if (!partOfRigidBody) {
			// Loop through all other points
			// if they pass through this edge, then we correct them
			for (Body body : bodies) {
				if (body != body1 && body != body2) {
					// Check for a collision between this edge's two points
					// and the bodie's current and last position
					float[] collision = Collision.checkSegmentToSegment(
							body1.getX(), body1.getY(), body2.getX(),
							body2.getY(), body.getX(), body.getY(),
							body.getLastX(), body.getLastY());
					// There's a collision
					if (collision != null) {
						// Find the axis perpendicular to the edge (its normal)
						float axisX = body1.getY() - body2.getY();
						float axisY = body2.getX() - body1.getX();
						// normalize the axis
						// instead of finding its length, we can just use the
						// restingDistance
						axisX /= restingDistance;
						axisY /= restingDistance;

						// Make sure the collision normal is pointed towards
						// Body a's last position
						// Use the "line formula" (find the dot product of
						// normal and (centerA-centerB))
						float line = axisX * (body.getLastX() - body.getX())
								+ axisY * (body.getLastY() - body.getY());
						if (line < 0) {
							axisX = -axisX;
							axisY = -axisY;
						}

						// Find the distance between body and the line
						float deltaX = body.getX() - collision[0];
						float deltaY = body.getY() - collision[1];
						float length = (float) Math.sqrt(deltaX * deltaX
								+ deltaY * deltaY);

						// Get the collisionVector
						float collisionVectorX = length * axisX;
						float collisionVectorY = length * axisY;

						// If preserveVelocity is on, we find body's
						// before-collision velocity
						float vx = 0, vy = 0;
						;
						if (preserveVelocity) {
							vx = body.getX() - body.getLastX();
							vy = body.getY() - body.getLastY();
						}

						// Move body half way back the collisionVector
						body.setX(body.getX() + collisionVectorX * 0.5f);
						body.setY(body.getY() + collisionVectorY * 0.5f);

						if (preserveVelocity) {
							body.setLastX(body.getX() - vx);
							body.setLastY(body.getY() - vy);
						}

						float t = 0;
						if (Math.abs(body1.getX() - body2.getX()) > Math
								.abs(body1.getY() - body2.getY())) {
							t = (body.getX() - collisionVectorX - body1.getX())
									/ (body2.getX() - body1.getX());
						} else {
							t = (body.getX() - collisionVectorX - body1.getX())
									/ (body2.getY() - body1.getY());
						}

						// Find the scaling factor
						float lambda = 1f / (t * t + (1 - t)
								* (1 - collision[2]));

						float v1x = 0, v1y = 0, v2x = 0, v2y = 0;
						if (preserveVelocity) {
							v1x = body1.getX() - body1.getLastX();
							v1y = body1.getY() - body1.getLastY();
							v2x = body2.getX() - body2.getLastX();
							v2y = body2.getY() - body2.getLastY();
						}
						// Push the edge's bodies
						body1.setX(body1.getX() - collisionVectorX * (1 - t)
								* 0.5f * lambda);
						body1.setY(body1.getY() - collisionVectorY * (1 - t)
								* 0.5f * lambda);

						body2.setX(body2.getX() - collisionVectorX * t * 0.5f
								* lambda);
						body2.setY(body2.getY() - collisionVectorY * t * 0.5f
								* lambda);
						if (preserveVelocity) {
							body1.setLastX(body1.getX() - v1x);
							body1.setLastY(body1.getY() - v1y);
							body2.setLastX(body2.getX() - v2x);
							body2.setLastY(body2.getY() - v2y);
						}
					}
				}
			}
		}
		// solve distance constraints inherited from Link
		super.solveConstraint();
	}

	public void setPartOfPolygon(boolean partOfRigidBody) {
		this.partOfRigidBody = partOfRigidBody;
	}

	public boolean partOfPolygon () {
		return partOfRigidBody;
	}
	
//	public Edge clone (PhysicsMediator physics) {
//		// clone body1 and body2
//		Body bodyClone1 = body1.clone(physics);
//		Body bodyClone2 = body2.clone(physics);
//		
//		Edge clone = new Edge(bodyClone1, bodyClone2, restingDistance);
//		//clone.partOfRigidBody = partOfRigidBody;
//		clone.stiffness = stiffness;
//		
//		physics.addConstraint(clone);
//		
//		return clone;
//	}
}
