package bluethen.gazelle.constraints;

import bluethen.gazelle.Body;
import bluethen.gazelle.PhysicsMediator;

public class Link implements Constraint {
	protected Body body1;
	protected Body body2;

	float restingDistance;
	float stiffness;
	
	boolean visible = true;

	public Link(Body body1, Body body2, float restingDistance) {
		this(body1, body2, restingDistance, 1f);
	}

	public Link(Body body1, Body body2, float restingDistance, float stiffness) {
		this.body1 = body1;
		this.body2 = body2;
		this.restingDistance = restingDistance;
		this.stiffness = stiffness;

		body1.addLink(this);
		body2.addLink(this);
	}

	public void solveConstraint() {
		if (!body1.isLocked() || !body2.isLocked()) {
			// Velocity preservation does nothing in this constraint

			// Find x and y difference of both bodies
			float deltaX = body1.getX() - body2.getX();
			float deltaY = body1.getY() - body2.getY();

			// Get distance
			float distance = (float) Math.sqrt(deltaX * deltaX + deltaY
					* deltaY);

			// Find the difference and normalize it
			float difference = (restingDistance - distance) / distance;

			// How much do we move each body
			float correction1 = 0, correction2 = 0;
			if (!body1.isLocked() && !body2.isLocked()) {
				// Compute how much to move each body based on mass
				float inverseMass1 = 1f / body1.getMass();
				float inverseMass2 = 1f / body2.getMass();

				// Correction is the fraction of the translation distance each
				// point should get
				// It depends on their mass differences and the stiffness of the
				// link
				correction1 = (inverseMass1 / (inverseMass1 + inverseMass2))
						* stiffness;
				correction2 = (inverseMass2 / (inverseMass1 + inverseMass2)) * stiffness;
			} else if (body1.isLocked() && !body2.isLocked()) {
				correction1 = 0;
				correction2 = 1; // Move body2 100% of the way
			} else if (!body1.isLocked() && body2.isLocked()) {
				correction1 = 1; // Move body1 100% of the way
				correction2 = 0;
			}
			// Correct the body positions
			body1.setX(body1.getX() + deltaX * correction1 * difference);
			body1.setY(body1.getY() + deltaY * correction1 * difference);

			body2.setX(body2.getX() - deltaX * correction2 * difference);
			body2.setY(body2.getY() - deltaY * correction2 * difference);
		}
	}

	public Body getBody1() {
		return body1;
	}

	public Body getBody2() {
		return body2;
	}
	
//	public Link clone (PhysicsMediator physics) {
//		// clone body1 and body2
//		Body bodyClone1 = body1.clone(physics);
//		Body bodyClone2 = body2.clone(physics);
//		
//		Link clone = new Link(bodyClone1, bodyClone2, restingDistance);
//		clone.stiffness = stiffness;
//		
//		physics.addConstraint(clone);
//		
//		return clone;
//	}
	
	public float getStiffness () {
		return stiffness;
	}
	public void setStiffness (float stiff) {
		stiffness = stiff;
	}
	public float getRestingDistance() {
		return restingDistance;
	}
	public void setVisible (boolean vis) {
		visible = vis;
	}
	public boolean isVisible () {
		return visible;
	}
}
