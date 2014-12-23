package bluethen.gazelle.constraints;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.newdawn.slick.geom.Ellipse;

import bluethen.gazelle.Body;
import bluethen.gazelle.Grid;
import bluethen.gazelle.PhysicsMediator;
import bluethen.gazelle.Renderer;
import bluethen.gazelle.util.collision.CircleToCircle;
import bluethen.gazelle.util.collision.Collision;
import bluethen.gazelle.util.collision.PolygonToCircle;
import bluethen.gazelle.util.collision.PreserveVelocity;

/*
 * Circle constraint
 * Kept at a minimum distance from all other boundaries, circles, and bodies
 */
public class Circle implements Constraint, Rigid {
	Body body;
	float radius;
	float damping = 0.95f;

	int cellLocation;
	boolean inGrid = false;

	public Circle(Body body) {
		this(body, 10);
	}

	public Circle(Body body, float radius) {
		this.body = body;
		this.radius = radius;

//		body.setShape(new Ellipse(0, 0, radius, radius));
	}

	public void solveConstraint(Grid grid, boolean preserveVelocity, boolean soft) {
		grid.updatePosition(this);
		List<Rigid> rigidBodies = grid.getNearbyBodies(getX(), getY());

		for (Rigid otherRigid : rigidBodies) {
//			Renderer.addLine(otherRigid.getX(), otherRigid.getY(), getX(), getY());
			boolean collided = false;
			if (otherRigid instanceof Polygon) {
				Polygon otherPoly = (Polygon) otherRigid;
				collided = PolygonToCircle.resolve(otherPoly, this, preserveVelocity);
				grid.updatePosition(this);
				grid.updatePosition(otherPoly);
			}
			if (otherRigid instanceof Circle) {
				collided = CircleToCircle.resolve(this, (Circle)otherRigid, damping, preserveVelocity);
			}
			if (collided) {
				for (Body b : otherRigid.getBodies())
					for (Lock l : b.getLocks())
						l.solveConstraints(b);
				for (Body b : this.getBodies())
					for (Lock l : b.getLocks())
						l.solveConstraints(b);
			}
		}
	}

	public float getX() {
		return body.getX();
	}

	public float getY() {
		return body.getY();
	}
	public void setX (float x) {
		body.setX(x);
	}
	public void setY (float y) {
		body.setY(y);
	}
	public Body getBody() {
		return body;
	}

	// Unneeded for circle
	public void calculateBoundingRadius() {
	}

	public float getBoundingRadius() {
		return getRadius();
	}

	public float getRadius() {
		return radius;
	}

	public boolean contains(float x, float y) {
		float deltaX = x - body.getX();
		float deltaY = y - body.getY();
		return deltaX * deltaX + deltaY * deltaY < radius * radius;
	}

	public boolean isInGrid() {
		return inGrid;
	}

	public void setCell(int cell) {
		inGrid = true;
		cellLocation = cell;
	}

	public int getCell() {
		return cellLocation;
	}

	@Override
	public Body[] getBodies() {
		return new Body[] { body };
	}

	@Override
	// does nothing as it is unneeded
	public void correctBody() {
	}
	
	public void destroySelf (PhysicsMediator physics) {
		// destroy body
		physics.removeBody(body);
		// destroy self
		physics.removeConstraint(this);
	}
	
	// Creates a copy of all of its parts
	// bodies included
	public Circle clone (PhysicsMediator physics) {
		Body bClone = body.clone(physics);
		
		Circle clone = new Circle(bClone);
		
		// copy primitives
		//clone.slidingFriction = slidingFriction;
		clone.damping = damping;
		clone.radius = radius;
		//clone.boundingRadius = boundingRadius;
	
		physics.addConstraint(clone);
		
		return clone;
	}
}
