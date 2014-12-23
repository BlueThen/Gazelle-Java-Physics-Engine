package bluethen.gazelle.constraints;

import java.util.List;

import bluethen.gazelle.Body;
import bluethen.gazelle.Grid;
import bluethen.gazelle.util.collision.CircleToCircle;
import bluethen.gazelle.util.collision.PolygonToCircle;

public class Ball extends Circle implements Constraint, Rigid {
	float angle = 0;
	float lastAngle = 0;
	public Ball(Body body, float radius) {
		super(body, radius);
	}
	public Ball(Body body) {
		super(body);
	}
	public Ball(Body body, float radius, float angle) {
		this(body,radius);
		this.angle = angle;
		lastAngle = angle;
	}
	public void solveConstraint(Grid grid, boolean preserveVelocity) {
		grid.updatePosition(this);
		List<Rigid> rigidBodies = grid.getNearbyBodies(getX(), getY());

		for (Rigid otherRigid : rigidBodies) {
//			Renderer.addLine(otherRigid.getX(), otherRigid.getY(), getX(), getY());
			if (otherRigid instanceof Polygon) {
				Polygon otherPoly = (Polygon) otherRigid;
				PolygonToCircle.resolve(otherPoly, this, preserveVelocity);
				grid.updatePosition(this);
				grid.updatePosition(otherPoly);
			}
			if (otherRigid instanceof Circle) {
				CircleToCircle.resolve(this, (Circle)otherRigid, damping, preserveVelocity);
			}
		}
	}
	public void setAngle (float angle) {
		this.angle = angle;
	}
	public void setLastAngle (float lastAngle) {
		this.lastAngle = lastAngle;
	}
	public float getAngle () {
		return angle;
	}
	public float getAngleVel() {
		return angle - lastAngle;
	}
	public float getLastAngle () {
		return lastAngle;
	}	
}
