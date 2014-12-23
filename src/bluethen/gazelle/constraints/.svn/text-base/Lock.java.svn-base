package bluethen.gazelle.constraints;

import bluethen.gazelle.Body;


/*
 * Locks 2 polygons together so they're at a specific translation apart
 * anything that happens to one polygon happens to the other as well
 */
public class Lock implements Constraint {
	protected Body body1;
	protected Body body2;
	
	// offset for x and y from poly1 to poly2
	float xOff;
	float yOff;
	

	public Lock (Body body1, Body body2, float xOff, float yOff) {
		body1.addLock(this);
		body2.addLock(this);
		
		this.body1 = body1;
		this.body2 = body2;
		this.xOff = xOff;
		this.yOff = yOff;
	}
	
	public void solveConstraint () {
		// xOff and yOff offsets should be enforced outside of Lock class
		// however, we'll apply offsets here as well, just in case
		// assume body1 is the "parent" body
		solveConstraints(body1);
	}
	public void solveConstraints (Body parent) {
		if (body1 == parent) {
			body2.setX(body1.getX() + xOff);
			body2.setY(body1.getY() + yOff);
		}
		else if (body2 == parent) {
			body1.setX(body2.getX() - xOff);
			body1.setY(body2.getY() - yOff);
		}
	}
}
