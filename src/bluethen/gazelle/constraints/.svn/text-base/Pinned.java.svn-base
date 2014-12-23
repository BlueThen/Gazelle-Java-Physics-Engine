package bluethen.gazelle.constraints;

import bluethen.gazelle.Body;

public class Pinned implements Constraint {
	Body body;

	float x;
	float y;

	public Pinned(Body body) {
		this(body, body.getX(), body.getY());
	}

	public Pinned(Body body, float x, float y) {
		this.body = body;
		this.x = x;
		this.y = y;
		body.setLocked(true);
	}

	public void solveConstraint() {
		body.setX(x);
		body.setY(y);
		body.setLastX(x);
		body.setLastY(y);
	}

	public void setX(float x) {
		this.x = x;
	}

	public void setY(float y) {
		this.y = y;
	}

}
