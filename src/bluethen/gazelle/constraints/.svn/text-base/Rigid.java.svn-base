package bluethen.gazelle.constraints;

import bluethen.gazelle.Body;
import bluethen.gazelle.Grid;
import bluethen.gazelle.PhysicsMediator;

public interface Rigid extends Constraint {
	public float getX();
	public float getY();

	public int getCell();
	public void setCell(int cell);
	public boolean isInGrid();

	public void calculateBoundingRadius();
	public float getBoundingRadius();

	public void solveConstraint(Grid grid, boolean preserveVelocity, boolean soft);

	// get a list of bodies anchored to this shape
	public Body[] getBodies();

	public void correctBody();

	public boolean contains(float x, float y);
	
	public void destroySelf(PhysicsMediator physics);
	
	public Rigid clone(PhysicsMediator physics);
}
