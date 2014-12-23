package bluethen.gazelle;

import java.util.List;
import java.util.ArrayList;

import org.newdawn.slick.Graphics;

import bluethen.gazelle.constraints.Rigid;
import bluethen.gazelle.util.PMath;

/*
 * Grid
 * Organizes RigidBodies into cells for fast lookup
 */

public class Grid {
	private List<Rigid>[] grid;

	int cellColumns;

	int cellRows;
	int cellSize;

	int width;
	int height;

	public Grid(int width, int height) {
		resetGrid(width, height, 10);
	}

	private void resetGrid(int width, int height, int cellSize) {
		this.width = width;
		this.height = height;
		this.cellSize = cellSize;
		cellColumns = (int) (Math.ceil(width / (float) cellSize)) + 1;
		cellRows = (int) (Math.ceil(height / (float) cellSize)) + 1;

		grid = new ArrayList[cellColumns * cellRows];

		for (int i = 0; i < grid.length; i++)
			grid[i] = new ArrayList<Rigid>();
	}

	public void drawGrid(Graphics g) {
		for (int x = 0; x < cellColumns; x++) {
			for (int y = 0; y < cellRows; y++) {
				g.drawRect(x * cellSize, y * cellSize, cellSize, cellSize);
			}
		}
	}

	public List<Rigid> getNearbyBodies(float x, float y) {
		int cellCol = boundX((int) x / cellSize);
		int cellRow = boundY((int) y / cellSize);

//		Renderer.addLine(cellCol * cellSize, cellRow * cellSize, x,y);
		
		List<Rigid> nearby = new ArrayList<Rigid>();

		// Check cells on left
		if (cellCol > 0) {
			if (cellRow > 0)
				for (Rigid rb : grid[(cellRow - 1) * cellColumns + (cellCol - 1)]) // top left
					nearby.add(rb);
			for (Rigid rb : grid[cellRow * cellColumns + (cellCol - 1)]) // middle left
				nearby.add(rb);
			if (cellRow < cellRows)
				for (Rigid rb : grid[(cellRow + 1) * cellColumns + (cellCol - 1)]) // bottom left
					nearby.add(rb);
		}

		// check cells down middle column
		if (cellRow > 0)
			for (Rigid rb : grid[(cellRow - 1) * cellColumns + cellCol]) // top middle
				nearby.add(rb);
		for (Rigid rb : grid[cellRow * cellColumns + cellCol]) // middle
			nearby.add(rb);
		if (cellRow < cellRows)
			for (Rigid rb : grid[cellRow + 1 * cellColumns + cellCol]) // bottom middle
				nearby.add(rb);

		// check cells down right column
		// not sure why we have to do -1 here... the last check sometimes breaks
		// without it
		if (cellCol < cellColumns-1) {
			if (cellRow > 0)
				for (Rigid rb : grid[(cellRow - 1) * cellColumns + (cellCol + 1)]) // top right
					nearby.add(rb);
			for (Rigid rb : grid[cellRow * cellColumns + (cellCol + 1)]) // middle right
				nearby.add(rb);
			if (cellRow < cellRows)
				for (Rigid rb : grid[(cellRow + 1) * cellColumns + (cellCol + 1)]) // bottom right
					nearby.add(rb);
		}
		return nearby;
	}

	public void check(float radius) {
		if ((int) (radius * 2) > cellSize)
			resetGrid(width, height, (int) (radius * 2));
	}

	// y * width + x
	public void populate(List<Rigid> rigidBodies) {
		// for (Rigid rb : rigidBodies)
		// if ((int)(rb.getBoundingRadius()*2) > cellSize) {
		// resetGrid(width,height,(int)(rb.getBoundingRadius()*2));
		// }
		for (Rigid rb : rigidBodies)
			updatePosition(rb);
	}

	public void updatePosition(Rigid rb) {
		int newCell = worldToGridIndex(rb.getX(), rb.getY());
		if (!rb.isInGrid()) {
			addToCell(rb, newCell);
		} else if (rb.getCell() != newCell) {
			removeFromCell(rb, rb.getCell());
			addToCell(rb, newCell);
		}
//		Renderer.addLine(rb.getX(), rb.getY(), cellSize * boundX((int) rb.getX() / cellSize), 
//				cellSize * boundY((int) rb.getY() / cellSize));
	}

	public void addToCell(Rigid rb, int cell) {
		grid[cell].add(rb);
		rb.setCell(cell);
	}

	public void removeFromCell(Rigid rb, int cell) {
		if (grid[cell].contains(rb)) {
			grid[cell].remove(rb);
		}
	}

	public int worldToGridIndex(float x, float y) {
		return PMath.constraint(boundY((int) y / cellSize) * cellColumns + boundX((int) x / cellSize), 0, grid.length - 1);
	}

	public int boundX(int x) {
		return PMath.constraint(x, 0, cellColumns);
	}

	public int boundY(int y) {
		return PMath.constraint(y, 0, cellRows - 2);
	}

	public int getBodyCount() {
		int count = 0;
		for (List cell : grid) {
			count += cell.size();
		}
		return count;
	}
}
