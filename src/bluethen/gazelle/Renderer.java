package bluethen.gazelle;

import java.util.ArrayList;
import java.util.List;

import org.newdawn.slick.Color;
import org.newdawn.slick.Graphics;
import org.newdawn.slick.geom.Shape;

import bluethen.gazelle.constraints.Ball;
import bluethen.gazelle.constraints.Circle;
import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.constraints.Link;
import bluethen.gazelle.constraints.Polygon;
import bluethen.gazelle.constraints.Rigid;

public class Renderer {
	static List<Float> x1 = new ArrayList<Float>();
	static List<Float> y1 = new ArrayList<Float>();
	static List<Float> x2 = new ArrayList<Float>();
	static List<Float> y2 = new ArrayList<Float>();

	public static void draw(ConstraintManager constraints,
			Graphics g) {
//		constraints.rigidBodyGrid.drawGrid(g);
		List<Rigid> rigidBodies = constraints.getRigidBodies();
		for (Rigid rb : rigidBodies) {
			if (rb instanceof Circle) {
				Circle circle = (Circle) rb;
				g.drawOval(circle.getBody().getX() - circle.getRadius(), circle.getBody().getY() - circle.getRadius(), circle.getRadius() * 2, circle.getRadius() * 2);
				if (rb instanceof Ball) {
					Ball ball = (Ball)rb;
					g.drawLine(ball.getBody().getX(), ball.getBody().getY(),
							ball.getBody().getX() + ball.getRadius() * (float)Math.cos(ball.getAngle()), 
							ball.getBody().getY() + ball.getRadius() * (float)Math.sin(ball.getAngle()));
					
				}
					
			}
		}

		// for ( circle : constraints.circleConstraints) {
		// g.drawOval(circle.getBody().getX()-circle.getRadius(),
		// circle.getBody().getY()-circle.getRadius(), circle.getRadius()*2,
		// circle.getRadius()*2);
		// }
//		for (Body body : bodyPool) {
//			// g.drawOval(body.getX()-2.5f, body.getY()-2.5f, 5,5);
//
//			// drawing each link twice!
//			// for (Link link : body.getLinks())
//			// g.drawLine(link.getBody1().getX(), link.getBody1().getY(),
//			// link.getBody2().getX(), link.getBody2().getY());
//		}
		
		for (Rigid rigid : constraints.rigidConstraints) {
			if (rigid instanceof Polygon) {
				Polygon poly = (Polygon) rigid;
				org.newdawn.slick.geom.Polygon shape = new org.newdawn.slick.geom.Polygon();
				for (Body b : poly.getVertices()) {
					shape.addPoint(b.getX(), b.getY());
				}
				shape.setClosed(true);
				g.setColor(new Color(0,0,0));
				g.fill(shape);
				g.setColor(new Color(255,255,255));
				g.draw(shape);
				
			}
		}
		
		for (Link link : constraints.getLinks()) {
			if (link.isVisible()) 
				g.drawLine(link.getBody1().getX(), link.getBody1().getY(), 
						link.getBody2().getX(), link.getBody2().getY());
		}
		for (Edge edge : constraints.getEdges()) {
			if (!edge.partOfPolygon() && edge.isVisible())
				g.drawLine(edge.getBody1().getX(), edge.getBody1().getY(), 
						edge.getBody2().getX(), edge.getBody2().getY());
		}
		
		drawLines(g);
//		clearLines();
	}

	public static void addLine(float aX, float aY, float bX, float bY) {
		if (x1 == null) {
			x1 = new ArrayList<Float>();
			y1 = new ArrayList<Float>();

			x2 = new ArrayList<Float>();
			y2 = new ArrayList<Float>();
		}
		x1.add(aX);
		y1.add(aY);

		x2.add(bX);
		y2.add(bY);
	}

	static void drawLines(Graphics g) {
		for (int i = 0; i < x1.size(); i++) {
			g.drawLine(x1.get(i), y1.get(i), x2.get(i), y2.get(i));
		}
//		clearLines();
	}

	static void clearLines() {
		x1.clear();
		y1.clear();

		x2.clear();
		y2.clear();
	}

}
