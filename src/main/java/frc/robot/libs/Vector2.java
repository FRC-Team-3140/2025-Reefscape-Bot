package frc.robot.libs;

public class Vector2 {
    public final double X;
    public final double Y;

    public Vector2(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }

    public Vector2() {
        this.X = 0;
        this.Y = 0;
    }

    public Vector2 add(Vector2 other){
        return new Vector2(X + other.X, Y + other.Y);
    }

    public Vector2 neg() {
        return new Vector2(-X, -Y);
    }

    public Vector2 sub(Vector2 other) {
        return new Vector2(X - other.X, Y - other.Y);
    }

    public Vector2 lerp(Vector2 other, double alpha) {
        return new Vector2((other.X - X)*alpha + X, (other.Y - Y)*alpha + Y);
    }
}
