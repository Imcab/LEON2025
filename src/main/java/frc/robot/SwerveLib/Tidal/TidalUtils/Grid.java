package frc.robot.SwerveLib.Tidal.TidalUtils;

public class Grid {

    private Domain XaxisDomain;
    private Domain YaxisDomain;
    private Coordinate coordinate;
    private double tolerance;

    /**
     * Creates a grid, similar to the domain but is a 2 axis instead of one
     * @param x the value in X to check
     * @param y the value in Y to check
     * @param tolerance the tolerance to have (from 0-100) where 5% is plus-minus 0.05 in x and y (for Example)
     */
    public Grid(double x, double y, double tolerance){

        this.tolerance = tolerance;
        this.coordinate = new Coordinate(x, y);
        XaxisDomain = new Domain(x - (tolerance/100), x + (tolerance/100));
        YaxisDomain = new Domain(y - (tolerance/100), y + (tolerance/100));
    }

    /**
     * Creates a grid, similar to the domain but is a 2 axis instead of one
     * @param Coordinate the coordinate to check in X and Y axis
     * @param tolerance the tolerance to have (from 0-100) where 5% is plus-minus 0.05 in x and y (for Example)
     */
    public Grid(Coordinate coordinate, double tolerance){

        this.tolerance = tolerance;
        this.coordinate = coordinate;
        XaxisDomain = new Domain(coordinate.inX() - (tolerance/100), coordinate.inX() + (tolerance/100));
        YaxisDomain = new Domain(coordinate.inY() - (tolerance/100), coordinate.inY() + (tolerance/100));
    }

    public Coordinate asCoordinate(){
        return coordinate;
    }

    public double getTolerance(){
        return tolerance;
    }

    public Coordinate domain(){
        return new Coordinate(XaxisDomain.minValue(), XaxisDomain.maxValue());
    }
    
    public Coordinate range(){
        return new Coordinate(YaxisDomain.minValue(), YaxisDomain.maxValue());
    }

    public boolean atGrid(Coordinate coordinate){
        return XaxisDomain.inRange(coordinate.inX()) && YaxisDomain.inRange(coordinate.inY());
    }


}
