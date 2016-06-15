package funciones;

import EDU.gatech.cc.is.util.Vec2;

import java.util.Comparator;
import java.util.function.Function;
import java.util.function.ToDoubleFunction;
import java.util.function.ToIntFunction;
import java.util.function.ToLongFunction;

/**
 * Created by angel on 22/01/2016.
 */
public class Vec2YComparator implements Comparator<Vec2> {

    @Override
    public int compare(Vec2 o1, Vec2 o2) {
        Double y1 = o1.y;
        Double y2 = o2.y;
        return y1.compareTo(y2);
    }
}
