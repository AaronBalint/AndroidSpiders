package au.com.netplaysoftware.androidspiders;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import java.util.LinkedList;

// general purpose Cartesian point class
class Point {
    float x, y;

    Point (float x, float y)
    {
        set (x, y);
    }

    void set (float x, float y)
    {
        this.x = x;
        this.y = y;
    }
}

class TurnAngle {
    float direction; // (anti)clockwise
    float angle; // radians
}

class Spiders {

    // AI user input states
    public static final int DOWN = 1;
    public static final int MOVE = 2;
    public static final int UP = 3;
    public static final int IDLE = 4;

    private final float octant = (float)(Math.PI/4.0); // one octant in radians
    private final int [] feetPos = new int[] {0, 4, 1, 5, 6, 2, 7, 3}; // leg movement order
    private final int [] anglePos = new int[] {0, 2, 5, 7, 1, 3, 4, 6}; // octant for corresponding leg

    // spider movement cycle
    private Point bodyStart; // start position
    private Point bodyEnd; // end position
    private Point bodyPosition; //current position

    // spider leg coordinates
    private Point[] foot = new Point[8]; // current foot position
    private Point[] footInc = new Point[8]; // foot incremental movement throughout current cycle
    private Point[] footPosition = new Point[8]; // foot start position in current cycle
    private Point[] joints = new Point[11]; // leg joint position

    private float walkStage = 0; // controls which legs are moving in the current cycle
    private float faceDirection; // direction in which spider is facing
    private float newFaceDirection; // where we are turning too
    private float jointDirection; // The leg angles lag face direction for realistic movement
    private float newJointDirection; // where we are turning too legwise
    private float walkDirection; // This can be different to face direction, we can walk backwards or sideways etc
    private float walkDistance; // how far we walk in a single AI response
    private float walkingStepSize; // largest step size
    private float stepSize; // our current distance per walk cycle (can be zero when stationary)
    private float turnrate; // how quickly we turn
    private float spiderSize; // Size of this spider
    private float [] legAngles; // Store for pre-calculated leg angles
    private boolean spiderStationary = false;

    private boolean[] missingLeg = new boolean[8]; // Is the given leg missing
    private int legsLost = 0;
    private int lastLegLostCount = 0; // used to determine if we have lost a leg in the last AI update
    private int pinnedLeg = -1; // if not -1, then the designated leg is pinned by the user
    private boolean beingDragged = false;
    private long beingDraggedTime = 0; // used to determine how long the spider has been dragged
    private int haveBrokenLeg = -1; // used to indicate which leg to use for broken animation
    private boolean [] twitchLeg = new boolean[8]; // lists which legs to twitch when squashed
    private long [] brokenLegTimeout = new long[8]; // used for fading of broken leg
    private float [] brokenLegAngle = new float[8]; // angle of broken leg

    private Point touchDownPoint = new Point (0,0); // user touch input
    private Point pinnedPoint = new Point (0,0); // user hold input

    // the following controls how quickly the spider moves, independent of walk distance
    private int targetFramesPerStep = 8; // normal walking space animation frames per step
    private int walkFramesPerStep = 8; // current frames per step
    private float incX, incY; // animation increments for a total walk cycle
    private float body_inc_x, body_inc_y; // animation increments

    private float alert_proximity; // spider alert AI distance
    private long retreat_timeout = 0; // used to pause spider after retreating

    private boolean spider_squished = false;
    private boolean checkedSquishState = false;
    private long squishedTime = 0;
    private float [] squishTX = {0,0,0,0,0};
    private float [] squishRadius = {0,0,0,0,0};
    private float dragDistance = 0;

    private TurnAngle turn = new TurnAngle(); // general purpose

    // constructor
    // x, y - spider position 
    // direction - initial face and walk direction (radians)
    // turn_rate - how quickly the spider rotates (octants per walk cycle)
    // step_size - how far each leg moves in a walk cycle. This is independent of walk distance
    // walk_distance - initial distance traversed in each walk cycle
    // size - overall size of spider
    // frames_per_step - number of animation frames per walk cycle
    public Spiders (int x, int y, float direction, float turn_rate, float step_size, float walk_distance, float spider_size, int frames_per_step) {

        bodyStart = new Point (x, y);
        bodyPosition = new Point (x, y);
        turnrate = turn_rate;
        direction = polarNormal(direction);
        faceDirection = direction;
        newFaceDirection = direction;
        jointDirection = direction;
        newJointDirection = direction;
        walkDirection = direction;
        walkDistance = walk_distance;
        stepSize = step_size;
        walkingStepSize = stepSize;
        spiderSize = spider_size;

        if (frames_per_step < 6) frames_per_step = 6;
        targetFramesPerStep = frames_per_step;
        walkFramesPerStep = frames_per_step;

        alert_proximity = spiderSize * 3;

        for (int i=0; i<8; i++) {
            footInc[i] = new Point (0,0);
            missingLeg[i] = false;
            twitchLeg[i] = false;
        }
        for (int i=0; i<11; i++) {
            joints[i] = new Point(0, 0);
        }

        setWalkDirection(walkDirection);
        set_positions (x, y);
        update_feet (x, y);
        update_joints ();

        initialize_draw();
    }

    // makes sure the polar angle is 0 <= angle <= 2*PI
    float polarNormal (float angle) {
        while (angle < 0) angle += 2*Math.PI;
        while (angle > 2*Math.PI) angle -= 2*Math.PI;
        return angle;
    }

    // Sets walking speed (distance per walk cycle)
    void setWalkDistance (float distance) {
        walkDistance = distance;
        stepSize = walkingStepSize;
    }

    // Sets the direction in which the spider is facing
    void setFaceDirection (float direction) {

        float a = 0;
        setLegAngles(0);
        int cnt = 0;
        for (int i = 0; i < 8; i++) {
            int p = feetPos[i];
            if (missingLeg[p]) continue;
            a = a + legAngles[i];
            cnt++;
        }
        if (cnt > 0) a = a / cnt;
        newFaceDirection = polarNormal(direction - a);
    }

    void setFramesPerStep (int framesPerStep) {
        walkFramesPerStep = framesPerStep;
    }

    // Sets total animation increment for a full walk cycle
    void setWalkDirection (float direction) {

        walkDirection = polarNormal(direction);

        if (legsLost >= 8){
            incX = 0;
            incY = 0;
        }
        else {
            incX = (float) Math.cos(walkDirection);
            incY = (float) Math.sin(walkDirection);

            incX = incX * stepSize;
            incY = incY * stepSize;
        }
    }

    // Calculates leg angles based on given direction
    void setLegAngles (float direction)
    {
        float d1 = octant + direction - 2f*octant;
        float d2 = 3f * octant + direction - 2f*octant;
        float st = 2f*octant/3f;
        legAngles = new float[] {d1, d1-st, d1-2*st, d1-3*st, d2, d2+st, d2+2*st, d2+3*st};
    }
    
    
    // Sets the initial position of the spider at (x,y)
    void set_positions (int x, int y){

        bodyStart.set (x, y);
        bodyEnd = new Point (x + incX, y + incY);
        body_inc_x = incX;
        body_inc_y = incY;

        float size = spiderSize/2.5f;
        setLegAngles(faceDirection);

        for (int i=0; i<8; i++) {
            int p = feetPos[i];
            float d = legAngles[i];
            footPosition[p] = new Point (x+(float)Math.cos(d)*size, y+(float)Math.sin(d)*size);
            foot[p] = new Point (footPosition[p].x, footPosition[p].y);
        }
    }

    // This function updates the relative feet position 
    void reset_feet (float x, float y)
    {
        float size = spiderSize/2.5f;
        setLegAngles(faceDirection);

        for (int i=0; i<8; i++) {
            int p = feetPos[i];
            if (missingLeg[p] || pinnedLeg == p) continue;
            float d = legAngles[i];
            footPosition[p].set(x + (float) Math.cos(d) * size, y + (float) Math.sin(d) * size);
            foot[p].set(footPosition[p].x, footPosition[p].y);
        }
    }

    // This function updates the direction of foot movement 
    void update_feet (int x, int y) {

        float ex = x + incX;
        float ey = y + incY;
        float size = spiderSize/2.5f;
        setLegAngles(faceDirection);
        spiderStationary = (pinnedLeg == -1);

        for (int i=0; i<8; i++) {
            int p = feetPos[i];
            if (missingLeg[p] || pinnedLeg == p) continue;
            float d = legAngles[i];
            footInc[p].set (ex+(float)Math.cos(d)*size - footPosition[p].x, ey+(float)Math.sin(d)*size - footPosition[p].y);
            if (Math.abs(footInc[p].x) > 1 || Math.abs(footInc[p].y) > 1) spiderStationary = false;
        }
    }

    // This function updates the relative coordinates of the spider body parts when changing direction
    void update_joints () {

        float joint = spiderSize/8f;
        setLegAngles(jointDirection);

        for (int i=0; i<8; i++) {
            int p = feetPos[i];
            float d = legAngles[i];
            joints[p].set ((float)Math.cos(d)*joint, (float)Math.sin(d)*joint);
        }

        // red mark
        float d = 4f*octant + jointDirection;
        joints[8].set ((float)Math.cos(d), (float)Math.sin(d));

        // fangs/eyes
        d = jointDirection + octant/3f;
        joints[9].set ((float)Math.cos(d), (float)Math.sin(d));
        d = jointDirection - octant/3f;
        joints[10].set ((float)Math.cos(d), (float)Math.sin(d));
    }

    // This function calculates the smallest turn angle (radians) and direction of turn (1 anticlockwise, -1 clockwise) from the current to new direction (in radians)
    void turn_in_smallest_angle (float current_direction, float new_direction, TurnAngle turn)
    {
        float turn_direction;
        float turnAngle;
        if (new_direction > current_direction) {
            if ((new_direction - current_direction) > Math.PI) {
                turn_direction = -1;
                turnAngle = (float) (2.0 * Math.PI - (new_direction - current_direction));
            } else {
                turn_direction = 1;
                turnAngle = new_direction - current_direction;
            }
        } else {
            if ((current_direction - new_direction) > Math.PI) {
                turn_direction = 1;
                turnAngle = (float) (2.0 * Math.PI - (current_direction - new_direction));
            } else {
                turn_direction = -1;
                turnAngle = current_direction - new_direction;
            }
        }

        turn.direction = turn_direction;
        turn.angle = turnAngle;
    }

    // Function for calculating body and leg positions for the current walk cycle.
    // This should be called from the main animation loop.
    void takeStep (){

        if (spider_squished) return;

        walkStage += 1.0 / walkFramesPerStep;

        if (walkStage >= 1.0) {
            walkStage = 0;
            body_inc_x = incX;
            body_inc_y = incY;
            if (pinnedLeg != -1){
                bodyEnd.set (bodyStart.x, bodyStart.y);
                reset_feet (bodyStart.x, bodyStart.y);
            }
            else {
                bodyStart.set(bodyEnd.x, bodyEnd.y);
                bodyEnd.set(bodyStart.x + incX, bodyStart.y + incY);
            }
            update_feet((int) (bodyStart.x), (int) (bodyStart.y));
            newJointDirection = faceDirection; // joint direction lags face direction for realistic movement
        }

        float dx = body_inc_x / walkFramesPerStep;
        float dy = body_inc_y / walkFramesPerStep;
        walkDistance -= (Math.sqrt (dx*dx + dy*dy));
        if (walkDistance <= 0){
            stepSize = 0;
            setWalkDirection(walkDirection);
        }

        if (newFaceDirection != faceDirection) {
            turn_in_smallest_angle (faceDirection, newFaceDirection, turn);
            float inc = octant * turnrate / walkFramesPerStep;
            if (inc > turn.angle) faceDirection = newFaceDirection;
            else faceDirection += (turn.direction * inc);
            faceDirection = polarNormal(faceDirection);
        }

        if (jointDirection != newJointDirection) {
            turn_in_smallest_angle (jointDirection, newJointDirection, turn);
            float inc = octant * turnrate / walkFramesPerStep;
            if (inc > turn.angle) jointDirection = newJointDirection;
            else jointDirection += (turn.direction * inc);
            jointDirection = polarNormal(jointDirection);
            update_joints();
        }

        if (pinnedLeg == -1) bodyPosition.set (bodyStart.x + body_inc_x * walkStage, bodyStart.y + body_inc_y * walkStage);

        if (!spiderStationary) {

            float leg_lift_height_scale = 2f;
            float leg_lift_height;
            int start_foot = 0, end_foot = 4;

            if (walkStage >= 0.25 && walkStage < 0.75 ) {
                start_foot = 4; end_foot = 8;
                leg_lift_height = (float) Math.abs (Math.cos(walkStage * 8f * octant)) / leg_lift_height_scale;
            }
            else {
                leg_lift_height = (float) Math.abs (Math.cos(walkStage * 8f * octant)) / leg_lift_height_scale;
            }

            for (int i = start_foot; i < end_foot; i++) {
                if (missingLeg[i]) continue;
                if (i == pinnedLeg){
                    footPosition[i].set(pinnedPoint.x, pinnedPoint.y);
                    foot[i].set(pinnedPoint.x, pinnedPoint.y);
                }
                else {
                    footPosition[i].set(footPosition[i].x + footInc[i].x * 2f / walkFramesPerStep, footPosition[i].y + footInc[i].y * 2f / walkFramesPerStep);
                    foot[i].set(footPosition[i].x - joints[i].x * leg_lift_height, footPosition[i].y - joints[i].y * leg_lift_height);
                }
            }
        }
    }
    
    // Returns the leg index that is closest to the supplied angle
    int find_leg (float angle) {

        angle = polarNormal(angle);
        float min_angle = 999999;
        int leg = -1;
        setLegAngles(faceDirection);
        for (int i=0; i<8; i++){
            int p = feetPos[i];
            if (missingLeg[p]) continue;
            turn_in_smallest_angle (angle, polarNormal(legAngles[i]), turn);
            if (turn.angle < octant && turn.angle < min_angle) {
                min_angle = turn.angle;
                leg = p;
            }
        }

        return leg;
    }

    // Returns size of spider
    public float spiderSize (){
        return (spiderSize);
    }

    // Returns true if we have lost a leg in the last AI update
    public boolean lostALeg (){
        return (lastLegLostCount != legsLost);
    }

    // Returns the number of missing legs
    public int legsLost (){
        return (legsLost);
    }

    // Returns true if the spider has been squashed in the last AI update
    public boolean spiderJustSquished (){
        if (!spider_squished || checkedSquishState) return false;
        checkedSquishState = true;
        return true;
    }

    // Returns how long we have been draging this spider (milliseconds).
    public long dragTime (long time){
        if (!beingDragged) return 0L;
        return (time - beingDraggedTime);
    }

    public void setDragTime (long time){
        beingDraggedTime = time;
    }

    // Main routine for calculating spider AI behaviour
    // x1, y1 point of (last) interaction
    // mode of interaction - IDLE, DOWN, MOVE, UP
    void spiderAI (float x1, float y1, int mode) {

        lastLegLostCount = legsLost;
        if (spider_squished) return;

        boolean break_leg = false;

        float SCALE = spiderSize / 2f;

        float x = bodyPosition.x;
        float y = bodyPosition.y;

        float dx = x - x1;
        float dy = y - y1;
        float distance = (float) Math.sqrt(dx * dx + dy * dy);
        float hit_distance = distance;

        if (mode == UP) {
            if (distance < SCALE && dragDistance < SCALE) {
                // spider squashed
                bodyStart.set (bodyPosition.x, bodyPosition.y);
                bodyEnd.set (bodyStart.x, bodyStart.y);
                float size = spiderSize/2.5f - spiderSize/8f;
                int spread = (int)(size / 4);
                for (int i=0; i<8; i++) {
                    if (missingLeg[i]) continue;
                    brokenLegAngle[i] = legAngles[anglePos[i]];
                    if (Math.random() > 0.7) twitchLeg[i] = true;
                    foot[i].set (bodyStart.x + joints[i].x + (float)(Math.random()*spread-spread/2), bodyStart.y + joints[i].y + (float)(Math.random()*spread-spread/2));
                    brokenLegAngle[i] += (float)(Math.random()*octant) - octant/2f;
                    footPosition[i].set (foot[i].x + (float)Math.cos(brokenLegAngle[i])*size, foot[i].y + (float)Math.sin(brokenLegAngle[i])*size);
                    brokenLegTimeout[i] = current_time();
                    missingLeg[i] = true;
                }
                spider_squished = true;
                squishedTime = current_time();
                legsLost = 8;
                beingDragged = false;
                pinnedLeg = -1;
            }
            else{
                // cancel drag
                dragDistance = 0;
                haveBrokenLeg = -1;
                if (pinnedLeg == -1) return;
                beingDragged = false;
                pinnedLeg = -1;
            }
        }

        float pinned_distance = 0;

        if (pinnedLeg != -1 && mode == MOVE) {

            dx = x1 - touchDownPoint.x;
            dy = y1 - touchDownPoint.y;

            dragDistance += (float)Math.sqrt (dx*dx + dy*dy);

            footPosition[pinnedLeg].x = x1;
            footPosition[pinnedLeg].y = y1;

            touchDownPoint.x += dx;
            touchDownPoint.y += dy;

            dx = x1 - bodyPosition.x;
            dy = y1 - bodyPosition.y;
            pinned_distance = (float)Math.sqrt (dx*dx + dy*dy);

            if (pinned_distance > 2*SCALE) break_leg = true;
            if (pinned_distance > SCALE) {
                if (!beingDragged){
                    beingDragged = true;
                    beingDraggedTime = current_time();
                }
                pinned_distance = SCALE;
            }

            if (break_leg){
                legsLost++;
                bodyEnd.set (bodyStart.x, bodyStart.y);
                haveBrokenLeg = pinnedLeg;
                brokenLegAngle[pinnedLeg] = (float)(Math.PI + legAngles[anglePos[pinnedLeg]]);
                twitchLeg[pinnedLeg] = true;
                brokenLegTimeout[pinnedLeg] = current_time();
                missingLeg[pinnedLeg] = true;
                beingDragged = false;
                pinnedLeg = -1;
            }
        }

        if (haveBrokenLeg != -1 && mode == MOVE){
            float size = spiderSize/2.5f;
            foot[haveBrokenLeg].set (x1 + (float)Math.cos(brokenLegAngle[haveBrokenLeg])*size, y1 + (float)Math.sin(brokenLegAngle[haveBrokenLeg])*size);
            brokenLegAngle[haveBrokenLeg] += (float)(Math.random()*octant) - octant/2f;
            footPosition[haveBrokenLeg].set (foot[haveBrokenLeg].x + (float)Math.cos(brokenLegAngle[haveBrokenLeg])*size, foot[haveBrokenLeg].y + (float)Math.sin(brokenLegAngle[haveBrokenLeg])*size);
        }

        double angle = polarNormal((float) (Math.atan((x1 - x) / (y1 - y))));
        if (x1 > x && y1 > y) angle = Math.PI / 2.0 - angle;
        else if (x1 < x && y1 > y) angle = Math.PI / 2.0 - angle;
        else if (x1 < x && y1 < y) angle = 3.0 * Math.PI / 2.0 - angle;
        else angle = 3.0 * Math.PI / 2.0 - angle;

        double point_angle = angle;

        angle = angle + Math.random() * Math.PI/3f - Math.PI/6f;
        if (mode == IDLE) {
            // random spider movement
            turn_in_smallest_angle (walkDirection, (float)angle, turn);
            if (turn.angle > Math.PI/2f) angle = walkDirection + turn.direction*Math.PI/2f;
        }
        double walk_angle = angle;

        if (pinnedLeg != -1 && mode == MOVE) {
            // dragging spider - rotate spider based on leg and drag direction
            bodyStart.x = x1 + (float)Math.cos (point_angle+Math.PI)*pinned_distance;
            bodyStart.y = y1 + (float)Math.sin (point_angle+Math.PI)*pinned_distance;
            bodyPosition.set (bodyStart.x, bodyStart.y);

            setLegAngles(0);
            setFaceDirection((float) (angle - legAngles[anglePos[pinnedLeg]]));
            reset_feet (bodyStart.x, bodyStart.y);
        }

        float proximity = alert_proximity;
        // set spider movement speed relative to point of last interaction
        int framesPerStep = (int)(5 + hit_distance / SCALE);
        if (mode == IDLE){
            framesPerStep = framesPerStep * 2;
            proximity = proximity / 2f;
        }

        int leg = pinnedLeg;
        if (distance < SCALE && mode == DOWN){
            leg = find_leg ((float)point_angle);
            touchDownPoint.set (x1, y1);
        }

        if (leg == -1) {
            if (distance > proximity) {
                if (Math.random() > 0.5 || current_time() < retreat_timeout) return; // keep spider stationary
                // random movement
                framesPerStep = (int) (targetFramesPerStep + Math.random() * targetFramesPerStep);
                distance -= proximity;
                distance = distance * (float) Math.random();
            } else {
                // move away from point of interaction
                distance = proximity - distance;
                walk_angle += Math.PI;
                if (mode != IDLE) retreat_timeout = (long) (current_time() + Math.random() * 1000 + 500);
            }

            setWalkDistance(distance);
            setFaceDirection((float) (angle));
            setWalkDirection((float) (walk_angle));
            setFramesPerStep(framesPerStep);
        }
        else {
            pinnedLeg = leg;
            bodyStart.set (bodyPosition.x, bodyPosition.y);
            bodyEnd.set (bodyStart.x, bodyStart.y);
            pinnedPoint.set (footPosition[leg].x, footPosition[leg].y);
            setWalkDistance(break_leg ? 6 * SCALE : SCALE);
            setWalkDirection(legAngles[anglePos[pinnedLeg]]);
            setFramesPerStep(framesPerStep);
        }
    }

    // Change the spider walk direction if the point (x1, y1) is less than (distance) units away.
    // The walk direction is directly away from the point in question. 
    void avoid_position (float x1, float y1, float distance){

        float x = bodyPosition.x;
        float y = bodyPosition.y;

        float dx = x - x1;
        float dy = y - y1;
        float point_distance = (float) Math.sqrt(dx * dx + dy * dy);

        float avoid_distance = distance + spiderSize/2f;

        if (point_distance <= avoid_distance){

            double angle = polarNormal((float) (Math.atan((x1 - x) / (y1 - y))));
            if (x1 > x && y1 > y) angle = Math.PI / 2.0 - angle;
            else if (x1 < x && y1 > y) angle = Math.PI / 2.0 - angle;
            else if (x1 < x && y1 < y) angle = 3.0 * Math.PI / 2.0 - angle;
            else angle = 3.0 * Math.PI / 2.0 - angle;

            double walk_angle = angle + Math.PI;
            setWalkDistance(avoid_distance - point_distance);
            setWalkDirection((float) (walk_angle));
        }
    }

    // For each spider find the closest spider next to it.  If it is too close, then move away.
    static void preventCollisions (LinkedList<Spiders> spiders) {

        for (int i=1; i<spiders.size(); i++){
            Spiders sp1 = spiders.get(i);
            float min_distance = 999999;
            int idx = 0;
            for (int j=0; j<i; j++){
                Spiders sp2 = spiders.get(j);
                float dx = sp1.bodyPosition.x - sp2.bodyPosition.x;
                float dy = sp1.bodyPosition.y - sp2.bodyPosition.y;
                float distance = (float) Math.sqrt(dx * dx + dy * dy);
                if (distance < min_distance) {
                    idx = j;
                    min_distance = distance;
                }
            }
            Spiders sp2 = spiders.get(idx);

            if (sp2.spiderSize >= sp1.spiderSize || Math.random() > 0.5)
                if (Spiders.current_time() > sp1.retreat_timeout)
                    sp1.avoid_position (sp2.bodyPosition.x, sp2.bodyPosition.y, sp2.spiderSize/2f);
                else
                if (Spiders.current_time() > sp2.retreat_timeout)
                    sp2.avoid_position (sp1.bodyPosition.x, sp1.bodyPosition.y, sp1.spiderSize/2f);
        }
    }

    // device specific display routines =============================================
    
    // ANDROID display routines

    Paint paint = new Paint();
    float spot_size;
    int spot_color;

    // device specific system time (milliseconds)
    static long current_time() {
        return System.currentTimeMillis();
    }

    void initialize_draw () {

        paint.setAntiAlias(true);
        paint.setStrokeWidth(2f);
        paint.setColor(Color.BLACK);
        paint.setStyle(Paint.Style.FILL);
        paint.setStrokeJoin(Paint.Join.ROUND);

        float radius = spiderSize / 8f;
        spot_size = (float)(radius/3f + Math.random()*radius/6f - radius/12f);
        spot_color = Color.rgb((int)(Math.random()*80 + 176), 0, 0);
    }

    // draw the spider
    void draw (Canvas canvas) {

        final int bodyColor = Color.BLACK;

        float global_alpha  = 1f;

        if (spider_squished) {
            global_alpha = 1f - (current_time() - squishedTime) / 3000f;
            if (global_alpha < 0) return;
        }

        float radius = spiderSize / 8f;

        paint.setColor(bodyColor);
        int shadow = 3;
        paint.setAlpha ((int)(60*global_alpha));
        for (int i = 0; i < 8; i++) {
            if (missingLeg[i]) continue;
            canvas.drawLine(bodyPosition.x + joints[i].x+shadow, bodyPosition.y + joints[i].y+shadow, foot[i].x+shadow, foot[i].y+shadow, paint);
        }
        canvas.drawCircle(bodyPosition.x+shadow, bodyPosition.y+shadow, radius, paint);

        paint.setColor(bodyColor);

        for (int i = 0; i < 8; i++) {
            if (missingLeg[i]){
                long delta = current_time() - brokenLegTimeout[i];
                float alpha = delta / 3000f;
                if (alpha > 1) alpha = 1;
                paint.setAlpha ((int)((1f-alpha) * 255 * global_alpha));
                if (alpha < 1){
                    int xm = (int)(foot[i].x + footPosition[i].x) / 2;
                    int ym = (int)(foot[i].y + footPosition[i].y) / 2;
                    if (twitchLeg[i]){
                        xm += (int)((Math.random()*8-4) * (1f-alpha));
                        ym += (int)((Math.random()*8-4) * (1f-alpha));
                    }
                    canvas.drawLine(foot[i].x, foot[i].y, xm, ym, paint);
                    canvas.drawLine(xm, ym, footPosition[i].x, footPosition[i].y, paint);
                }
            }
            else {
                paint.setAlpha((int) (255 * global_alpha));
                canvas.drawLine(bodyPosition.x + joints[i].x, bodyPosition.y + joints[i].y, foot[i].x, foot[i].y, paint);
            }
        }

        for (int k=0; k<(spider_squished ? 5 : 1); k++) {

            if (spider_squished && squishRadius[0] == 0){
                for (int j=0; j<5; j++){
                    squishTX[j] = (float)(Math.random() * radius - radius/2f);
                    squishRadius[j] = (float)(Math.random() * radius - radius/2f);
                }
            }

            if (spider_squished) canvas.translate (squishTX[k], squishTX[k]);

            float rad = radius + squishRadius[k];

            paint.setColor(0xff804040);
            paint.setAlpha((int)((spider_squished ? 70 : 255) * global_alpha));
            paint.setStrokeWidth(spiderSize / 25f);
            paint.setStrokeWidth(spiderSize / 25f);
            paint.setStrokeCap(Paint.Cap.ROUND);
            for (int i = 9; i < 11; i++) {
                canvas.drawLine(bodyPosition.x, bodyPosition.y, bodyPosition.x + joints[i].x * rad * 1.2f, bodyPosition.y + joints[i].y * rad * 1.2f, paint);
            }
            paint.setStrokeCap(Paint.Cap.SQUARE);

            paint.setColor(bodyColor);
            paint.setAlpha((int)((spider_squished ? 70 : 255) * global_alpha));
            paint.setStrokeWidth(2f);
            canvas.drawCircle(bodyPosition.x, bodyPosition.y, rad, paint);

            paint.setColor(spot_color);
            paint.setAlpha((int)((spider_squished ? 70 : 255) * global_alpha));
            canvas.drawCircle(bodyPosition.x + joints[8].x * radius / 2.5f, bodyPosition.y + joints[8].y * radius / 2.5f, spot_size, paint);

            if (spider_squished) canvas.translate (-squishTX[k], -squishTX[k]);
        }
    }
}
