package au.com.netplaysoftware.androidspiders;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Point;
import android.media.AudioManager;
import android.media.SoundPool;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import java.util.LinkedList;

public class MainCanvas extends SurfaceView implements
        SurfaceHolder.Callback {

    SurfaceHolder surfaceHolder;

    private final int MAX_SPIDERS = 15; // Maximum number of spiders displayed
    private final int GLOBAL_SCALE = 20; // Determines overall size of spiders
    private final int GLOBAL_FPS = 30; // Desired animation frame rate
    private final int AI_UPDATE_RATE = 5000; // How often we update each spiders behaviour

    private int X_SIZE; // display width in pixels
    private int Y_SIZE; // display height in pixels
    private int SCALE; // spider size relative to screen size

    LinkedList<Spiders> spiders = new LinkedList<>(); // the list of spiders

    private long lastAIUpdateTime; // track last AI update time
    private int spiderUpdateTick; // track which spider AI needs updating

    private SoundPool sound;

    private int splat1Sound;
    private int splat2Sound;
    private int splat3Sound;
    private int popSound;
    private int yeepSound;

    public MainCanvas(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        constructMainCanvas(context);
    }

    public MainCanvas(Context context, AttributeSet attrs) {
        super(context, attrs);
        constructMainCanvas(context);
    }

    public MainCanvas(Context context) {
        super(context);
        constructMainCanvas(context);
    }

    public void constructMainCanvas(Context context) {

        surfaceHolder = getHolder();
        surfaceHolder.addCallback(this);
        setFocusable(true);
        setWillNotDraw(false);

        if (!isInEditMode()) {
            sound = new SoundPool(32, AudioManager.STREAM_MUSIC, 0);
            splat1Sound = sound.load(context, R.raw.splat1, 1);
            splat2Sound = sound.load(context, R.raw.splat2, 1);
            splat3Sound = sound.load(context, R.raw.splat3, 1);
            popSound = sound.load(context, R.raw.pop, 1);
            yeepSound = sound.load(context, R.raw.yeep, 1);
        }
    }

    private void Initialize() {

        X_SIZE = getWidth();
        Y_SIZE = getHeight();
        SCALE = X_SIZE / GLOBAL_SCALE;

        spiders.clear();

        for (int i=0; i < MAX_SPIDERS; i++) {

            // randomly place spider on view
            int x = (int) (Math.random() * X_SIZE);
            int y = (int) (Math.random() * Y_SIZE);

            // generate random size
            float size = 2 * SCALE;
            size = size + (float) (Math.random() * size / 2f) - size / 4f;

            // make an occasional big spider
            if (Math.random() <= 0.02 * MAX_SPIDERS) size = size * 2;

            // Spiders (int x, int y, float direction, float turn_rate, float step_size, walk_distance, float size, int fps)
            spiders.add(new Spiders(x, y, 0, 2f, size / 3, 0, size, 5));
        }

        spiderUpdateTick = 0;
        lastAIUpdateTime = System.currentTimeMillis();

        invalidate();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        //Initialize();
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        Initialize();
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {

        Point point = new Point ((int)event.getX(), (int)event.getY());

        switch (event.getAction() & MotionEvent.ACTION_MASK) {
            case MotionEvent.ACTION_POINTER_DOWN:
            case MotionEvent.ACTION_DOWN:
                inputDown(point);
                break;

            case MotionEvent.ACTION_MOVE:
                inputMove(point);
                break;

            case MotionEvent.ACTION_POINTER_UP:
            case MotionEvent.ACTION_UP:
                inputUp(point);
                break;

            default:
                return super.onTouchEvent(event);
        }
        return true;
    }

    public void inputDown(Point point) {
        // update spider reaction
        for (int i = 0; i < spiders.size(); i++) {
            spiders.get(i).spiderAI(point.x, point.y, Spiders.DOWN);
        }
        Spiders.preventCollisions(spiders);
    }

    public void inputMove(Point point) {
        // spider reaction
        for (int i = 0; i < spiders.size(); i++) {
            if (Math.random() > 0.7) continue;
            Spiders s = spiders.get(i);
            s.spiderAI(point.x, point.y, Spiders.MOVE);

            // if removed a leg play pop sound
            if (s.lostALeg())
                sound.play(popSound, 0.5f, 0.5f, 0, 0, 1f + s.legsLost() / 8f * 0.3f);

            // make spider yelp when being dragged by leg
            long now = System.currentTimeMillis();
            if (s.dragTime(now) > 500 + Math.random() * 1000){
                s.setDragTime (now);
                float pitch;
                if (s.spiderSize() > 3f * SCALE) pitch = 0.5f;
                else pitch = (2f * SCALE) / s.spiderSize();
                sound.play(yeepSound, 0.5f, 0.5f, 0, 0, pitch);
            }
        }
        Spiders.preventCollisions(spiders);
    }

    public void inputUp(Point point) {
        // spider reaction
        for (int i = 0; i < spiders.size(); i++) {
            Spiders s = spiders.get(i);
            s.spiderAI(point.x, point.y, Spiders.UP);
            if (s.spiderJustSquished()) {
                double select = Math.random();
                float pitch;
                if (s.spiderSize() > 3f * SCALE) pitch = 0.5f;
                else pitch = (2f * SCALE) / s.spiderSize();
                if (select < 0.4) sound.play(splat1Sound, 0.5f, 0.5f, 0, 0, pitch);
                else if (select < 0.8) sound.play(splat2Sound, 0.5f, 0.5f, 0, 0, pitch);
                else sound.play(splat3Sound, 0.5f, 0.5f, 0, 0, pitch);
            }
        }
        Spiders.preventCollisions(spiders);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        canvas.save();

        long now = System.currentTimeMillis();

        if (now - lastAIUpdateTime >= AI_UPDATE_RATE / MAX_SPIDERS) {
            lastAIUpdateTime = now;
            spiderUpdateTick = (spiderUpdateTick + 1) % MAX_SPIDERS;
            if (spiderUpdateTick < spiders.size()) {
                spiders.get(spiderUpdateTick).spiderAI((float) Math.random() * X_SIZE, (float) Math.random() * Y_SIZE, Spiders.IDLE);
                Spiders.preventCollisions(spiders);
            }
        }

        for (int i=0; i < spiders.size(); i++) {
            Spiders s = spiders.get(i);
            s.takeStep();
            s.draw(canvas);
        }

        postInvalidateDelayed (1000 / GLOBAL_FPS);

        canvas.restore();
    }
}