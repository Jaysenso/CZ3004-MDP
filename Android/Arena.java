package com.example.mdp;

import android.animation.AnimatorSet;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.Chronometer;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import android.animation.ObjectAnimator;
import android.graphics.Color;

import androidx.appcompat.app.AppCompatActivity;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.nio.charset.Charset;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;



public class Arena extends AppCompatActivity {
    public static final String SHARED_PREFS = "sharedPrefs";
    private static final String TAG = "Arena->DEBUG";
    public static boolean firstStart = true;
    private CanvasGrid canvasGrid;

    public void saveData() {
        SharedPreferences sharedPreferences = getSharedPreferences(SHARED_PREFS, MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPreferences.edit();

        editor.putFloat("obs1X", findViewById(R.id.obstacle1).getTranslationX());
        editor.putFloat("obs1Y", findViewById(R.id.obstacle1).getTranslationY());
        editor.putFloat("obs1Rotation", findViewById(R.id.obstacle1).getRotation());

        editor.putFloat("obs2X", findViewById(R.id.obstacle2).getTranslationX());
        editor.putFloat("obs2Y", findViewById(R.id.obstacle2).getTranslationY());
        editor.putFloat("obs2Rotation", findViewById(R.id.obstacle2).getRotation());

        editor.putFloat("obs3X", findViewById(R.id.obstacle3).getTranslationX());
        editor.putFloat("obs3Y", findViewById(R.id.obstacle3).getTranslationY());
        editor.putFloat("obs3Rotation", findViewById(R.id.obstacle3).getRotation());

        editor.putFloat("obs4X", findViewById(R.id.obstacle4).getTranslationX());
        editor.putFloat("obs4Y", findViewById(R.id.obstacle4).getTranslationY());
        editor.putFloat("obs4Rotation", findViewById(R.id.obstacle4).getRotation());

        editor.putFloat("obs5X", findViewById(R.id.obstacle5).getTranslationX());
        editor.putFloat("obs5Y", findViewById(R.id.obstacle5).getTranslationY());
        editor.putFloat("obs5Rotation", findViewById(R.id.obstacle5).getRotation());

        editor.putFloat("obs6X", findViewById(R.id.obstacle6).getTranslationX());
        editor.putFloat("obs6Y", findViewById(R.id.obstacle6).getTranslationY());
        editor.putFloat("obs6Rotation", findViewById(R.id.obstacle6).getRotation());

        editor.putFloat("obs7X", findViewById(R.id.obstacle7).getTranslationX());
        editor.putFloat("obs7Y", findViewById(R.id.obstacle7).getTranslationY());
        editor.putFloat("obs7Rotation", findViewById(R.id.obstacle7).getRotation());

        editor.putFloat("obs8X", findViewById(R.id.obstacle8).getTranslationX());
        editor.putFloat("obs8Y", findViewById(R.id.obstacle8).getTranslationY());
        editor.putFloat("obs8Rotation", findViewById(R.id.obstacle8).getRotation());

        editor.putFloat("carX", findViewById(R.id.car).getTranslationX());
        editor.putFloat("carY", findViewById(R.id.car).getTranslationY());
        editor.putFloat("carRotation", findViewById(R.id.car).getRotation());

        int x = (int) (car.getX() + SNAP_GRID_INTERVAL) / SNAP_GRID_INTERVAL;
        int y = (int) (car.getY() + SNAP_GRID_INTERVAL) / SNAP_GRID_INTERVAL;

        editor.putString("x_tv", String.valueOf(car_x.getText()));
        editor.putString("y_tv", String.valueOf(car_y.getText()));
        editor.putString("car_dir", String.valueOf(car_dir.getText()));
        editor.apply();
    }

    public void loadData() {
        SharedPreferences sharedPreferences = getSharedPreferences(SHARED_PREFS, MODE_PRIVATE);
        obstacle1.setX(sharedPreferences.getFloat("obs1X", 0.0f));
        obstacle1.setY(sharedPreferences.getFloat("obs1Y", 0.0f));
        obstacle1.setRotation((sharedPreferences.getFloat("obs1Rotation", 0.0f)));

        obstacle2.setX(sharedPreferences.getFloat("obs2X", 0.0f));
        obstacle2.setY(sharedPreferences.getFloat("obs2Y", 0.0f));
        obstacle2.setRotation((sharedPreferences.getFloat("obs2Rotation", 0.0f)));

        obstacle3.setX(sharedPreferences.getFloat("obs3X", 0.0f));
        obstacle3.setY(sharedPreferences.getFloat("obs3Y", 0.0f));
        obstacle3.setRotation((sharedPreferences.getFloat("obs3Rotation", 0.0f)));

        obstacle4.setX(sharedPreferences.getFloat("obs4X", 0.0f));
        obstacle4.setY(sharedPreferences.getFloat("obs4Y", 0.0f));
        obstacle4.setRotation((sharedPreferences.getFloat("obs4Rotation", 0.0f)));

        obstacle5.setX(sharedPreferences.getFloat("obs5X", 0.0f));
        obstacle5.setY(sharedPreferences.getFloat("obs5Y", 0.0f));
        obstacle5.setRotation((sharedPreferences.getFloat("obs5Rotation", 0.0f)));

        obstacle6.setX(sharedPreferences.getFloat("obs6X", 0.0f));
        obstacle6.setY(sharedPreferences.getFloat("obs6Y", 0.0f));
        obstacle6.setRotation((sharedPreferences.getFloat("obs6Rotation", 0.0f)));

        obstacle7.setX(sharedPreferences.getFloat("obs7X", 0.0f));
        obstacle7.setY(sharedPreferences.getFloat("obs7Y", 0.0f));
        obstacle7.setRotation((sharedPreferences.getFloat("obs7Rotation", 0.0f)));

        obstacle8.setX(sharedPreferences.getFloat("obs8X", 0.0f));
        obstacle8.setY(sharedPreferences.getFloat("obs8Y", 0.0f));
        obstacle8.setRotation((sharedPreferences.getFloat("obs8Rotation", 0.0f)));

        car.setX(sharedPreferences.getFloat("carX", 0.0f));
        car.setY(sharedPreferences.getFloat("carY", 0.0f));
        car.setRotation(sharedPreferences.getFloat("carRotation", 0.0f));

        car_x.setText((sharedPreferences.getString("x_tv", "")));
        car_y.setText((sharedPreferences.getString("y_tv", "")));
        car_dir.setText((sharedPreferences.getString("car_dir", "")));
        Log.d(TAG, "obs1X: " + sharedPreferences.getFloat("obs1X", 0.0f));
    }

    private static final int SNAP_GRID_INTERVAL = 35;
    private static final int ANIMATOR_DURATION = 1000;

    private final int INITIAL_X = SNAP_GRID_INTERVAL - SNAP_GRID_INTERVAL;
    private final int INITIAL_Y = 18 * SNAP_GRID_INTERVAL - SNAP_GRID_INTERVAL;

    private boolean canSetObstacles = false;
    private String curMode = "IDLE";
    private String timerMode = "STOP";

    Button IRButton, SPButton, resetButton, preset1Button, setButton, timerButton, saveButton;
    ImageView obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, car;
    TextView statusWindow, car_x, car_y, car_dir;

    Map<Integer, ImageView> obstacles;


    protected void onPause() {
        super.onPause();
        Log.d("onpause", "OnPause() called");
        saveData();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        // Restore saved instance state
        super.onCreate(savedInstanceState);
        Log.d(TAG, "onCreate called");

        setContentView(R.layout.arena);

        // start listening for incoming messages
        LocalBroadcastManager.getInstance(this).registerReceiver(myReceiver, new IntentFilter("incomingMessage"));

        initObstacles();
        initButtons();
        initMovementButtons();

        if (!firstStart) {
            loadData();
        } else {
            firstStart = false;
            saveData();
        }
    }

    /**
     * Initializes obstacles and setup listeners
     */
    private void initObstacles() {
        obstacle1 = findViewById(R.id.obstacle1);
        obstacle2 = findViewById(R.id.obstacle2);
        obstacle3 = findViewById(R.id.obstacle3);
        obstacle4 = findViewById(R.id.obstacle4);
        obstacle5 = findViewById(R.id.obstacle5);
        obstacle6 = findViewById(R.id.obstacle6);
        obstacle7 = findViewById(R.id.obstacle7);
        obstacle8 = findViewById(R.id.obstacle8);

        obstacles = new HashMap<Integer, ImageView>() {
            {
                put(1, obstacle1);
                put(2, obstacle2);
                put(3, obstacle3);
                put(4, obstacle4);
                put(5, obstacle5);
                put(6, obstacle6);
                put(7, obstacle7);
                put(8, obstacle8);
            }
        };

        obstacle1.setOnClickListener(view -> {
            obstacle1.setRotation((obstacle1.getRotation() + 90) % 360);
            int orientation = (int) obstacle1.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle1.setImageResource(Helper.resources.get("o1n"));
                    break;
                case 1:
                    obstacle1.setImageResource(Helper.resources.get("o1e"));
                    break;
                case 2:
                    obstacle1.setImageResource(Helper.resources.get("o1s"));
                    break;
                case 3:
                    obstacle1.setImageResource(Helper.resources.get("o1w"));
                    break;
            }
        });

        obstacle2.setOnClickListener(view -> {
            obstacle2.setRotation((obstacle2.getRotation() + 90) % 360);
            int orientation = (int) obstacle2.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle2.setImageResource(Helper.resources.get("o2n"));
                    break;
                case 1:
                    obstacle2.setImageResource(Helper.resources.get("o2e"));
                    break;
                case 2:
                    obstacle2.setImageResource(Helper.resources.get("o2s"));
                    break;
                case 3:
                    obstacle2.setImageResource(Helper.resources.get("o2w"));
                    break;
            }
        });

        obstacle3.setOnClickListener(view -> {
            obstacle3.setRotation((obstacle3.getRotation() + 90) % 360);
            int orientation = (int) obstacle3.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle3.setImageResource(Helper.resources.get("o3n"));
                    break;
                case 1:
                    obstacle3.setImageResource(Helper.resources.get("o3e"));
                    break;
                case 2:
                    obstacle3.setImageResource(Helper.resources.get("o3s"));
                    break;
                case 3:
                    obstacle3.setImageResource(Helper.resources.get("o3w"));
                    break;
            }
        });

        obstacle4.setOnClickListener(view -> {
            obstacle4.setRotation((obstacle4.getRotation() + 90) % 360);
            int orientation = (int) obstacle4.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle4.setImageResource(Helper.resources.get("o4n"));
                    break;
                case 1:
                    obstacle4.setImageResource(Helper.resources.get("o4e"));
                    break;
                case 2:
                    obstacle4.setImageResource(Helper.resources.get("o4s"));
                    break;
                case 3:
                    obstacle4.setImageResource(Helper.resources.get("o4w"));
                    break;
            }
        });

        obstacle5.setOnClickListener(view -> {
            obstacle5.setRotation((obstacle5.getRotation() + 90) % 360);
            int orientation = (int) obstacle5.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle5.setImageResource(Helper.resources.get("o5n"));
                    break;
                case 1:
                    obstacle5.setImageResource(Helper.resources.get("o5e"));
                    break;
                case 2:
                    obstacle5.setImageResource(Helper.resources.get("o5s"));
                    break;
                case 3:
                    obstacle5.setImageResource(Helper.resources.get("o5w"));
                    break;
            }
        });

        obstacle6.setOnClickListener(view -> {
            obstacle6.setRotation((obstacle6.getRotation() + 90) % 360);
            int orientation = (int) obstacle6.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle6.setImageResource(Helper.resources.get("o6n"));
                    break;
                case 1:
                    obstacle6.setImageResource(Helper.resources.get("o6e"));
                    break;
                case 2:
                    obstacle6.setImageResource(Helper.resources.get("o6s"));
                    break;
                case 3:
                    obstacle6.setImageResource(Helper.resources.get("o6w"));
                    break;
            }
        });

        obstacle7.setOnClickListener(view -> {
            obstacle7.setRotation((obstacle7.getRotation() + 90) % 360);
            int orientation = (int) obstacle7.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle7.setImageResource(Helper.resources.get("o7n"));
                    break;
                case 1:
                    obstacle7.setImageResource(Helper.resources.get("o7e"));
                    break;
                case 2:
                    obstacle7.setImageResource(Helper.resources.get("o7s"));
                    break;
                case 3:
                    obstacle7.setImageResource(Helper.resources.get("o7w"));
                    break;
            }
        });

        obstacle8.setOnClickListener(view -> {
            obstacle8.setRotation((obstacle8.getRotation() + 90) % 360);
            int orientation = (int) obstacle8.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle8.setImageResource(Helper.resources.get("o8n"));
                    break;
                case 1:
                    obstacle8.setImageResource(Helper.resources.get("o8e"));
                    break;
                case 2:
                    obstacle8.setImageResource(Helper.resources.get("o8s"));
                    break;
                case 3:
                    obstacle8.setImageResource(Helper.resources.get("o8w"));
                    break;
            }
        });

        obstacle1.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;
            int orientation;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!canSetObstacles) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        orientation = (int) obstacle1.getRotation();
                        obstacle1.setRotation(0);
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle1.setX(obstacle1.getX() + dx);
                        obstacle1.setY(obstacle1.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle1.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle1.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        // Log.d(TAG, "obstacle1 is at " + snapToX + "," + snapToY);
                        obstacle1.setX(snapToX);
                        obstacle1.setY(snapToY);
                        obstacle1.setRotation(orientation % 360);
                        break;
                    default:
                        break;
                }
                return true;
            }
        });

        obstacle2.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;
            int orientation;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!canSetObstacles) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        orientation = (int) obstacle2.getRotation();
                        obstacle2.setRotation(0);
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle2.setX(obstacle2.getX() + dx);
                        obstacle2.setY(obstacle2.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle2.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle2.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        // Log.d(TAG, "obstacle2 is at " + snapToX + "," + snapToY);
                        obstacle2.setX(snapToX);
                        obstacle2.setY(snapToY);
                        obstacle2.setRotation(orientation % 360);
                        break;
                    default:
                        break;
                }
                return true;
            }
        });

        obstacle3.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;
            int orientation;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!canSetObstacles) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        orientation = (int) obstacle3.getRotation();
                        obstacle3.setRotation(0);
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle3.setX(obstacle3.getX() + dx);
                        obstacle3.setY(obstacle3.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle3.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle3.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;

                        obstacle3.setX(snapToX);
                        obstacle3.setY(snapToY);
                        obstacle3.setRotation(orientation % 360);
                        break;
                    default:
                        break;
                }
                return true;
            }
        });

        obstacle4.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;
            int orientation;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!canSetObstacles) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        orientation = (int) obstacle4.getRotation();
                        obstacle4.setRotation(0);
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle4.setX(obstacle4.getX() + dx);
                        obstacle4.setY(obstacle4.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle4.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle4.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;

                        obstacle4.setX(snapToX);
                        obstacle4.setY(snapToY);
                        obstacle4.setRotation(orientation % 360);
                        break;
                    default:
                        break;
                }
                return true;
            }
        });

        obstacle5.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;
            int orientation;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!canSetObstacles) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        orientation = (int) obstacle5.getRotation();
                        obstacle5.setRotation(0);
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle5.setX(obstacle5.getX() + dx);
                        obstacle5.setY(obstacle5.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle5.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle5.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;

                        obstacle5.setX(snapToX);
                        obstacle5.setY(snapToY);
                        obstacle5.setRotation(orientation % 360);
                        break;
                    default:
                        break;
                }
                return true;
            }
        });

        obstacle6.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;
            int orientation;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!canSetObstacles) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        orientation = (int) obstacle6.getRotation();
                        obstacle6.setRotation(0);
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle6.setX(obstacle6.getX() + dx);
                        obstacle6.setY(obstacle6.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle6.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle6.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;

                        obstacle6.setX(snapToX);
                        obstacle6.setY(snapToY);
                        obstacle6.setRotation(orientation % 360);
                        break;
                    default:
                        break;
                }
                return true;
            }
        });

        obstacle7.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;
            int orientation;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!canSetObstacles) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        orientation = (int) obstacle7.getRotation();
                        obstacle7.setRotation(0);
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle7.setX(obstacle7.getX() + dx);
                        obstacle7.setY(obstacle7.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle7.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle7.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;

                        obstacle7.setX(snapToX);
                        obstacle7.setY(snapToY);
                        obstacle7.setRotation(orientation % 360);
                        break;
                    default:
                        break;
                }
                return true;
            }
        });

        obstacle8.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;
            int orientation;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!canSetObstacles) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        orientation = (int) obstacle8.getRotation();
                        obstacle8.setRotation(0);
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle8.setX(obstacle8.getX() + dx);
                        obstacle8.setY(obstacle8.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle8.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle8.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL))
                                * SNAP_GRID_INTERVAL;

                        obstacle8.setX(snapToX);
                        obstacle8.setY(snapToY);
                        obstacle8.setRotation(orientation % 360);
                        break;
                    default:
                        break;
                }
                return true;
            }
        });
    }

    private void setObstacleImage(int obstacleNumber, String image) {
        int orientation = (int) obstacles.get(obstacleNumber).getRotation();

        try {
            if (orientation == 0) {
                obstacles.get(obstacleNumber).setImageResource(Helper.resources.get(image + "n"));
            } else if (orientation == 90) {
                obstacles.get(obstacleNumber).setImageResource(Helper.resources.get(image + "e"));
            } else if (orientation == 180) {
                obstacles.get(obstacleNumber).setImageResource(Helper.resources.get(image + "s"));
            } else if (orientation == 270) {
                obstacles.get(obstacleNumber).setImageResource(Helper.resources.get(image + "w"));
            } else {
                obstacles.get(obstacleNumber).setImageResource(Helper.resources.get(image));
                obstacles.get(obstacleNumber).setRotation(0);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /*
     * Initializes the arrow buttons
     */
    private void initMovementButtons() {
        ImageButton forwardButton = findViewById(R.id.forwardButton);
        forwardButton.setOnClickListener(v -> {

            // Bluetooth message
            if (BluetoothService.BluetoothConnectionStatus) {
                byte[] bytes = "STM:n".getBytes(Charset.defaultCharset());
            }

            // Animation
            forwardButtonCommand(1);
        });

        ImageButton reverseButton = findViewById(R.id.reverseButton);
        reverseButton.setOnClickListener(v -> {
            if (BluetoothService.BluetoothConnectionStatus) {
                byte[] bytes = "STM:s".getBytes(Charset.defaultCharset());
            }

            // Animation
            reverseButtonCommand(1);
        });

        ImageButton leftButton = findViewById(R.id.leftButton);
        leftButton.setOnClickListener(v -> {
            if (BluetoothService.BluetoothConnectionStatus) {
                byte[] bytes = "STM:w".getBytes(Charset.defaultCharset());
            }

            leftCommand();
        });

        ImageButton rightButton = findViewById(R.id.rightButton);
        rightButton.setOnClickListener(v -> {
            if (BluetoothService.BluetoothConnectionStatus) {
                byte[] bytes = "STM:e".getBytes(Charset.defaultCharset());
            }
            rightCommand();
        });
    }

    /**
     * Initalizes buttons, car and setup listeners
     */
    private void initButtons() {
        // Declarations
        car = findViewById(R.id.car);
        car_x = findViewById(R.id.x_coor);
        car_y = findViewById(R.id.y_coor);
        car_dir = findViewById(R.id.directionText);
        IRButton = findViewById(R.id.irBtn);
        SPButton = findViewById(R.id.spBtn);
        resetButton = findViewById(R.id.resetBtn);
        preset1Button = findViewById(R.id.savedBtn);
        setButton = findViewById(R.id.setBtn);
        saveButton = findViewById(R.id.sendBtn);
        timerButton = findViewById(R.id.timerBtn);
        statusWindow = findViewById(R.id.robotStatusText);

        // Events
        IRButton.setOnClickListener(view -> beginIRTask());
        SPButton.setOnClickListener(view -> beginSPTask());
        resetButton.setOnClickListener(view -> setResetButton());
        preset1Button.setOnClickListener(view -> setPreset1Button());
        setButton.setOnClickListener(view -> toggleSetMode());
        saveButton.setOnClickListener(view -> sendObstacles());
        //timerButton.setOnClickListener(view -> stopTimerButton());
        timerButton.setOnClickListener(view -> toggleTimerBtn());

        // Initialize car to bottom left
        car.setX(INITIAL_X);
        car.setY(INITIAL_Y);
        updateXYDirText();
    }

    private void sleepFor(int time) {
        try {
            TimeUnit.MILLISECONDS.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void forwardButtonCommand(int noOfGrids) {
        int orientation = (int) car.getRotation();
        int new_x, new_y;
        ObjectAnimator animator;
        switch (((orientation / 90) % 4 + 4) % 4) {
            case 0:
                new_y = (int) car.getY() - noOfGrids * SNAP_GRID_INTERVAL;
                car.setY(new_y);
                animator = ObjectAnimator.ofFloat(car, "y", new_y);
                animator.setDuration((long) noOfGrids * ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 1:
                new_x = (int) car.getX() + noOfGrids * SNAP_GRID_INTERVAL;
                car.setX(new_x);
                animator = ObjectAnimator.ofFloat(car, "x", new_x);
                animator.setDuration((long) noOfGrids * ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 2:
                new_y = (int) car.getY() + noOfGrids * SNAP_GRID_INTERVAL;
                car.setY(new_y);
                animator = ObjectAnimator.ofFloat(car, "y", new_y);
                animator.setDuration((long) noOfGrids * ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 3:
                new_x = (int) car.getX() - noOfGrids * SNAP_GRID_INTERVAL;
                car.setX(new_x);
                animator = ObjectAnimator.ofFloat(car, "x", new_x);
                animator.setDuration((long) noOfGrids * ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            default:
                break;
        }
    }

    private void reverseButtonCommand(int noOfGrids) {
        int orientation = (int) car.getRotation();
        int new_x, new_y;
        ObjectAnimator animator;
        switch (((orientation / 90) % 4 + 4) % 4) {
            case 0:
                new_y = (int) car.getY() + noOfGrids * SNAP_GRID_INTERVAL;
                car.setY(new_y);
                animator = ObjectAnimator.ofFloat(car, "y", new_y);
                animator.setDuration((long) noOfGrids * ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 1:
                new_x = (int) car.getX() - noOfGrids * SNAP_GRID_INTERVAL;
                car.setX(new_x);
                animator = ObjectAnimator.ofFloat(car, "x", new_x);
                animator.setDuration((long) noOfGrids * ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 2:
                new_y = (int) car.getY() - noOfGrids * SNAP_GRID_INTERVAL;
                car.setY(new_y);
                animator = ObjectAnimator.ofFloat(car, "y", new_y);
                animator.setDuration((long) noOfGrids * ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 3:
                new_x = (int) car.getX() + noOfGrids * SNAP_GRID_INTERVAL;
                car.setX(new_x);
                animator = ObjectAnimator.ofFloat(car, "x", new_x);
                animator.setDuration((long) noOfGrids * ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            default:
                break;
        }
    }

    public void leftCommand() {
        int orientation = (int) car.getRotation();
        switch (((orientation / 90) % 4 + 4) % 4) {
            case 0:
                car.setRotation(270);
                break;
            case 1:
                car.setRotation(0);
                break;
            case 2:
                car.setRotation(90);
                break;
            case 3:
                car.setRotation(180);
                break;
            default:
                break;
        }

        updateXYDirText();
    }

    private void rightCommand() {
        int orientation = (int) car.getRotation();
        switch (((orientation / 90) % 4 + 4) % 4) {
            case 0:
                car.setRotation(90);
                break;
            case 1:
                car.setRotation(180);
                break;
            case 2:
                car.setRotation(270);
                break;
            case 3:
                car.setRotation(0);
                break;
            default:
                break;
        }

        updateXYDirText();
    }

    private void stopTimerButton() {
        Chronometer IRTimer = findViewById(R.id.timer);
        IRTimer.stop();
        updateStatusWindow("Ready");
    }

    private void toggleTimerBtn() {
        if (timerMode.equals("STOP")) {
            beginStart();
            timerMode = "START";
            timerButton.setText("STOP");
        } else if (timerMode.equals("START")) {
            stopTimerButton();
            timerMode = "STOP";
            timerButton.setText("START");
        }
    }

    private void beginStart() {
        if (BluetoothService.BluetoothConnectionStatus) {
            JSONObject jsonStartCommand = new JSONObject();
            try {
                jsonStartCommand.put("cat", "control");
                jsonStartCommand.put("value", "start");
            } catch (JSONException e) {
                e.printStackTrace();
            }

            String formattedStartCommand = jsonStartCommand.toString();
            byte[] bytes = formattedStartCommand.getBytes(Charset.defaultCharset());
            BluetoothService.write(bytes);
            updateStatusWindow("Running");
        } else {
            updateStatusWindow("Not Ready");
            Toast.makeText(Arena.this, "Please connect to Bluetooth.", Toast.LENGTH_SHORT).show();
            sleepFor(ANIMATOR_DURATION);
            updateStatusWindow("Ready");
            return;
        }

        Chronometer IRTimer = findViewById(R.id.timer);
        long elapsedRealtime = SystemClock.elapsedRealtime();
        IRTimer.setBase(elapsedRealtime);
        IRTimer.start();
    }

    private void beginIRTask() {
        String IRstart = "ALG:START";

        if (BluetoothService.BluetoothConnectionStatus) {
            byte[] bytes = IRstart.getBytes(Charset.defaultCharset());
            BluetoothService.write(bytes);
            Toast.makeText(Arena.this, "Obstacles sent", Toast.LENGTH_SHORT).show();
            updateStatusWindow("IR Started");
        } else {
            updateStatusWindow("IR Not Started");
            Toast.makeText(Arena.this, "Please connect to Bluetooth.", Toast.LENGTH_SHORT).show();
            sleepFor(ANIMATOR_DURATION);
            updateStatusWindow("Ready");
            return;
        }
    }

    private void beginSPTask() {
        String SPstart = "STM:START";

        if (BluetoothService.BluetoothConnectionStatus) {
            byte[] bytes = SPstart.getBytes(Charset.defaultCharset());
            BluetoothService.write(bytes);
            Toast.makeText(Arena.this, "Shortest Path Started.", Toast.LENGTH_SHORT).show();
            updateStatusWindow("SP Started");
        } else {
            updateStatusWindow("SP Not Started");
            Toast.makeText(Arena.this, "Please connect to Bluetooth.", Toast.LENGTH_SHORT).show();
            sleepFor(ANIMATOR_DURATION);
            updateStatusWindow("Ready");
            return;
        }
    }

    private void setResetButton() {
        // Reset Timer
        Chronometer IRTimer = findViewById(R.id.timer);
        IRTimer.setBase(SystemClock.elapsedRealtime());
        IRTimer.stop();
        updateStatusWindow("Ready");

        obstacle1.setTranslationX(0);
        obstacle1.setTranslationY(0);

        obstacle2.setTranslationX(0);
        obstacle2.setTranslationY(0);

        obstacle3.setTranslationX(0);
        obstacle3.setTranslationY(0);

        obstacle4.setTranslationX(0);
        obstacle4.setTranslationY(0);

        obstacle5.setTranslationX(0);
        obstacle5.setTranslationY(0);

        obstacle6.setTranslationX(0);
        obstacle6.setTranslationY(0);

        obstacle7.setTranslationX(0);
        obstacle7.setTranslationY(0);

        obstacle8.setTranslationX(0);
        obstacle8.setTranslationY(0);

        car.setX(INITIAL_X);
        car.setY(INITIAL_Y);
        car.setRotation(0);
        updateXYDirText();

        obstacle1.setImageResource(Helper.resources.get("o1n"));
        obstacle1.setTag(Helper.resources.get("o1n"));
        obstacle2.setImageResource(Helper.resources.get("o2n"));
        obstacle2.setTag(Helper.resources.get("o2n"));
        obstacle3.setImageResource(Helper.resources.get("o3n"));
        obstacle3.setTag(Helper.resources.get("o3n"));
        obstacle4.setImageResource(Helper.resources.get("o4n"));
        obstacle4.setTag(Helper.resources.get("o4n"));
        obstacle5.setImageResource(Helper.resources.get("o5n"));
        obstacle5.setTag(Helper.resources.get("o5n"));
        obstacle6.setImageResource(Helper.resources.get("o6n"));
        obstacle6.setTag(Helper.resources.get("o6n"));
        obstacle7.setImageResource(Helper.resources.get("o7n"));
        obstacle7.setTag(Helper.resources.get("o7n"));
        obstacle8.setImageResource(Helper.resources.get("o8n"));
        obstacle1.setTag(Helper.resources.get("o8n"));

        obstacle1.setRotation(0);
        obstacle2.setRotation(0);
        obstacle3.setRotation(0);
        obstacle4.setRotation(0);
        obstacle5.setRotation(0);
        obstacle6.setRotation(0);
        obstacle7.setRotation(0);
        obstacle8.setRotation(0);
    }

    private void setPreset1Button() {
        updateStatusWindow("Ready");

        /*
        * 1 Grid 35
        * Count from Top to Bottom
        * Count from Left to Right
        *
        * North = 0
        * East = 90
        * South = 180
        * West = 270
        * */

        obstacle1.setX(35);
        obstacle1.setY(105);
        obstacle1.setRotation(90);
        obstacle1.setImageResource(Helper.resources.get("o1e"));

        obstacle2.setX(175);
        obstacle2.setY(245);
        obstacle2.setRotation(180);
        obstacle2.setImageResource(Helper.resources.get("o2s"));

        obstacle3.setX(280);
        obstacle3.setY(490);
        obstacle3.setRotation(0);
        obstacle3.setImageResource(Helper.resources.get("o3n"));

        obstacle4.setX(385);
        obstacle4.setY(175);
        obstacle4.setRotation(90);
        obstacle4.setImageResource(Helper.resources.get("o4e"));

        obstacle5.setX(525);
        obstacle5.setY(595);
        obstacle5.setRotation(270);
        obstacle5.setImageResource(Helper.resources.get("o5w"));

        obstacle6.setX(560);
        obstacle6.setY(0);
        obstacle6.setRotation(180);
        obstacle6.setImageResource(Helper.resources.get("o6s"));

        obstacle7.setX(665);
        obstacle7.setY(350);
        obstacle7.setRotation(270);
        obstacle7.setImageResource(Helper.resources.get("o7w"));

        //obstacle8.setX(0);
        //obstacle8.setY(0);
        //obstacle8.setRotation(0);
        //obstacle8.setImageResource(Helper.resources.get("o8e"));
    }

    private void toggleSetMode() {
        canSetObstacles = !canSetObstacles;
        if (curMode.equals("IDLE")) {
            curMode = "SET";
            setButton.setText("Done");
        } else if (curMode.equals("SET")) {
            curMode = "IDLE";
            setButton.setText("Set");
        }
    }

    private void sendObstacles() {

        JSONObject obstaclesJson = new JSONObject();
        try {
            JSONArray obstacleArray = new JSONArray();

            JSONObject obstacle1Json = getObstacleJSON(obstacle1, 0);
            if (obstacle1Json.length() > 0) {
                obstacleArray.put(obstacle1Json);
            }

            JSONObject obstacle2Json = getObstacleJSON(obstacle2, 1);
            if (obstacle2Json.length() > 0) {
                obstacleArray.put(obstacle2Json);
            }

            JSONObject obstacle3Json = getObstacleJSON(obstacle3, 2);
            if (obstacle3Json.length() > 0) {
                obstacleArray.put(obstacle3Json);
            }

            JSONObject obstacle4Json = getObstacleJSON(obstacle4, 3);
            if (obstacle4Json.length() > 0) {
                obstacleArray.put(obstacle4Json);
            }

            JSONObject obstacle5Json = getObstacleJSON(obstacle5, 4);
            if (obstacle5Json.length() > 0) {
                obstacleArray.put(obstacle5Json);
            }

            JSONObject obstacle6Json = getObstacleJSON(obstacle6, 5);
            if (obstacle6Json.length() > 0) {
                obstacleArray.put(obstacle6Json);
            }

            JSONObject obstacle7Json = getObstacleJSON(obstacle7, 6);
            if (obstacle7Json.length() > 0) {
                obstacleArray.put(obstacle7Json);
            }

            JSONObject obstacle8Json = getObstacleJSON(obstacle8, 7);
            if (obstacle8Json.length() > 0) {
                obstacleArray.put(obstacle8Json);
            }

            obstaclesJson.put("obstacles", obstacleArray);
            obstaclesJson.put("mode", "0");
        } catch (JSONException e) {
            e.printStackTrace();
        }

        if (BluetoothService.BluetoothConnectionStatus) {

            JSONObject fullObstacleJson = new JSONObject();
            try {
                fullObstacleJson.put("cat", "obstacles");
                fullObstacleJson.put("value", obstaclesJson);
            } catch (JSONException e) {
                e.printStackTrace();;
            }

            String formattedObstacle = fullObstacleJson.toString();
            byte[] bytes = formattedObstacle.getBytes(Charset.defaultCharset());
            BluetoothService.write(bytes);
            Toast.makeText(Arena.this, "Obstacles sent", Toast.LENGTH_SHORT).show();
        }
    }

    private String getObstacleLocation(ImageView obstacle) {
        return (int) obstacle.getX() + "," + (int) obstacle.getY() + "," + getImageOrientation(obstacle);
    }

    private JSONObject getObstacleJSON(ImageView obstacle, int id) {
        JSONObject obstacleJson = new JSONObject();
        try {
            int x = (int) obstacle.getX() / SNAP_GRID_INTERVAL;
            int y = (int) obstacle.getY() / SNAP_GRID_INTERVAL;
            // (0,0) starts from top left hence invert y
            int new_y = 20 - y - 1;
            Log.d(TAG, "Obstacle at " + x + "," + new_y);

            if (x <= 19 && new_y <= 19){
                obstacleJson.put("x", x);
                obstacleJson.put("y", new_y);
                obstacleJson.put("id", id);
                obstacleJson.put("d", getImageOrientation(obstacle));
            }
        } catch (JSONException e) {
            e.printStackTrace();
        }

        return obstacleJson;
    }

    private int getImageOrientation(ImageView obstacle) {
        switch (((int) ((obstacle.getRotation() / 90) % 4 + 4) % 4)) {
            case 0:
                return 0;   //North
            case 1:
                return 2;   //East
            case 2:
                return 4;   //South
            case 3:
                return 6;   //West
            default:
                return 1000;
        }
    }

    private void updateStatusWindow(String msg) {
        statusWindow.setText(msg);
        Log.d(TAG, "Status window: " + msg);
    }

    private void updateXYDirText() {
        int x = (int) (car.getX() + SNAP_GRID_INTERVAL) / SNAP_GRID_INTERVAL;
        int y = (int) (car.getY() + SNAP_GRID_INTERVAL) / SNAP_GRID_INTERVAL;
        // (0,0) starts from top left hence invert y
        int new_y = 20 - y - 1;
        car_x.setText(String.valueOf(x));
        car_y.setText(String.valueOf(new_y));

        int direction = (int) car.getRotation();

        if (direction == 315)
            car_dir.setText("North-West");
        else if (direction == 0)
            car_dir.setText("North");
        else if (direction == 45)
            car_dir.setText("North-East");
        else if (direction == 90)
            car_dir.setText("East");
        else if (direction == 135)
            car_dir.setText("South-East");
        else if (direction == 180)
            car_dir.setText("South");
        else if (direction == 225)
            car_dir.setText("South-West");
        else if (direction == 270)
            car_dir.setText("West");
        else
            car_dir.setText("None");
    }

    // Broadcast Receiver for incoming string messages
    BroadcastReceiver myReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String message = intent.getStringExtra("receivedString");
            String JSONmessage = intent.getStringExtra("receivedJSON");
            Log.d(TAG, "Broadcast Receiver String message: " + message);
            Log.d(TAG, "Broadcast Receiver JSON message: " + JSONmessage);
            // Convert string json into json object
            JSONObject jsonObject = null;
            try {
                if (JSONmessage != null)
                    jsonObject = new JSONObject(JSONmessage);

            } catch (JSONException e) {
                Log.d(TAG, "Exception: " + e.getMessage());
            }
            String command;

            try {
                Log.d("Enter Try Loop","Enter Try Loop");

                //JSON format: {"cat": "status", "value": 'finished"}
                if (jsonObject.has("cat") && jsonObject.getString("cat").equals("status")) {
                    if (jsonObject.has("value") && jsonObject.getString("value").equals("finished")) {
                        stopTimerButton();
                        timerMode = "STOP";
                        timerButton.setText("START");
                    }
                }

                // ########EXTRACT TARGET - CHANGE THE IMAGE ID TO OBSTACLE ID#######################
                // JSON format: {'image_id': '36', 'obstacle_id': '0'}
                //{"cat": "image-rec", "value": {"image_id": "21", "obstacle_id": "4"}}
                if (jsonObject.has("cat") && jsonObject.getString("cat").equals("image-rec")) {
                    // Extract image_id from JSON
                    JSONObject valueObject = jsonObject.getJSONObject("value");

                    String image_id = valueObject.getString("image_id");
                    int obstacle_id = valueObject.getInt("obstacle_id");

                    // Debug statements
                    Log.d(TAG, "image_id value = " + image_id);
                    Log.d(TAG, "obstacle_id value = " + obstacle_id);

                    // Update Character Value
                    if (Integer.parseInt(image_id) == 0) {
                        Toast.makeText(Arena.this, "Image not recognized, trying again", Toast.LENGTH_SHORT).show();
                    } else {
                        // RMB TO PLUS 1 !!
                        setObstacleImage(obstacle_id + 1, image_id);
                        Toast.makeText(Arena.this,
                                "Obstacle " + image_id + " changed to Target ID: " + obstacle_id,
                                Toast.LENGTH_SHORT).show();
                    }
                }

                if(jsonObject.has("value")){
                    jsonObject = new JSONObject(JSONmessage);
                    JSONObject valueObject = jsonObject.getJSONObject("value");


                    Handler handler = new Handler();
                    int prev_x = 0;
                    int prev_y = 0;

                        int x = valueObject.getInt("x");
                        int y = valueObject.getInt("y");
                        int d = valueObject.getInt("d");

                        // Log x, y, and d values
                        Log.d(TAG, "Path: x=" + x + ", y=" + y + ", d=" + d);


                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                Log.d("ROUTE MAPPING ACTIVATED","ROUTE MAPPING ACTIVATED");
                                canvasGrid = findViewById(R.id.canvasGrid);
                                if (canvasGrid == null) {
                                    Log.e(TAG, "CanvasGrid is null. Check if R.id.canvasGrid is correct in your layout XML.");
                                    return; // Handle the null case appropriately
                                }

                                // Update car location
                                int new_x = (int) x * SNAP_GRID_INTERVAL - SNAP_GRID_INTERVAL;
                                int new_y = (int) 18 * SNAP_GRID_INTERVAL - SNAP_GRID_INTERVAL - (y-1) * SNAP_GRID_INTERVAL;
                                int prev_x = (int) car.getX();
                                int prev_y = (int) car.getY();

                                if(d == 0){
                                    car.setRotation(0);
                                    car_dir.setText("North");
                                }
                                else if(d == 2){
                                    car.setRotation(90);
                                    car_dir.setText("East");
                                }
                                else if(d == 4){
                                    car.setRotation(180);
                                    car_dir.setText("South");
                                }
                                else if(d == 6){
                                    car.setRotation(270);
                                    car_dir.setText("West");
                                }

                                // Animate car movement
                                ObjectAnimator xAnimator = ObjectAnimator.ofFloat(car, "x", prev_x, new_x);
                                ObjectAnimator yAnimator = ObjectAnimator.ofFloat(car, "y", prev_y, new_y);
                                AnimatorSet animatorSet = new AnimatorSet();
                                animatorSet.playTogether(xAnimator, yAnimator);
                                animatorSet.setDuration(2000); // Set duration in milliseconds (adjust as needed)
                                animatorSet.start();

                                updateXYDirText();

                                // Update grid coordinate
                                canvasGrid.setColorForGrid(y, x, Color.parseColor("#FF0000"));
                                canvasGrid.postInvalidate();

                            }
                        }, 3000);

                }


            } catch (Exception e) {
                Log.d(TAG, "Exception: " + e.getMessage());
                return;
            }

            try {
                command = message.substring(0, message.indexOf(','));
            } catch (IndexOutOfBoundsException e) {
                // Toast.makeText(Arena.this, "Invalid message format!",
                // Toast.LENGTH_SHORT).show();
                return;
            } catch (Exception e) {
                Log.d(TAG, "Exception: " + e.getMessage());
                return;
            }
        }
    };
}
