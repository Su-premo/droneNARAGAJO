package com.viasofts.mygcs;

import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.graphics.PointF;
import android.net.Uri;
import android.os.Handler;
import android.os.Message;
import android.support.annotation.NonNull;
import android.support.v4.app.FragmentManager;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import com.naver.maps.geometry.LatLng;
import com.naver.maps.geometry.LatLngBounds;
import com.naver.maps.map.CameraUpdate;
import com.naver.maps.map.LocationTrackingMode;
import com.naver.maps.map.MapFragment;
import com.naver.maps.map.NaverMap;
import com.naver.maps.map.OnMapReadyCallback;
import com.naver.maps.map.UiSettings;
import com.naver.maps.map.overlay.LocationOverlay;
import com.naver.maps.map.overlay.Marker;
import com.naver.maps.map.overlay.OverlayImage;
import com.naver.maps.map.overlay.PolygonOverlay;
import com.naver.maps.map.overlay.PolylineOverlay;
import com.naver.maps.map.util.FusedLocationSource;
import com.o3dr.android.client.ControlTower;
import com.o3dr.android.client.Drone;
import com.o3dr.android.client.apis.ControlApi;
import com.o3dr.android.client.apis.MissionApi;
import com.o3dr.android.client.apis.VehicleApi;
import com.o3dr.android.client.interfaces.DroneListener;
import com.o3dr.android.client.interfaces.LinkListener;
import com.o3dr.android.client.interfaces.TowerListener;
import com.o3dr.services.android.lib.coordinate.LatLong;
import com.o3dr.services.android.lib.coordinate.LatLongAlt;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.companion.solo.SoloAttributes;
import com.o3dr.services.android.lib.drone.companion.solo.SoloState;
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter;
import com.o3dr.services.android.lib.drone.connection.ConnectionType;
import com.o3dr.services.android.lib.drone.mission.Mission;
import com.o3dr.services.android.lib.drone.mission.item.spatial.Waypoint;
import com.o3dr.services.android.lib.drone.property.Altitude;
import com.o3dr.services.android.lib.drone.property.Attitude;
import com.o3dr.services.android.lib.drone.property.Battery;
import com.o3dr.services.android.lib.drone.property.Gps;
import com.o3dr.services.android.lib.drone.property.Speed;
import com.o3dr.services.android.lib.drone.property.State;
import com.o3dr.services.android.lib.drone.property.Type;
import com.o3dr.services.android.lib.drone.property.VehicleMode;
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus;
import com.o3dr.services.android.lib.model.AbstractCommandListener;
import com.o3dr.services.android.lib.model.SimpleCommandListener;
import com.o3dr.services.android.lib.util.MathUtils;
import com.viasofts.mygcs.activities.helpers.BluetoothDevicesActivity;
import com.viasofts.mygcs.utils.TLogUtils;
import com.viasofts.mygcs.utils.prefs.DroidPlannerPrefs;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;


public class MainActivity extends AppCompatActivity implements DroneListener, TowerListener, LinkListener, OnMapReadyCallback {

    private static final String TAG = MainActivity.class.getSimpleName();

    private Drone drone;
    private int droneType = Type.TYPE_UNKNOWN;
    private ControlTower controlTower;
    private final Handler handler = new Handler();

    private static final int DEFAULT_UDP_PORT = 14550;
    private static final int DEFAULT_USB_BAUD_RATE = 57600;

    private Spinner modeSelector;

    Handler mainHandler;

    private static final int LOCATION_PERMISSION_REQUEST_CODE = 1000;
    private FusedLocationSource locationSource;
    private NaverMap naverMap;

    private Button btnConnect;
    private Button mBtnSetConnectionType;
    private DroidPlannerPrefs mPrefs;
    private static final long EVENTS_DISPATCHING_PERIOD = 200L; //MS


    private TextView speedTextView;
    private TextView altitudeTextView;
    private TextView voltageTextView;
    private TextView yawTextView;
    private TextView satelliteCountsTextView;
    private Marker markerDroneLocation = new Marker();

    private Button armButton;
    private Button btnControlAltitude;
    private Button btnRaiseAltitude;
    private Button btnLowerAltitude;
    private Double mTakeoffAltitude = 5.0;

    private Marker markerGuideModeLC = new Marker();
    private LatLng guidedPoint;
    private boolean checkGuideRun = false;

    private ArrayList<LatLng> polylineDroneAL = new ArrayList<>();
    private Button mapButton;
    private Button btnMapBasic;
    private Button btnMapSatellite;
    private Button btnMapTerrain;
    private Button btnMapCadastral;
    private Button btnMapLock;
    private Timer timerMapLock;
    private TimerTask timerTaskMapLock;
    private Button resetOverlaysBtn;
    private ArrayList<String> recyclerDataAL = new ArrayList<>();

    private Button btnPathDistance;
    private Button btnLengthenPathDistance;
    private Button btnShortenPathDistance;
    private Button btnPathWidth;
    private Button btnWidenPathWidth;
    private Button btnNarrowPathWidthDown;
    private Double mPathWidth = 5.0;
    private Double mPathDistance = 50.0;

    private Button btnMission;
    private Button btnMissionAB;
    private Button btnMissionUndo;
    private Marker markerA = new Marker();
    private Marker markerB = new Marker();
    private Boolean missionABflag = false;
    private Boolean aFlag = false;
    private Boolean bFlag = false;
    private PolylineOverlay polylineABmission = new PolylineOverlay();

    private MathUtils mathUtils = new MathUtils();

    private Button btnSetMission;
    private Mission mission = new Mission();
    private Boolean missionSentFlag = false;
    private Boolean missionStartFlag = false;

    // return home
    private LatLong homePositionG;
    private LatLng homePositionN;
    private Boolean checkReturnHome = false;
    private Marker homeMarker = new Marker();
    private ArrayList<LatLng> ABmissionArr = new ArrayList<>();


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        setContentView(R.layout.activity_main);
        locationSource = new FusedLocationSource(this, LOCATION_PERMISSION_REQUEST_CODE);

        setView();

        FragmentManager fm = getSupportFragmentManager();
        MapFragment mapFragment = (MapFragment) fm.findFragmentById(R.id.map);
        if (mapFragment == null) {
            mapFragment = MapFragment.newInstance();
            fm.beginTransaction().add(R.id.map, mapFragment).commit();
        }

        mapFragment.getMapAsync(this);

        final Context context = getApplicationContext();
        this.controlTower = new ControlTower(context);
        this.drone = new Drone(context);

        // Spinner modeSelector
        this.modeSelector.setOnItemSelectedListener(new Spinner.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
//                ((TextView) parent.getChildAt(0)).setTextColor(Color.WHITE);
                onFlightModeSelected(view);
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // Do nothing
            }
        });

        mainHandler = new Handler(getApplicationContext().getMainLooper());

        // test
        SharedPreferences sharedPref = getPreferences(getApplicationContext().MODE_PRIVATE);
        SharedPrefManager.getInstance().init(sharedPref);
        mPrefs = DroidPlannerPrefs.getInstance(getApplicationContext());

        mBtnSetConnectionType = findViewById(R.id.button_set_conn_type);
        final String[] types = getResources().getStringArray(R.array.connection_type);
        mBtnSetConnectionType.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                builder
                        .setTitle("SET CONNECTION TYPE")
                        .setItems(R.array.connection_type, new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int id) {
                                String value = types[id];
                                SharedPrefManager.save(SharedPrefManager.KEY_CONNECTION, value);
                                mBtnSetConnectionType.setText(value);
                            }
                        });

                final AlertDialog alert = builder.create();
                alert.show();
            }
        });

        String connectionType = SharedPrefManager.read(SharedPrefManager.KEY_CONNECTION);
        if (connectionType.equals("")) {
            connectionType = types[0];
        }

        mBtnSetConnectionType.setText(connectionType);

    }

    private void setView() {
        btnConnect = (Button) findViewById(R.id.btnConnect);

        speedTextView = (TextView) findViewById(R.id.speedValueTextView);
        altitudeTextView = (TextView) findViewById(R.id.altitudeValueTextView);
        voltageTextView = (TextView) findViewById(R.id.voltageValueTextView);
        yawTextView = (TextView) findViewById(R.id.yawValueTextView);
        satelliteCountsTextView = (TextView) findViewById(R.id.satelliteValueTextView);

        modeSelector = (Spinner) findViewById(R.id.modeSelect);
        armButton = (Button) findViewById(R.id.btnArm);
        btnControlAltitude = (Button) findViewById(R.id.btnControlAltitude);
        btnRaiseAltitude = (Button) findViewById(R.id.btnRaiseAltitude);
        btnLowerAltitude = (Button) findViewById(R.id.btnLowerAltitude);

        resetOverlaysBtn = (Button) findViewById(R.id.btn_resetOverlays);

        btnPathDistance = (Button) findViewById(R.id.btnPathDistance);
        btnLengthenPathDistance = (Button) findViewById(R.id.btnUpPathDistance);
        btnShortenPathDistance = (Button) findViewById(R.id.btnDownPathDistance);
        btnPathWidth = (Button) findViewById(R.id.btnPathWidth);
        btnWidenPathWidth = (Button) findViewById(R.id.btnUpPathWidth);
        btnNarrowPathWidthDown = (Button) findViewById(R.id.btnDownPathWidth);
        btnMission = (Button) findViewById(R.id.btnMission);
        btnMissionAB = (Button) findViewById(R.id.btnAB);
        btnMissionUndo = (Button) findViewById(R.id.btnMissionUndo);
        btnSetMission = (Button) findViewById(R.id.btnSetMission);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions, @NonNull int[] grantResults) {
        if (locationSource.onRequestPermissionsResult(
                requestCode, permissions, grantResults)) {
            if (!locationSource.isActivated()) { // ?????? ?????????
                naverMap.setLocationTrackingMode(LocationTrackingMode.None);
            }
            return;
        }
        super.onRequestPermissionsResult(
                requestCode, permissions, grantResults);
    }


    @Override
    public void onStart() {
        super.onStart();
        this.controlTower.connect(this);
        updateVehicleModesForType(this.droneType);
    }

    @Override
    public void onStop() {
        super.onStop();
        if (this.drone.isConnected()) {
            this.drone.disconnect();
            updateConnectedButton(false);
        }

        this.controlTower.unregisterDrone(this.drone);
        this.controlTower.disconnect();
    }


    @Override
    public void onDroneEvent(String event, Bundle extras) {
        Log.d("eventLog", event);
        switch (event) {
            case AttributeEvent.STATE_CONNECTED:
                alertUser("Drone Connected");
                updateConnectedButton(this.drone.isConnected());
                updateArmButton();

//                checkSoloState();

                break;

            case AttributeEvent.STATE_DISCONNECTED:
                alertUser("Drone Disconnected");
                updateConnectedButton(this.drone.isConnected());
                updateArmButton();

                break;

            case AttributeEvent.STATE_UPDATED:
            case AttributeEvent.STATE_ARMING:
                updateArmButton();
                break;

            case AttributeEvent.TYPE_UPDATED:
                Type newDroneType = this.drone.getAttribute(AttributeType.TYPE);
                if (newDroneType.getDroneType() != this.droneType) {
                    this.droneType = newDroneType.getDroneType();
                    updateVehicleModesForType(this.droneType);
                }
                break;

            case AttributeEvent.BATTERY_UPDATED:
                updateBatteryVoltage();
                break;

            case AttributeEvent.STATE_VEHICLE_MODE:
                updateVehicleMode();
                break;

            case AttributeEvent.SPEED_UPDATED:
                updateSpeed();
                break;

            case AttributeEvent.ALTITUDE_UPDATED:
                updateAltitude();
                break;

            case AttributeEvent.ATTITUDE_UPDATED:
                updateYaw();
                break;

            case AttributeEvent.GPS_COUNT:
                updateSatelliteCounts();
                break;

            case AttributeEvent.GPS_POSITION:
                updateDroneGPS();

                State vehicleState = this.drone.getAttribute(AttributeType.STATE);
                VehicleMode vehicleMode = vehicleState.getVehicleMode();

                if (vehicleMode == vehicleMode.COPTER_GUIDED && checkGuideRun) {
                    if (checkGoal() == true) {
                        alertUser("Drone reached the guided point");
                        changeToLoiterMode();
                        markerGuideModeLC.setMap(null);
                        checkGuideRun = false;
                    }
                }
                if (vehicleMode == vehicleMode.COPTER_GUIDED && checkReturnHome) {
                    if (checkBackHome() == true) {
                        changeToLandMode(); // land
                        alertUser("drone's at home position");
                        checkReturnHome = false;
                        homeMarker.setMap(null);
                        Gps droneGps = this.drone.getAttribute(AttributeType.GPS);
                        LatLong vehiclePosition = droneGps.getPosition();
                        Log.d("returnedHome", vehiclePosition.toString());
                    }
                }
                break;


            default:
                // Log.i("DRONE_EVENT", event); //Uncomment to see events from the drone
                break;
        }
    }


    private void onFlightModeSelected(View view) {
        VehicleMode vehicleMode = (VehicleMode) this.modeSelector.getSelectedItem();

        VehicleApi.getApi(this.drone).setVehicleMode(vehicleMode, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                alertUser("Vehicle mode change successful.");
            }

            @Override
            public void onError(int executionError) {
                alertUser("Vehicle mode change failed: " + executionError);
            }

            @Override
            public void onTimeout() {
                alertUser("Vehicle mode change timed out.");
            }
        });
    }

    // Connect Button
    public void connectBtn(View view) {
        if (drone.isConnected()) {
            drone.disconnect();
        } else {
            //drone.connect(ConnectionParameter.newUdpConnection(null));
            drone.connect(retrieveConnectionParameters());
        }
    }

    // txt switched on connect button when button clicked
    private void updateConnectedButton(boolean isConnected) {
        if (isConnected) {
            btnConnect.setText("Disconnect");
        } else {
            btnConnect.setText("Connect");
            mBtnSetConnectionType.setText("Connect Type");
        }
    }


    private ConnectionParameter retrieveConnectionParameters() {
        final @ConnectionType.Type int connectionType = mPrefs.getConnectionParameterType();

        // Generate the uri for logging the tlog data for this session.
        Uri tlogLoggingUri = TLogUtils.getTLogLoggingUri(getApplicationContext(),
                connectionType, System.currentTimeMillis());

        int idx = getConnectionTypeIdx();

        ConnectionParameter connParams;
        //switch (connectionType) {
        //switch (ConnectionType.TYPE_BLUETOOTH) {
        switch (idx) {
            case ConnectionType.TYPE_USB:
                connParams = ConnectionParameter.newUsbConnection(mPrefs.getUsbBaudRate(),
                        tlogLoggingUri, EVENTS_DISPATCHING_PERIOD);
                break;

            case ConnectionType.TYPE_UDP:
                if (mPrefs.isUdpPingEnabled()) {
                    connParams = ConnectionParameter.newUdpWithPingConnection(
                            mPrefs.getUdpServerPort(),
                            mPrefs.getUdpPingReceiverIp(),
                            mPrefs.getUdpPingReceiverPort(),
                            "Hello".getBytes(),
                            ConnectionType.DEFAULT_UDP_PING_PERIOD,
                            tlogLoggingUri,
                            EVENTS_DISPATCHING_PERIOD);
                } else {
                    connParams = ConnectionParameter.newUdpConnection(mPrefs.getUdpServerPort(),
                            tlogLoggingUri, EVENTS_DISPATCHING_PERIOD);
                }
                break;

            case ConnectionType.TYPE_TCP:
                connParams = ConnectionParameter.newTcpConnection(mPrefs.getTcpServerIp(),
                        mPrefs.getTcpServerPort(), tlogLoggingUri, EVENTS_DISPATCHING_PERIOD);
                break;

            case ConnectionType.TYPE_BLUETOOTH:
                String btAddress = mPrefs.getBluetoothDeviceAddress();

                if (TextUtils.isEmpty(btAddress)) {
                    connParams = null;
                    startActivity(new Intent(getApplicationContext(), BluetoothDevicesActivity.class).addFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                } else {
                    connParams = ConnectionParameter.newBluetoothConnection(btAddress,
                            tlogLoggingUri, EVENTS_DISPATCHING_PERIOD);
                }
                break;

            default:
                Log.e("myLog", "Unrecognized connection type: " + connectionType);
                connParams = null;
                break;
        }

        return connParams;
    }

    public int getConnectionTypeIdx() {
        final String[] types = getResources().getStringArray(R.array.connection_type);
        String strConn = mBtnSetConnectionType.getText().toString();

        int idx = 3;
        for (int i = 0; i < types.length; i++) {
            if (types[i].equals(strConn)) {
                return i;
            }
        }

        return idx;
    }

    private void checkSoloState() {
        final SoloState soloState = drone.getAttribute(SoloAttributes.SOLO_STATE);
        if (soloState == null) {
            alertUser("Unable to retrieve the solo state.");
        } else {
            alertUser("Solo state is up to date.");
        }
    }

    // Arm Button
    public void onArmButtonTap(View view) {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        if (vehicleState.isFlying()) {
            // Land
            VehicleApi.getApi(this.drone).setVehicleMode(VehicleMode.COPTER_LAND, new SimpleCommandListener() {
                @Override
                public void onError(int executionError) {
                    alertUser("Unable to land the vehicle.");
                }

                @Override
                public void onTimeout() {
                    alertUser("Landing timed out.");
                }
            });
        } else if (vehicleState.isArmed()) {
            AlertDialog.Builder takeOffAlertBuilder = new AlertDialog.Builder(MainActivity.this);
            takeOffAlertBuilder.setMessage("????????? ?????????????????? ????????? ???????????????.\n??????????????? ???????????????.")
                    .setPositiveButton("??????", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            // Take off
                            ControlApi.getApi(drone).takeoff(mTakeoffAltitude, new AbstractCommandListener() {

                                @Override
                                public void onSuccess() {
                                    alertUser("Taking off...");
                                }

                                @Override
                                public void onError(int i) {
                                    alertUser("Unable to take off.");
                                }

                                @Override
                                public void onTimeout() {
                                    alertUser("Taking off timed out.");
                                }
                            });
                        }
                    })
                    .setNegativeButton("??????", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            dialog.cancel();
                        }
                    });
            takeOffAlertBuilder.show();
        } else if (!vehicleState.isConnected()) {
            // Connect
            alertUser("Connect to a drone first");
        } else {
            AlertDialog.Builder armAlertBuilder = new AlertDialog.Builder(MainActivity.this);
            armAlertBuilder.setMessage("????????? ???????????????.\n????????? ???????????? ???????????????.")
                    .setPositiveButton("??????", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            // Connected but not Armed
                            VehicleApi.getApi(drone).arm(true, false, new SimpleCommandListener() {
                                @Override
                                public void onError(int executionError) {
                                    alertUser("Unable to arm vehicle.");
                                }

                                @Override
                                public void onTimeout() {
                                    alertUser("Arming operation timed out.");
                                }
                            });
                        }
                    })
                    .setNegativeButton("??????", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            dialog.cancel();
                        }
                    });
            armAlertBuilder.show();
        }
    }

    // Altitude buttons = get altitude value + raise/lower button
    public void controlAltitude(View view) {
        if (btnRaiseAltitude.getVisibility() == View.INVISIBLE && btnLowerAltitude.getVisibility() == View.INVISIBLE) {
            btnRaiseAltitude.setVisibility(View.VISIBLE);
            btnLowerAltitude.setVisibility(View.VISIBLE);
        } else {
            btnRaiseAltitude.setVisibility(View.INVISIBLE);
            btnLowerAltitude.setVisibility(View.INVISIBLE);
        }
    }

    // raise altitude +0.5
    public void raiseAltitude(View view) {
        if (mTakeoffAltitude < 10) {
            mTakeoffAltitude = mTakeoffAltitude + 0.5;
            btnControlAltitude.setText(String.valueOf(mTakeoffAltitude) + "m\n????????????");
        }
    }

    // lower altitude -0.5
    public void lowerAltitude(View view) {
        if (mTakeoffAltitude > 2) {
            mTakeoffAltitude = mTakeoffAltitude - 0.5;
            btnControlAltitude.setText(String.valueOf(mTakeoffAltitude) + "m\n????????????");
        }
    }

    protected void updateArmButton() {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);

        if (vehicleState.isFlying()) {
            // Land
            armButton.setText("LAND");
        } else if (vehicleState.isArmed()) {
            // Take off
            armButton.setText("TAKE OFF");
        } else if (vehicleState.isConnected()) {
            // Connected but not Armed
            armButton.setText("ARM");
        }
    }


    protected void updateBatteryVoltage() {
        Battery droneVoltage = this.drone.getAttribute(AttributeType.BATTERY);
        voltageTextView.setText(String.format("%3.1f", droneVoltage.getBatteryVoltage()) + "V");
    }

    protected void updateVehicleModesForType(int droneType) {
        List<VehicleMode> vehicleModes = VehicleMode.getVehicleModePerDroneType(droneType);
        ArrayAdapter<VehicleMode> vehicleModeArrayAdapter = new ArrayAdapter<VehicleMode>(this, android.R.layout.simple_spinner_item, vehicleModes);
        vehicleModeArrayAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        this.modeSelector.setAdapter(vehicleModeArrayAdapter);
    }

    protected void updateVehicleMode() {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        VehicleMode vehicleMode = vehicleState.getVehicleMode();
        ArrayAdapter arrayAdapter = (ArrayAdapter) this.modeSelector.getAdapter();
        this.modeSelector.setSelection(arrayAdapter.getPosition(vehicleMode));
    }

    protected void updateSpeed() {
        Speed droneSpeed = this.drone.getAttribute(AttributeType.SPEED);
        speedTextView.setText(String.format("%3.1f", droneSpeed.getGroundSpeed()) + "m/s");
    }

    protected void updateAltitude() {
        Altitude droneAltitude = this.drone.getAttribute(AttributeType.ALTITUDE);
        altitudeTextView.setText(String.format("%3.1f", droneAltitude.getAltitude()) + "m");
    }

    protected void updateYaw() {
//        YawCondition droneYaw = this.drone.getAttribute(AttributeType.ATTITUDE);
        Attitude droneYaw = this.drone.getAttribute(AttributeType.ATTITUDE);

        if (droneYaw.getYaw() < 0) {
            yawTextView.setText(String.format("%3.1f", 360 + droneYaw.getYaw()) + "deg");
        } else {
            yawTextView.setText(String.format("%3.1f", droneYaw.getYaw()) + "deg");
        }
    }

    protected void updateSatelliteCounts() {
        Gps satelliteCounts = this.drone.getAttribute(AttributeType.GPS);
        satelliteCountsTextView.setText(String.format("%d", satelliteCounts.getSatellitesCount()));
    }

    // Current Location of Drone
    protected void updateDroneGPS() {
        markerDroneLocation.setMap(null);
        Attitude droneAttitude = this.drone.getAttribute(AttributeType.ATTITUDE);
        double vehicleAttitude = 0.0;
        Gps droneGps = this.drone.getAttribute(AttributeType.GPS);
        LatLong vehiclePosition = droneGps.getPosition();

        if (droneAttitude.getYaw() < 0) {
            vehicleAttitude = 360 + droneAttitude.getYaw();
        } else {
            vehicleAttitude = droneAttitude.getYaw();
        }
//        Log.d("myLog","value : " + vehiclePosition);

        markerDroneLocation.setIcon(OverlayImage.fromResource(R.drawable.dronoverlay3));
        markerDroneLocation.setAnchor(new PointF(0.5f, 0.8f));
        markerDroneLocation.setPosition(castToLatLng(vehiclePosition));
        markerDroneLocation.setAngle((float) vehicleAttitude);
        markerDroneLocation.setMap(naverMap);

        // polyline follows drone gps
        PolylineOverlay polylineFollowDrone = new PolylineOverlay();
        polylineDroneAL.add(markerDroneLocation.getPosition());
        polylineFollowDrone.setCoords(polylineDroneAL);
        polylineFollowDrone.setColor(Color.WHITE);
        polylineFollowDrone.setWidth(3);
        polylineFollowDrone.setCapType(PolylineOverlay.LineCap.Round);
        polylineFollowDrone.setJoinType(PolylineOverlay.LineJoin.Round);
        polylineFollowDrone.setMap(naverMap);

        homePositionG = new LatLong(polylineDroneAL.get(0).latitude, polylineDroneAL.get(0).longitude);
        homePositionN = polylineDroneAL.get(0);
    }

    // Send drone to guided point
    public void GuidedModeLC() {

        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        VehicleMode vehicleMode = vehicleState.getVehicleMode();

        if (vehicleMode != vehicleMode.COPTER_GUIDED) {
            AlertDialog.Builder guidedModeAlertBuilder = new AlertDialog.Builder(MainActivity.this);
            guidedModeAlertBuilder.setMessage("??????????????? ????????????\n?????????????????? ????????? ???????????????.")
                    .setPositiveButton("??????", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            changeToGuidedMode();
                            goToGuidedPoint();
                        }
                    })
                    .setNegativeButton("??????", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            dialog.cancel();
                        }
                    });
            guidedModeAlertBuilder.show();

        } else {
            goToGuidedPoint();
        }

    }

    private void goToGuidedPoint() {
        guidedPoint = new LatLng(markerGuideModeLC.getPosition().latitude, markerGuideModeLC.getPosition().longitude);
        ControlApi.getApi(drone).goTo(castToLatLong(guidedPoint), true, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                checkGuideRun = true;
                alertUser("Sending to Guided Point.");
            }

            @Override
            public void onError(int executionError) {
                alertUser("Unable to send to Guided Point.");
            }

            @Override
            public void onTimeout() {
                alertUser("Sending operation timed out.");
            }
        });
    }

    private void changeToGuidedMode() {
        VehicleApi.getApi(drone).setVehicleMode(VehicleMode.COPTER_GUIDED, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                alertUser("change successful to Guided-Mode.");
            }

            @Override
            public void onError(int executionError) {
                alertUser("Unable to change to Guided-Mode.");
            }

            @Override
            public void onTimeout() {
                alertUser("Changing to Guided-Mode timed out.");
            }
        });
    }

    private void changeToLoiterMode() {
        VehicleApi.getApi(drone).setVehicleMode(VehicleMode.COPTER_LOITER, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                alertUser("change successful to Loiter-mode.");
            }

            @Override
            public void onError(int executionError) {
                alertUser("Unable to change to Loiter-mode.");
            }

            @Override
            public void onTimeout() {
                alertUser("Changing to Loiter-mode timed out.");
            }
        });
    }

    public boolean checkGoal() {
        Gps droneGps = this.drone.getAttribute(AttributeType.GPS);
        LatLong vehiclePosition = droneGps.getPosition();
        guidedPoint = new LatLng(markerGuideModeLC.getPosition().latitude, markerGuideModeLC.getPosition().longitude);
        return guidedPoint.distanceTo(castToLatLng(vehiclePosition)) <= 1;
    }

    public void recyclerViewMessage() {

        // RecyclerView??? LinearLayoutManager ?????? ??????.
        RecyclerView recyclerView = findViewById(R.id.recycler1);
        recyclerView.setLayoutManager(new LinearLayoutManager(this));

        // RecyclerView??? SimpleTextAdapter ?????? ??????.
        SimpleTextAdapter simpleTextadapter = new SimpleTextAdapter(recyclerDataAL);
        recyclerView.setAdapter(simpleTextadapter);
        recyclerView.setHasFixedSize(true);
        recyclerView.smoothScrollToPosition(simpleTextadapter.getItemCount());
    }

    // Buttons to set the map type
    public void MissionButton(View view) {
        if (btnMissionAB.getVisibility() == View.INVISIBLE && btnMissionUndo.getVisibility() == View.INVISIBLE) {
            btnMissionAB.setVisibility(View.VISIBLE);
            btnMissionUndo.setVisibility(View.VISIBLE);

            // remove zoom control(+/-)
            UiSettings uiSettings = naverMap.getUiSettings();
            uiSettings.setZoomControlEnabled(false);
        } else {
            btnMissionAB.setVisibility(View.INVISIBLE);
            btnMissionUndo.setVisibility(View.INVISIBLE);

            UiSettings uiSettings = naverMap.getUiSettings();
            uiSettings.setZoomControlEnabled(true);
        }
    }

    // Tap A-B button: the button for setting A,B markers and the missions show up
    public void buttonAB(View view) {
        missionABflag = true;
    }

    public void MissionUndo(View view) {
        mission.clear();
        btnMission.setText("Mission");
        btnSetMission.setVisibility(View.INVISIBLE);
        aFlag = false;
        bFlag = false;
        resetOverlaysBtn(view);
    }

    // Buttons to set flight path distance
    public void PathDistanceButton(View view) {
        if (btnLengthenPathDistance.getVisibility() == View.INVISIBLE && btnShortenPathDistance.getVisibility() == View.INVISIBLE) {
            btnLengthenPathDistance.setVisibility(View.VISIBLE);
            btnShortenPathDistance.setVisibility(View.VISIBLE);
        } else {
            btnLengthenPathDistance.setVisibility(View.INVISIBLE);
            btnShortenPathDistance.setVisibility(View.INVISIBLE);
        }
    }

    // lengthen flight path distance +5.0
    public void lengthenPathDistance(View view) {
        mPathDistance = mPathDistance + 5;
        btnPathDistance.setText(String.valueOf(mPathDistance) + "m\n????????????");
    }

    // shorten flight path distance -5.0 to 10m
    public void shortenPathDistance(View view) {
        if (mPathDistance > 10) {
            mPathDistance = mPathDistance - 5;
            btnPathDistance.setText(String.valueOf(mPathDistance) + "m\n????????????");
        }
    }

    // Button to set flight path width
    public void PathWidthButton(View view) {
        if (btnWidenPathWidth.getVisibility() == View.INVISIBLE && btnNarrowPathWidthDown.getVisibility() == View.INVISIBLE) {
            btnWidenPathWidth.setVisibility(View.VISIBLE);
            btnNarrowPathWidthDown.setVisibility(View.VISIBLE);
        } else {
            btnWidenPathWidth.setVisibility(View.INVISIBLE);
            btnNarrowPathWidthDown.setVisibility(View.INVISIBLE);
        }
    }

    // widen flight path width +0.5 to 10m
    public void widenPathWidth(View view) {
        if (mPathWidth < 10) {
            mPathWidth = mPathWidth + 0.5;
            btnPathWidth.setText(String.valueOf(mPathWidth) + "m\n?????????");
        }
    }

    // narrow flight path width down -0.5 to 2m
    public void narrowDownPathWidth(View view) {
        if (mPathWidth > 2) {
            mPathWidth = mPathWidth - 0.5;
            btnPathWidth.setText(String.valueOf(mPathWidth) + "m\n?????????");
        }
    }

    public void MissionABPolyline() {
        int missionCount = (int) (mPathDistance / mPathWidth) * 2 + 2;

        double lineDistance = mathUtils.getDistance2D(new LatLong(ABmissionArr.get(0).latitude, ABmissionArr.get(0).longitude), new LatLong(ABmissionArr.get(1).latitude, ABmissionArr.get(1).longitude));

        for (int i = 2; i < missionCount; i++) {
            if (i % 4 == 2 || i % 4 == 3) {
                if (i % 4 == 2) {
                    LatLong latLong = mathUtils.newCoordFromBearingAndDistance(new LatLong(ABmissionArr.get(i - 1).latitude, ABmissionArr.get(i - 1).longitude),
                            mathUtils.getHeadingFromCoordinates(new LatLong(ABmissionArr.get(i - 2).latitude, ABmissionArr.get(i - 2).longitude),
                                    new LatLong(ABmissionArr.get(i - 1).latitude, ABmissionArr.get(i - 1).longitude)) + 90, mPathWidth);
                    ABmissionArr.add(i, new LatLng(latLong.getLatitude(), latLong.getLongitude()));
                } else if (i % 4 == 3) {
                    LatLong latLong = mathUtils.newCoordFromBearingAndDistance(new LatLong(ABmissionArr.get(i - 1).latitude, ABmissionArr.get(i - 1).longitude),
                            mathUtils.getHeadingFromCoordinates(new LatLong(ABmissionArr.get(i - 2).latitude, ABmissionArr.get(i - 2).longitude),
                                    new LatLong(ABmissionArr.get(i - 1).latitude, ABmissionArr.get(i - 1).longitude)) + 90, lineDistance);
                    ABmissionArr.add(i, new LatLng(latLong.getLatitude(), latLong.getLongitude()));
                }
            } else if (i % 4 == 0 || i % 4 == 1) {
                if (i % 4 == 0) {
                    LatLong latLong = mathUtils.newCoordFromBearingAndDistance(new LatLong(ABmissionArr.get(i - 1).latitude, ABmissionArr.get(i - 1).longitude),
                            mathUtils.getHeadingFromCoordinates(new LatLong(ABmissionArr.get(i - 2).latitude, ABmissionArr.get(i - 2).longitude),
                                    new LatLong(ABmissionArr.get(i - 1).latitude, ABmissionArr.get(i - 1).longitude)) - 90, mPathWidth);
                    ABmissionArr.add(i, new LatLng(latLong.getLatitude(), latLong.getLongitude()));
                } else if (i % 4 == 1) {
                    LatLong latLong = mathUtils.newCoordFromBearingAndDistance(new LatLong(ABmissionArr.get(i - 1).latitude, ABmissionArr.get(i - 1).longitude),
                            mathUtils.getHeadingFromCoordinates(new LatLong(ABmissionArr.get(i - 2).latitude, ABmissionArr.get(i - 2).longitude),
                                    new LatLong(ABmissionArr.get(i - 1).latitude, ABmissionArr.get(i - 1).longitude)) - 90, lineDistance);
                    ABmissionArr.add(i, new LatLng(latLong.getLatitude(), latLong.getLongitude()));
                }
            }
        }
        polylineABmission.setCoords(ABmissionArr);
        polylineABmission.setColor(Color.WHITE);
        polylineABmission.setCapType(PolylineOverlay.LineCap.Round);
        polylineABmission.setJoinType(PolylineOverlay.LineJoin.Round);
        polylineABmission.setMap(naverMap);
    }

    public void MissionAB(View view) {
        if (missionSentFlag == false && missionStartFlag == false) {
            //Todo: send mission to Drone. remember: altitude is up to drone's state.
            Altitude droneAltitude = this.drone.getAttribute(AttributeType.ALTITUDE);
            for (int i = 0; i < ABmissionArr.size(); i++) {
                Waypoint waypointforMission = new Waypoint();
                waypointforMission.setCoordinate(new LatLongAlt(ABmissionArr.get(i).latitude, ABmissionArr.get(i).longitude, droneAltitude.getAltitude()));
                waypointforMission.setDelay(1);
                mission.addMissionItem(waypointforMission);
            }
            MissionApi.getApi(this.drone).setMission(mission, true);
            btnSetMission.setText("Start\nMission");
            missionSentFlag = true;
        }
        if (missionSentFlag == true && missionStartFlag == false) {
            MissionApi.getApi(this.drone).startMission(true, true, new AbstractCommandListener() {

                @Override
                public void onSuccess() {
                    alertUser("Mission started successfully");
                    btnSetMission.setText("Pause\nMission");
                    missionStartFlag = true;
                }

                @Override
                public void onError(int executionError) {
                    alertUser("Mission hasn't started");
                }

                @Override
                public void onTimeout() {
                    alertUser("Mission start timed out");
                }
            });
        } else if (missionSentFlag == true && missionStartFlag == true) {
            MissionApi.getApi(this.drone).pauseMission(new AbstractCommandListener() {
                @Override
                public void onSuccess() {
                    alertUser("Mission paused");
                    changeToLoiterMode();
                    btnSetMission.setText("Start\nMission");
                    missionStartFlag = false;
                }

                @Override
                public void onError(int executionError) {
                    alertUser("Mission hasn't paused");
                }

                @Override
                public void onTimeout() {
                    alertUser("Mission pause timed out");
                }
            });
        }
    }

    public void resetOverlaysBtn(View view) {
        markerGuideModeLC.setMap(null);
        polylineDroneAL.clear();

        markerA.setMap(null);
        markerB.setMap(null);
        //preMarkers.setMap(null);

        polylineABmission.setMap(null);
        ABmissionArr.clear();
    }

    public void guidedHomeBtn(View view) {
        State checkVehicleState = drone.getAttribute(AttributeType.STATE);
        VehicleMode checkVehicleMode = checkVehicleState.getVehicleMode();

        if (checkVehicleMode == VehicleMode.COPTER_GUIDED) {
            homeMarker.setMap(null);
            homeMarker.setIconTintColor(Color.YELLOW);
            homeMarker.setPosition(homePositionN);
            homeMarker.setWidth(30);
            homeMarker.setHeight(30);
            homeMarker.setMap(naverMap);
            goHomePoint();

        } else {
            AlertDialog.Builder guidedDialog = new AlertDialog.Builder(MainActivity.this);
            guidedDialog.setTitle(null);
            guidedDialog.setMessage("??????????????? ???????????? ?????????????????? ????????? ???????????????.");
            guidedDialog.setPositiveButton("??????", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    homeMarker.setMap(null);
                    homeMarker.setIconTintColor(Color.YELLOW);
                    homeMarker.setPosition(homePositionN);
                    homeMarker.setWidth(30);
                    homeMarker.setHeight(30);
                    homeMarker.setMap(naverMap);


                    changeToGuidedMode();
                    goHomePoint();

                }
            });
            guidedDialog.setNegativeButton("??????", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    dialog.cancel();
                }
            });
            guidedDialog.show();
        }
    }

    private void goHomePoint() {

        ControlApi.getApi(this.drone).goTo(homePositionG, true, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                alertUser("Drone made it home safely");
                checkReturnHome = true;
            }

            @Override
            public void onError(int executionError) {
                alertUser("error to get drone back home");
            }

            @Override
            public void onTimeout() {
                alertUser("timed out to get drone back home");
            }
        });

    }

    public boolean checkBackHome() {
        Gps droneGps = this.drone.getAttribute(AttributeType.GPS);
        LatLong vehiclePosition = droneGps.getPosition();
        return homePositionN.distanceTo(castToLatLng(vehiclePosition)) <= 1;
    }

    private void changeToLandMode() {
        VehicleApi.getApi(drone).setVehicleMode(VehicleMode.COPTER_LAND, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                alertUser("change successful to Land-Mode.");
            }

            @Override
            public void onError(int executionError) {
                alertUser("Unable to change to Land-Mode.");
            }

            @Override
            public void onTimeout() {
                alertUser("Changing to Land-Mode timed out.");
            }
        });
    }

    @Override
    public void onDroneServiceInterrupted(String errorMsg) {

    }

    @Override
    public void onLinkStateUpdated(@NonNull LinkConnectionStatus connectionStatus) {

    }

    @Override
    public void onTowerConnected() {
        alertUser("DroneKit-Android Connected");
        this.controlTower.registerDrone(this.drone, this.handler);
        this.drone.registerDroneListener(this);
    }

    @Override
    public void onTowerDisconnected() {
        alertUser("DroneKit-Android Interrupted");
    }

    // Helper methods
    // ==========================================================

    protected void alertUser(String message) {
        //Toast.makeText(getApplicationContext(), message, Toast.LENGTH_SHORT).show();
        Log.d(TAG, message);

        // RecyclerView's data
        recyclerDataAL.add(String.format("??? " + message));
        recyclerViewMessage();
    }

    private void runOnMainThread(Runnable runnable) {
        mainHandler.post(runnable);
    }


    @Override
    public void onMapReady(@NonNull @org.jetbrains.annotations.NotNull NaverMap naverMap) {
        // current location of myself
        this.naverMap = naverMap;
        naverMap.setLocationSource(locationSource);
        naverMap.setLocationTrackingMode(LocationTrackingMode.Follow);
        LocationOverlay mylocationOverlay = naverMap.getLocationOverlay();
        mylocationOverlay.setVisible(true);

        naverMap.setMapType(NaverMap.MapType.Satellite);

        naverMap.setOnMapLongClickListener((point, LatLng) -> {
            markerGuideModeLC.setPosition(LatLng);
            markerGuideModeLC.setMap(naverMap);
            GuidedModeLC();
        });

        naverMap.setOnMapClickListener((point, LatLng) -> {
            if (missionABflag == true && aFlag == false && bFlag == false) {
                markerA.setPosition(LatLng);
                markerA.setIcon(OverlayImage.fromResource(R.drawable.xbox_a));
                markerA.setAnchor(new PointF(0.5f, 0.5f));
                markerA.setMap(naverMap);
                aFlag = true;
                ABmissionArr.add(LatLng);
            } else if (missionABflag == true && aFlag == true & bFlag == false) {
                markerB.setPosition(LatLng);
                markerB.setIcon(OverlayImage.fromResource(R.drawable.xbox_b));
                markerB.setAnchor(new PointF(0.5f, 0.5f));
                markerB.setMap(naverMap);
                bFlag = true;
                btnSetMission.setText("SEND\nMISSION");
                ABmissionArr.add(LatLng);
                MissionABPolyline();
                btnSetMission.setVisibility(View.VISIBLE);
            }
        });
    }

    // casting LatLng and LatLong
    private LatLng castToLatLng(LatLong latLong) {
        return new LatLng(latLong.getLatitude(), latLong.getLongitude());
    }

    private LatLong castToLatLong(LatLng latLng) {
        return new LatLong(latLng.latitude, latLng.longitude);
    }
}
