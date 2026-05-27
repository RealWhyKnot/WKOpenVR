package org.wkopenvr.quest;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.Service;
import android.content.ComponentName;
import android.content.Intent;
import android.content.SharedPreferences;
import android.net.Uri;
import android.os.Build;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.util.Log;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

public final class QuestCompanionService extends Service {
    public static final String ACTION_BOOT = "org.wkopenvr.quest.BOOT";

    private static final String TAG = "WKOpenVRQuest";
    private static final String PREFS = "wkopenvr_quest";
    private static final String PREF_PAIRING_KEY = "pairing_key";
    private static final String PREF_AUTO_LAUNCH = "auto_launch_enabled";
    private static final String PREF_PACKAGE = "selected_package";
    private static final String PREF_ACTIVITY = "selected_activity";

    private static final String EXTRA_PAIRING_KEY = "wkopenvr_pairing_key";
    private static final String EXTRA_AUTO_LAUNCH = "wkopenvr_auto_launch_enabled";
    private static final String EXTRA_PACKAGE = "wkopenvr_selected_package";
    private static final String EXTRA_ACTIVITY = "wkopenvr_selected_activity";
    private static final String EXTRA_LAUNCH_NOW = "wkopenvr_launch_now";

    private static final String CHANNEL_ID = "wkopenvr_quest_companion";
    private static final int NOTIFICATION_ID = 4159;
    private static final int HTTP_PORT = 39789;

    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private SharedPreferences prefs;
    private ServerSocket serverSocket;
    private Thread serverThread;
    private volatile boolean serverRunning;
    private boolean bootLaunchQueued;

    @Override
    public void onCreate() {
        super.onCreate();
        prefs = getSharedPreferences(PREFS, MODE_PRIVATE);
        startForeground(NOTIFICATION_ID, buildNotification());
        startHttpServer();
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        if (intent == null) {
            return START_STICKY;
        }

        String suppliedKey = intent.getStringExtra(EXTRA_PAIRING_KEY);
        if (suppliedKey != null) {
            if (!acceptPairingKey(suppliedKey)) {
                Log.w(TAG, "Ignoring command with invalid pairing key.");
                return START_STICKY;
            }
            applyIntentConfig(intent);
            if (intent.getBooleanExtra(EXTRA_LAUNCH_NOW, false)) {
                launchConfiguredApp();
            }
            return START_STICKY;
        }

        if (ACTION_BOOT.equals(intent.getAction())) {
            queueBootLaunch();
        }
        return START_STICKY;
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    @Override
    public void onDestroy() {
        serverRunning = false;
        if (serverSocket != null) {
            try {
                serverSocket.close();
            } catch (IOException ignored) {
            }
        }
        super.onDestroy();
    }

    private Notification buildNotification() {
        NotificationManager manager = getSystemService(NotificationManager.class);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && manager != null) {
            NotificationChannel channel = new NotificationChannel(
                    CHANNEL_ID,
                    "WKOpenVR Quest Companion",
                    NotificationManager.IMPORTANCE_LOW);
            manager.createNotificationChannel(channel);
        }

        Notification.Builder builder = Build.VERSION.SDK_INT >= Build.VERSION_CODES.O
                ? new Notification.Builder(this, CHANNEL_ID)
                : new Notification.Builder(this);
        return builder
                .setContentTitle("WKOpenVR Quest Companion")
                .setContentText("Ready for paired WKOpenVR commands")
                .setSmallIcon(android.R.drawable.stat_sys_upload_done)
                .setOngoing(true)
                .build();
    }

    private boolean acceptPairingKey(String suppliedKey) {
        if (!isValidPairingKey(suppliedKey)) {
            return false;
        }
        String storedKey = prefs.getString(PREF_PAIRING_KEY, "");
        if (storedKey.isEmpty()) {
            prefs.edit().putString(PREF_PAIRING_KEY, suppliedKey).apply();
            Log.i(TAG, "Pairing key established.");
            return true;
        }
        return storedKey.equals(suppliedKey);
    }

    private boolean isAuthorized(String suppliedKey) {
        String storedKey = prefs.getString(PREF_PAIRING_KEY, "");
        return isValidPairingKey(suppliedKey) && storedKey.equals(suppliedKey);
    }

    private static boolean isValidPairingKey(String key) {
        if (key == null || key.length() != 64) {
            return false;
        }
        for (int i = 0; i < key.length(); ++i) {
            char c = key.charAt(i);
            boolean hex = (c >= '0' && c <= '9') ||
                    (c >= 'a' && c <= 'f') ||
                    (c >= 'A' && c <= 'F');
            if (!hex) {
                return false;
            }
        }
        return true;
    }

    private void applyIntentConfig(Intent intent) {
        SharedPreferences.Editor editor = prefs.edit();
        if (intent.hasExtra(EXTRA_AUTO_LAUNCH)) {
            editor.putBoolean(PREF_AUTO_LAUNCH, intent.getBooleanExtra(EXTRA_AUTO_LAUNCH, false));
        }
        String packageName = intent.getStringExtra(EXTRA_PACKAGE);
        if (packageName != null) {
            editor.putString(PREF_PACKAGE, packageName);
        }
        String activityName = intent.getStringExtra(EXTRA_ACTIVITY);
        if (activityName != null) {
            editor.putString(PREF_ACTIVITY, activityName);
        }
        editor.apply();
    }

    private void queueBootLaunch() {
        if (bootLaunchQueued || !prefs.getBoolean(PREF_AUTO_LAUNCH, false)) {
            return;
        }
        bootLaunchQueued = true;
        mainHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                launchConfiguredApp();
                bootLaunchQueued = false;
            }
        }, 8000);
    }

    private boolean launchConfiguredApp() {
        String packageName = prefs.getString(PREF_PACKAGE, "");
        String activityName = prefs.getString(PREF_ACTIVITY, "");
        if (packageName.isEmpty()) {
            return false;
        }

        Intent launchIntent = null;
        if (!activityName.isEmpty()) {
            launchIntent = new Intent(Intent.ACTION_MAIN);
            launchIntent.addCategory(Intent.CATEGORY_LAUNCHER);
            launchIntent.setComponent(new ComponentName(packageName, activityName));
        } else if (getPackageManager() != null) {
            launchIntent = getPackageManager().getLaunchIntentForPackage(packageName);
        }
        if (launchIntent == null) {
            Log.w(TAG, "No launch intent for " + packageName);
            return false;
        }

        launchIntent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        try {
            startActivity(launchIntent);
            Log.i(TAG, "Launched " + packageName);
            return true;
        } catch (RuntimeException ex) {
            Log.w(TAG, "Launch failed for " + packageName, ex);
            return false;
        }
    }

    private void startHttpServer() {
        if (serverThread != null) {
            return;
        }
        serverRunning = true;
        serverThread = new Thread(new Runnable() {
            @Override
            public void run() {
                runHttpServer();
            }
        }, "WKOpenVRQuestHttp");
        serverThread.start();
    }

    private void runHttpServer() {
        try {
            serverSocket = new ServerSocket(HTTP_PORT, 4, InetAddress.getByName("0.0.0.0"));
            while (serverRunning) {
                Socket socket = serverSocket.accept();
                handleHttp(socket);
            }
        } catch (IOException ex) {
            if (serverRunning) {
                Log.w(TAG, "HTTP server stopped", ex);
            }
        }
    }

    private void handleHttp(Socket socket) {
        try (Socket s = socket) {
            BufferedReader reader = new BufferedReader(
                    new InputStreamReader(s.getInputStream(), StandardCharsets.US_ASCII));
            String requestLine = reader.readLine();
            if (requestLine == null || requestLine.isEmpty()) {
                return;
            }

            String headerKey = "";
            String line;
            while ((line = reader.readLine()) != null && !line.isEmpty()) {
                int colon = line.indexOf(':');
                if (colon <= 0) {
                    continue;
                }
                String name = line.substring(0, colon).trim().toLowerCase(Locale.ROOT);
                if ("x-wkopenvr-key".equals(name)) {
                    headerKey = line.substring(colon + 1).trim();
                }
            }

            String[] parts = requestLine.split(" ");
            if (parts.length < 2) {
                writeResponse(s.getOutputStream(), 400, "bad request\n");
                return;
            }
            Uri uri = Uri.parse("http://127.0.0.1" + parts[1]);
            String key = headerKey.isEmpty() ? uri.getQueryParameter("key") : headerKey;
            if (!isAuthorized(key)) {
                writeResponse(s.getOutputStream(), 403, "forbidden\n");
                return;
            }

            String path = uri.getPath();
            if ("/health".equals(path)) {
                writeResponse(s.getOutputStream(), 200, "paired\n");
            } else if ("/settings".equals(path) || "/list".equals(path)) {
                writeJsonResponse(s.getOutputStream(), currentSettingsJson());
            } else if ("/config".equals(path)) {
                applyHttpConfig(uri);
                writeResponse(s.getOutputStream(), 200, "configured\n");
            } else if ("/launch".equals(path)) {
                writeResponse(s.getOutputStream(), launchConfiguredApp() ? 200 : 500, "launch\n");
            } else {
                writeResponse(s.getOutputStream(), 404, "not found\n");
            }
        } catch (IOException ex) {
            Log.w(TAG, "HTTP request failed", ex);
        }
    }

    private void applyHttpConfig(Uri uri) {
        SharedPreferences.Editor editor = prefs.edit();
        String autoLaunch = uri.getQueryParameter("autoLaunch");
        if (autoLaunch != null) {
            editor.putBoolean(PREF_AUTO_LAUNCH,
                    "1".equals(autoLaunch) || "true".equalsIgnoreCase(autoLaunch));
        }
        String packageName = uri.getQueryParameter("package");
        if (packageName != null) {
            editor.putString(PREF_PACKAGE, packageName);
        }
        String activityName = uri.getQueryParameter("activity");
        if (activityName != null) {
            editor.putString(PREF_ACTIVITY, activityName);
        }
        editor.apply();
    }

    private String currentSettingsJson() {
        return "{" +
                "\"paired\":true," +
                "\"autoLaunchEnabled\":" + (prefs.getBoolean(PREF_AUTO_LAUNCH, false) ? "true" : "false") + "," +
                "\"selectedPackage\":\"" + jsonEscape(prefs.getString(PREF_PACKAGE, "")) + "\"," +
                "\"selectedActivity\":\"" + jsonEscape(prefs.getString(PREF_ACTIVITY, "")) + "\"," +
                "\"httpPort\":" + HTTP_PORT +
                "}\n";
    }

    private static String jsonEscape(String value) {
        if (value == null) {
            return "";
        }
        StringBuilder out = new StringBuilder(value.length() + 8);
        for (int i = 0; i < value.length(); ++i) {
            char c = value.charAt(i);
            switch (c) {
                case '\\':
                    out.append("\\\\");
                    break;
                case '"':
                    out.append("\\\"");
                    break;
                case '\n':
                    out.append("\\n");
                    break;
                case '\r':
                    out.append("\\r");
                    break;
                case '\t':
                    out.append("\\t");
                    break;
                default:
                    if (c < 0x20) {
                        out.append(String.format(Locale.ROOT, "\\u%04x", (int)c));
                    } else {
                        out.append(c);
                    }
                    break;
            }
        }
        return out.toString();
    }

    private static void writeResponse(OutputStream out, int statusCode, String body) throws IOException {
        String reason;
        switch (statusCode) {
            case 200:
                reason = "OK";
                break;
            case 400:
                reason = "Bad Request";
                break;
            case 403:
                reason = "Forbidden";
                break;
            case 404:
                reason = "Not Found";
                break;
            default:
                reason = "Error";
                break;
        }
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        String headers = "HTTP/1.1 " + statusCode + " " + reason + "\r\n" +
                "Content-Type: text/plain; charset=utf-8\r\n" +
                "Content-Length: " + bytes.length + "\r\n" +
                "Connection: close\r\n\r\n";
        out.write(headers.getBytes(StandardCharsets.US_ASCII));
        out.write(bytes);
    }

    private static void writeJsonResponse(OutputStream out, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        String headers = "HTTP/1.1 200 OK\r\n" +
                "Content-Type: application/json; charset=utf-8\r\n" +
                "Content-Length: " + bytes.length + "\r\n" +
                "Connection: close\r\n\r\n";
        out.write(headers.getBytes(StandardCharsets.US_ASCII));
        out.write(bytes);
    }
}
