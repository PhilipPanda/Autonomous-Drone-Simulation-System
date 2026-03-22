package com.adsim.network;

import com.adsim.model.TelemetryFrame;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;
import java.util.concurrent.atomic.AtomicReference;

public class TelemetryClient implements Runnable {

    private final String host;
    private final int port;
    private final AtomicReference<TelemetryFrame> latestFrame;
    private final AtomicReference<ConnectionState> connectionState;
    private final ObjectMapper mapper = new ObjectMapper();
    private volatile boolean running = true;

    public TelemetryClient(String host,
                           int port,
                           AtomicReference<TelemetryFrame> latestFrame,
                           AtomicReference<ConnectionState> connectionState) {
        this.host = host;
        this.port = port;
        this.latestFrame = latestFrame;
        this.connectionState = connectionState;
    }

    public void stop() {
        running = false;
    }

    @Override
    public void run() {
        while (running) {
            connectionState.set(ConnectionState.CONNECTING);
            try (Socket socket = new Socket(host, port)) {
                socket.setSoTimeout(5000);
                connectionState.set(ConnectionState.CONNECTED);
                BufferedReader reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                String line;
                while (running && (line = reader.readLine()) != null) {
                    if (line.isBlank()) continue;
                    try {
                        TelemetryFrame frame = mapper.readValue(line, TelemetryFrame.class);
                        latestFrame.set(frame);
                    } catch (Exception ignored) {
                    }
                }
            } catch (IOException e) {
                // connection lost or failed
            }
            if (running) {
                connectionState.set(ConnectionState.DISCONNECTED);
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        connectionState.set(ConnectionState.DISCONNECTED);
    }
}
