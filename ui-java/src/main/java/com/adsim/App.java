package com.adsim;

import com.adsim.ui.MainWindow;
import javafx.application.Application;
import javafx.stage.Stage;

public class App extends Application {

    private String host = "127.0.0.1";
    private int port = 5760;

    @Override
    public void init() {
        for (String param : getParameters().getRaw()) {
            if (param.startsWith("--host=")) {
                host = param.substring("--host=".length());
            } else if (param.startsWith("--port=")) {
                try {
                    port = Integer.parseInt(param.substring("--port=".length()));
                } catch (NumberFormatException ignored) {
                }
            }
        }
    }

    @Override
    public void start(Stage primaryStage) {
        new MainWindow(primaryStage, host, port);
    }

    public static void main(String[] args) {
        launch(args);
    }
}
