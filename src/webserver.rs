use futures_util::{SinkExt, StreamExt};
use serde::Serialize;
use std::convert::Infallible;
use std::net::SocketAddr;
use std::process::ExitCode;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tokio::signal;
use warp::http::StatusCode;
use warp::reject::MethodNotAllowed;
use warp::ws::WebSocket;
use warp::Filter;
use warp::Rejection;
use warp::Reply;

mod app;
mod attitude_estimation;
mod cam;
mod cam_cal;
mod common_axis;
mod opencvutils;
mod optim;
mod testingutils;
mod utils;
mod webutils;

#[derive(Debug)]
struct AppError {
    inner: String,
}
impl warp::reject::Reject for AppError {}

#[derive(Serialize)]
struct ErrorResponse {
    message: String,
    code: u16,
}

async fn handle_rejection(err: Rejection) -> Result<impl Reply, Infallible> {
    let code;
    let message;

    if err.is_not_found() {
        eprintln!("Rejection: 404 Not Found. Details: {:?}", err);
        code = StatusCode::NOT_FOUND;
        message = "Not Found".to_string();
    } else if let Some(app_err) = err.find::<AppError>() {
        // This is our custom ApplicationError!
        eprintln!("Application Error: {}", app_err.inner); // Log full error details
        code = StatusCode::INTERNAL_SERVER_ERROR;
        message = "Internal Server Error".to_string(); // Return a generic message to client
    } else if let Some(err) = err.find::<MethodNotAllowed>() {
        // Handle MethodNotAllowed specifically
        eprintln!("Method Not Allowed: {:?}", err);
        code = StatusCode::METHOD_NOT_ALLOWED;
        message = "Method Not Allowed".to_string();
    } else if let Some(e) = err.find::<warp::filters::body::BodyDeserializeError>() {
        // Handle deserialization errors
        eprintln!("Deserialization Error: {}", e);
        code = StatusCode::BAD_REQUEST;
        message = "Invalid request body".to_string();
    } else {
        // Fallback for any unhandled rejections
        eprintln!("Unhandled rejection: {:?}", err);
        code = StatusCode::INTERNAL_SERVER_ERROR; // Or BAD_REQUEST, depending on the unhandled type
        message = "An unexpected error occurred".to_string();
    }

    let json = warp::reply::json(&ErrorResponse {
        message,
        code: code.as_u16(),
    });

    Ok(warp::reply::with_status(json, code))
}

#[tokio::main]
async fn main() -> ExitCode {
    let application = Arc::new(app::App::load_or_default("config.json"));

    //Start camera thread
    let app_clone = Arc::clone(&application);
    let camera_thread_handle = thread::spawn(move || {
        if let Err(res) = app::tick(app_clone.as_ref()) {
            eprintln!("App failed: {:?}", res);
            println!("Shutting down due to app failure");
            app_clone.running.store(false, Ordering::Release);
        } else {
            println!("App terminated normally");
        }
    });

    // Serve / as /web/axisAlign.html
    let index = warp::path::end().map(|| {
        warp::reply::html(
            std::fs::read_to_string("web/axisAlign.html")
                .unwrap_or_else(|_| "Index not found".to_string()),
        )
    });

    // Serve static files from ./web
    let static_files = warp::fs::dir("web");

    // POST handlers
    let app_clone = Arc::clone(&application);
    let get_state_post_handler =
        warp::post()
            .and(warp::path!("api" / "get_state"))
            .and_then(move || {
                let app_cloned_for_fut = Arc::clone(&app_clone);
                async move {
                    println!("api/get_state");
                    match app_cloned_for_fut.get_state() {
                        Ok(state) => Ok(warp::reply::json(&state)),
                        Err(e) => Err(warp::reject::custom(AppError { inner: e })),
                    }
                }
            });

    let app_clone = Arc::clone(&application);
    let set_settings_post_handler = warp::post()
        .and(warp::path!("api" / "set_settings"))
        .and(warp::body::json())
        .and_then(move |rx: app::Settings| {
            let app_cloned_for_fut = Arc::clone(&app_clone);
            async move {
                println!("api/set_settings");
                match app_cloned_for_fut.set_settings(rx) {
                    Ok(state) => Ok(warp::reply::json(&state)),
                    Err(e) => Err(warp::reject::custom(AppError { inner: e })),
                }
            }
        });

    let app_clone = Arc::clone(&application);
    let shutdown_post_handler = warp::post()
        .and(warp::path!("api" / "shutdown"))
        .and(warp::body::json())
        .and_then(move |rx: serde_json::Value| {
            let app_cloned_for_fut = Arc::clone(&app_clone);
            async move {
                println!("api/shutdown");
                match rx {
                    serde_json::Value::Object(ref map)
                        if map.get("shutdown")
                            == Some(&serde_json::Value::String("shutdown".to_string())) =>
                    {
                        println!("Received shutdown command, shutting down...");
                        app_cloned_for_fut.set_returncode(31);
                        app_cloned_for_fut.running.store(false, Ordering::Release);
                        Ok(warp::reply::json(&serde_json::json!({})))
                    }
                    _ => Err(warp::reject::custom(AppError {
                        inner: "Payload must be {'shutdown': 'shutdown'} to shutdown".to_string(),
                    })),
                }
            }
        });

    let app_clone = Arc::clone(&application);
    let capture_post_handler = warp::post()
        .and(warp::path!("api" / "capture"))
        .and(warp::body::json())
        .and_then(move |rx: app::CameraModeRequest| {
            let app_cloned_for_fut = Arc::clone(&app_clone);
            async move {
                println!("api/capture");
                match app_cloned_for_fut.set_camera_mode(rx) {
                    Ok(state) => Ok(warp::reply::json(&state)),
                    Err(e) => Err(warp::reject::custom(AppError { inner: e })),
                }
            }
        });

    let app_clone = Arc::clone(&application);
    let axis_calibration_post_handler = warp::post()
        .and(warp::path!("api" / "axis_calibration"))
        .and(warp::body::json())
        .and_then(move |rx: app::AxisCalibrationRequest| {
            let app_cloned_for_fut = Arc::clone(&app_clone);
            async move {
                println!("api/axis_calibration");
                match app_cloned_for_fut.axis_calibration(rx) {
                    Ok(state) => Ok(warp::reply::json(&state)),
                    Err(e) => Err(warp::reject::custom(AppError { inner: e })),
                }
            }
        });

    let app_clone = Arc::clone(&application);
    let auto_calibration_post_handler = warp::post()
        .and(warp::path!("api" / "auto_calibration"))
        .and(warp::body::json())
        .and_then(move |rx: app::AutoCalibrationRequest| {
            let app_cloned_for_fut = Arc::clone(&app_clone);
            async move {
                println!("api/auto_calibration");
                match app_cloned_for_fut.auto_calibration(rx) {
                    Ok(state) => Ok(warp::reply::json(&state)),
                    Err(e) => Err(warp::reject::custom(AppError { inner: e })),
                }
            }
        });

    let app_clone = Arc::clone(&application);
    let stream_ws_route =
        warp::path!("api" / "stream")
            .and(warp::ws())
            .map(move |ws: warp::ws::Ws| {
                let a = Arc::clone(&app_clone);
                ws.on_upgrade(move |s| handle_stream_ws(s, a))
            });

    let routes = static_files
        .or(index)
        .or(get_state_post_handler)
        .or(set_settings_post_handler)
        .or(shutdown_post_handler)
        .or(capture_post_handler)
        .or(axis_calibration_post_handler)
        .or(auto_calibration_post_handler)
        .or(stream_ws_route)
        .recover(handle_rejection);

    let app_clone = Arc::clone(&application);
    let shutdown = async move {
        signal::ctrl_c()
            .await
            .expect("failed to install Ctrl+C handler");
        app_clone.running.store(false, Ordering::Release);
        println!("Received Ctrl+C, shutdown triggered...");
    };

    let addr: SocketAddr = ([0, 0, 0, 0], 5000).into();
    println!("Running server on {:?}", addr);
    let app_clone = Arc::clone(&application);
    let server_future = async {
        match warp::serve(routes).try_bind_with_graceful_shutdown(addr, shutdown) {
            Ok((_, future)) => {
                future.await;
            }
            Err(e) => {
                app_clone.running.store(false, Ordering::Release);
                eprintln!("Failed to bind to {}: {}", addr, e);
                println!("Server failed to start, cleanup triggered...");
            }
        };
    };

    let app_clone = Arc::clone(&application);
    let wait_for_signal_and_then_timeout = async move {
        while app_clone.running.load(Ordering::Acquire) {
            tokio::time::sleep(Duration::from_millis(50)).await; // Poll every 50ms
        }
        println!("Hard shutdown trigger: Detected `running` flag is false. Starting post-signal delay...");
        tokio::time::sleep(Duration::from_secs(2)).await;
    };

    tokio::select! {
        _ = server_future => {
            println!("Server shut down gracefully");
        }
        _ = wait_for_signal_and_then_timeout => {
            eprintln!("Server shut down failed. Forcing exit.");
        }
    }

    match camera_thread_handle.join() {
        Ok(_) => println!("Camera thread joined successfully"),
        Err(e) => eprintln!("Camera thread panicked: {:?}", e),
    }

    match application.get_returncode() {
        0 => ExitCode::SUCCESS,
        code => ExitCode::from(code),
    }
}

async fn handle_stream_ws(ws: WebSocket, application: Arc<app::App>) {
    let (mut tx, mut rx) = ws.split();

    println!("WebSocket stream connected");

    let dispatcher = &application.stream_dispatcher;

    while application.running.load(Ordering::Acquire) {
        let d = Duration::from_millis(100);
        let encoded_bytes = match dispatcher.get_blocking_with_timeout(d) {
            Ok(data) => data,
            Err(webutils::TimeoutError) => {
                // Timeout, continue to next iteration
                if let Ok(None) = tokio::time::timeout(Duration::from_millis(0), rx.next()).await {
                    println!("WebSocket stream disconnected by client");
                    return;
                }
                continue;
            }
        };

        // Send over WebSocket as json
        if let Err(_) = tx.send(warp::ws::Message::binary(encoded_bytes)).await {
            println!("WebSocket stream disconnected by client");
            return;
        }
    }

    if let Err(e) = tx.close().await {
        eprintln!("Error closing WebSocket stream: {}", e);
    } else {
        println!("WebSocket stream disconnected by server.");
    }
}
