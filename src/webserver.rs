use futures_util::{SinkExt, StreamExt};
use serde::Serialize;
use std::convert::Infallible;
use std::net::SocketAddr;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tokio::signal;
use warp::http::StatusCode;
use warp::ws::WebSocket;
use warp::Filter;
use warp::Rejection;
use warp::Reply;

mod app;
mod cam;
mod opencvutils;
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
        code = StatusCode::NOT_FOUND;
        message = "Not Found".to_string();
    } else if let Some(app_err) = err.find::<AppError>() {
        // This is our custom ApplicationError!
        eprintln!("Application Error: {}", app_err.inner); // Log full error details
        code = StatusCode::INTERNAL_SERVER_ERROR;
        message = "Internal Server Error".to_string(); // Return a generic message to client
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
async fn main() {
    let application = Arc::new(app::App::load_or_default("config.json"));

    //Start camera thread
    let app_clone = Arc::clone(&application);
    let camera_thread_handle = thread::spawn(move || {
        app::tick(app_clone.as_ref());
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
        .and_then(move |rx: app::Persistent| {
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

    // WebSocket handler
    let app_clone = Arc::clone(&application);
    let image_ws_route =
        warp::path!("api" / "image")
            .and(warp::ws())
            .map(move |ws: warp::ws::Ws| {
                let a = Arc::clone(&app_clone);
                ws.on_upgrade(move |s| handle_image_ws(s, a))
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
        .or(capture_post_handler)
        .or(image_ws_route)
        .or(stream_ws_route)
        .recover(handle_rejection);

    let app_clone = Arc::clone(&application);
    let shutdown = async move {
        signal::ctrl_c()
            .await
            .expect("failed to install Ctrl+C handler");
        println!("Received Ctrl+C, shutting down...");
        app_clone.running.store(false, Ordering::Release);
    };

    let addr: SocketAddr = ([0, 0, 0, 0], 5000).into();
    println!("Running server on {:?}", addr);
    let (_addr, server_future) = warp::serve(routes).bind_with_graceful_shutdown(addr, shutdown);

    let wait_for_signal_and_then_timeout = async move {
        while application.running.load(Ordering::Acquire) {
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

    camera_thread_handle.join().expect("Camera thread panicked");
    println!("Camera joined");
}

async fn handle_image_ws(ws: WebSocket, application: Arc<app::App>) {
    let (mut tx, mut _rx) = ws.split();

    println!("WebSocket image connected");

    let dispatcher = &application.image_dispatcher;

    while application.running.load(Ordering::Acquire) {
        // We need to copy the Vector inside the Arc to send it over WebSocket
        // Copying here is better as we don't block the mutex
        let d = Duration::from_millis(100);
        let encoded = match dispatcher.get_blocking_with_timeout(d) {
            Ok(data) => data.as_ref().clone(),
            Err(webutils::TimeoutError) => {
                // Timeout, continue to next iteration
                continue;
            }
        };
        // Send over WebSocket as binary
        if let Err(e) = tx.send(warp::ws::Message::binary(encoded)).await {
            eprintln!("WS send error: {}. disconnected", e);
            break;
        }
    }

    if let Err(e) = tx.close().await {
        eprintln!("Error sending WebSocket close frame: {}", e);
    } else {
        println!("Successfully sent WebSocket close frame.");
    }

    println!("WebSocket image disconnected gracefully");
}

async fn handle_stream_ws(ws: WebSocket, application: Arc<app::App>) {
    let (mut tx, mut _rx) = ws.split();

    println!("WebSocket stream connected");

    let dispatcher = &application.stream_dispatcher;

    while application.running.load(Ordering::Acquire) {
        let d = Duration::from_millis(100);
        let json_str = match dispatcher.get_blocking_with_timeout(d) {
            Ok(data) => data,
            Err(webutils::TimeoutError) => {
                // Timeout, continue to next iteration
                continue;
            }
        };

        // Send over WebSocket as json
        if let Err(e) = tx.send(warp::ws::Message::text(json_str)).await {
            eprintln!("WS send error: {}. disconnected", e);
            break;
        }
    }

    if let Err(e) = tx.close().await {
        eprintln!("Error sending WebSocket close frame: {}", e);
    } else {
        println!("Successfully sent WebSocket close frame.");
    }

    println!("WebSocket stream disconnected gracefully");
}
