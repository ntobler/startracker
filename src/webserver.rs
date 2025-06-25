#![cfg(feature = "webserver")]

use futures_util::{SinkExt, StreamExt};
use serde_json;
use std::convert::Infallible;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tokio::signal;
use warp::ws::WebSocket;
use warp::Filter;

mod cam;
mod opencvutils;
mod webutils;

#[tokio::main]
async fn main() {
    let running = Arc::new(AtomicBool::new(true)); // Initially set to true

    let image_dispatcher = Arc::new(webutils::DataDispatcher::<std::sync::Arc<Vec<u8>>>::new());
    let stream_dispatcher = Arc::new(webutils::DataDispatcher::<String>::new());

    //Start camera thread
    let image_dis = Arc::clone(&image_dispatcher);
    let stream_dis = Arc::clone(&stream_dispatcher);
    let running_clone = Arc::clone(&running);
    let camera_thread_handle = thread::spawn(move || {
        tick(
            image_dis.as_ref(),
            stream_dis.as_ref(),
            running_clone.as_ref(),
        );
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

    // POST handler
    let post_handler = warp::post()
        .and(warp::path!("api" / "get_state"))
        .and(warp::body::json().or_else(|_| async {
            let state = serde_json::json!({
                "view_settings": serde_json::json!({
                    "coordinate_frame": false,
                    "brightness": 1u32,
                    "image_type": "raw",
                    "target_quality": "50k",
                }),
            });

            Ok::<(serde_json::Value,), Infallible>((state,))
        }))
        .map(|mut payload: serde_json::Value| {
            println!("api/get_state");
            payload["parsed"] = serde_json::json!(true);
            warp::reply::json(&payload)
        });

    // WebSocket handler
    let running_clone = Arc::clone(&running);
    let image_ws_route =
        warp::path!("api" / "image")
            .and(warp::ws())
            .map(move |ws: warp::ws::Ws| {
                let dispatcher = Arc::clone(&image_dispatcher);
                let r = Arc::clone(&running_clone);
                ws.on_upgrade(move |s| handle_image_ws(s, dispatcher, r))
            });
    let running_clone = Arc::clone(&running);
    let stream_ws_route =
        warp::path!("api" / "stream")
            .and(warp::ws())
            .map(move |ws: warp::ws::Ws| {
                let dispatcher = Arc::clone(&stream_dispatcher);
                let r = Arc::clone(&running_clone);
                ws.on_upgrade(move |s| handle_stream_ws(s, dispatcher, r))
            });

    let routes = static_files
        .or(index)
        .or(post_handler)
        .or(image_ws_route)
        .or(stream_ws_route);

    println!("Running server on http://localhost:3030");

    let running_clone = Arc::clone(&running);
    let shutdown = async move {
        signal::ctrl_c()
            .await
            .expect("failed to install Ctrl+C handler");
        println!("Received Ctrl+C, shutting down...");
        running_clone.store(false, Ordering::Release);
    };

    let addr: SocketAddr = ([0, 0, 0, 0], 5000).into();
    let (_addr, server_future) = warp::serve(routes).bind_with_graceful_shutdown(addr, shutdown);

    let wait_for_signal_and_then_timeout = async move {
        while running.load(Ordering::Acquire) {
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

fn tick(
    image_dispatcher: &webutils::DataDispatcher<std::sync::Arc<Vec<u8>>>,
    stream_dispatcher: &webutils::DataDispatcher<String>,
    running: &AtomicBool,
) {
    println!("Setup Camera");

    let config = cam::CameraConfig {
        analogue_gain: 1,
        digital_gain: 4,
        exposure_us: 500000,
        binning: 2,
    };
    let mut camera = match cam::Camera::new(&config) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("Error initializing camera: {}.", e);
            return;
        }
    };

    camera.set_config(&config);

    let mut ie = opencvutils::ImageEncoder::new(Some(30.0));

    while running.load(Ordering::Acquire) {
        // Get fresh image
        let raw: cam::Frame<u8> = match camera.capture() {
            Ok(v) => v,
            Err(e) => {
                eprintln!("Error getting camera image: {}.", e);
                continue;
            }
        };

        let encoded = match ie.encode(&raw) {
            Some(v) => v,
            None => {
                eprintln!("Error encoding image");
                continue;
            }
        };

        image_dispatcher.put(std::sync::Arc::new(encoded));

        let data = serde_json::json!({
            "image_size": (raw.width, raw.height),
            "image_quality": ie.quality_str(),
        });

        stream_dispatcher.put(data.to_string());
    }

    println!("Camera thread stopped gracefully");
}

async fn handle_image_ws(
    ws: WebSocket,
    dispatcher: Arc<webutils::DataDispatcher<std::sync::Arc<Vec<u8>>>>,
    running: Arc<AtomicBool>,
) {
    let (mut tx, mut _rx) = ws.split();

    println!("WebSocket image connected");

    while running.load(Ordering::Acquire) {
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

async fn handle_stream_ws(
    ws: WebSocket,
    dispatcher: Arc<webutils::DataDispatcher<String>>,
    running: Arc<AtomicBool>,
) {
    let (mut tx, mut _rx) = ws.split();

    println!("WebSocket stream connected");

    while running.load(Ordering::Acquire) {
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
