#![cfg(feature = "webserver")]

use futures_util::{SinkExt, StreamExt};
use serde_json;
use std::convert::Infallible;
use std::net::SocketAddr;
use std::time::Duration;
use tokio::signal;
use tokio::time::sleep;
use warp::ws::WebSocket;
use warp::Filter;

pub mod cam;

#[tokio::main]
async fn main() {
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
            Ok::<(serde_json::Value,), Infallible>((serde_json::json!({}),))
        }))
        .map(|mut payload: serde_json::Value| {
            println!("api/get_state");
            payload["parsed"] = serde_json::json!(true);
            warp::reply::json(&payload)
        });

    // WebSocket handler
    let ws_route = warp::path!("api" / "image")
        .and(warp::ws())
        .map(|ws: warp::ws::Ws| ws.on_upgrade(handle_ws));

    let routes = static_files.or(index).or(post_handler).or(ws_route);

    println!("Running server on http://localhost:3030");

    let shutdown = async {
        signal::ctrl_c()
            .await
            .expect("failed to install Ctrl+C handler");
        println!("Received Ctrl+C, shutting down...");
    };

    let addr: SocketAddr = ([127, 0, 0, 1], 3030).into();
    let (_addr, server_future) = warp::serve(routes).bind_with_graceful_shutdown(addr, shutdown);

    server_future.await;

    println!("Server shut down gracefully");
}

async fn handle_ws(ws: WebSocket) {
    use opencv::{
        core::{Mat, Vector},
        imgcodecs::{imencode, IMWRITE_JPEG_QUALITY},
        prelude::*,
    };

    let (mut tx, mut _rx) = ws.split();

    println!("WebSocket connected");

    let mut camera = match cam::Camera::new() {
        Ok(v) => v,
        Err(e) => {
            eprintln!("Error initializing camera: {}.", e);
            return;
        }
    };

    loop {
        // Get fresh image
        let raw = match camera.capture() {
            Ok(v) => v,
            Err(e) => {
                eprintln!("Error getting camera image: {}.", e);
                continue;
            }
        };

        // convert to u8 image
        let scaled: Vec<u8> = raw.iter().map(|v| (v / 4) as u8).collect();
        let img = Mat::new_rows_cols_with_data(1080, 1920, &scaled).unwrap();

        // Encode to JPEG
        let mut buf = Vector::<u8>::new();
        imencode(
            ".jpg",
            &img,
            &mut buf,
            &Vector::<i32>::from(vec![IMWRITE_JPEG_QUALITY, 50]),
        )
        .unwrap();

        // Send over WebSocket as binary
        if let Err(e) = tx.send(warp::ws::Message::binary(buf.to_vec())).await {
            eprintln!("WS send error: {}. disconnected", e);
            break;
        }

        sleep(Duration::from_secs(1)).await;
    }
}

#[cfg(test)]
mod tests {
    use opencv;
    use opencv::core::MatTraitConst;
    use opencv::prelude::MatTraitConstManual;

    #[test]
    fn test_opencv_array() {
        let scaled: Vec<u8> = vec![1, 2, 3, 4, 5, 6];
        let mat = opencv::core::Mat::new_rows_cols_with_data(2, 3, &scaled).unwrap();
        let mut out_mat = opencv::core::Mat::new_rows_cols_with_default(
            mat.rows(),
            mat.cols(),
            opencv::core::CV_8UC1,
            opencv::core::Scalar::new(0.0, 0.0, 0.0, 0.0),
        )
        .unwrap();
        opencv::imgproc::threshold(
            &mat,
            &mut out_mat,
            2 as f64,
            1 as f64,
            opencv::imgproc::THRESH_BINARY,
        )
        .unwrap();
        let out_array = out_mat.data_bytes().unwrap();
        assert!(out_array[0] == 0);
        assert!(out_array[1] == 0);
        assert!(out_array[2] == 1);
        assert!(out_array[3] == 1);
        assert!(out_array[4] == 1);
        assert!(out_array[5] == 1);
    }
}
