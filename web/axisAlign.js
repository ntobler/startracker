
import ZoomHandler from './zoomHandler.js';

var state = null
var ui_zoom = 0.8
const R_MARGIN = 0.5


function resize() {
    let canvas = document.getElementById('canvas')
    canvas.width = document.body.clientWidth * window.devicePixelRatio
    canvas.height = document.body.clientHeight * window.devicePixelRatio
    redraw()
}


const zoomHandler = new ZoomHandler(0.25, 5, (zoom) => {
    ui_zoom = zoom
    redraw()
});

window.addEventListener('DOMContentLoaded', () => {
    window.onresize = resize
    resize()
    zoomHandler.attachEventListeners(document.getElementById('canvas'))
})

function redraw() {

    const scale = 50

    ui_zoom = zoomHandler.zoomLevel

    let canvas = document.getElementById('canvas')

    if (state === null) return

    let ctx = canvas.getContext("2d")


    ctx.save()
    ctx.setTransform(1, 0, 0, 1, 0, 0)
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    ctx.lineWidth = 1
    ctx.fillStyle = "white";
    ctx.strokeStyle = "white";

    let s = Math.min(canvas.height, canvas.width) / scale
    ctx.translate(canvas.width / 2, canvas.height / 2)


    ctx.lineWidth = 4


    drawCross(ctx, canvas.width / 2, canvas.height / 2)

    ctx.scale(s, s)


    ctx.lineWidth = 10 / scale
    ctx.font = "1px Consolas";

    drawRings(ctx)

    drawCameraFrame(ctx)

    ctx.lineWidth = 2 / scale

    drawStars(ctx)

    ctx.restore()
}

function drawCross(ctx, w, h) {
    ctx.save()
    ctx.strokeStyle = "#333"
    ctx.beginPath()
    ctx.moveTo(-w, 0)
    ctx.lineTo(w, 0)
    ctx.stroke()
    ctx.moveTo(0, -h)
    ctx.lineTo(0, h)
    ctx.stroke()
    ctx.restore()
}

function drawRings(ctx) {
    const ANGLE = Math.PI / 180 * -30
    const C = Math.cos(ANGLE)
    const S = Math.sin(ANGLE)
    ctx.save()

    ctx.strokeStyle = "#333F"
    ctx.fillStyle = "#333F"
    for (let deg of [0.5, 1, 2, 5, 10, 20, 30, 45, 60, 90]) {
        let r = deg * ui_zoom
        let f = Math.min(1, Math.max(0, (r - 0.8) * 0.4))
        if (f == 0) continue;
        let c = `rgba(40, 40, 40, ${f})`;
        ctx.strokeStyle = c
        ctx.fillStyle = c

        ctx.beginPath()
        ctx.arc(0, 0, r, 0, 2 * Math.PI)
        ctx.stroke()
        ctx.fillText(deg + "°", C * (r + R_MARGIN), S * (r + R_MARGIN));
    }
    ctx.restore()
}

function drawCameraFrame(ctx) {

    ctx.save()
    ctx.strokeStyle = "red"
    ctx.fillStyle = "red"

    let x = state.frame_points[0]
    let y = state.frame_points[1]
    ctx.fillText("Camera frame", x[0] * ui_zoom, y[0] * ui_zoom - R_MARGIN);
    ctx.beginPath()
    ctx.moveTo(x[0] * ui_zoom, y[0] * ui_zoom)
    for (let i = 1; i < x.length; i++) {
        ctx.lineTo(x[i] * ui_zoom, y[i] * ui_zoom)
    }
    ctx.closePath()
    ctx.stroke()

    ctx.restore()
}

function drawStars(ctx) {
    ctx.save()

    for (let i = 0; i < state.cat_xyz.length; i++) {
        let coord = state.cat_xyz[i]
        let mag = state.cat_mags[i]
        mag = Math.pow(100, (-mag / 5 / 2)) * ui_zoom * 0.7
        ctx.beginPath()
        ctx.arc(coord[0] * ui_zoom, coord[1] * ui_zoom, mag, 0, 2 * Math.PI)
        ctx.fill()
    }

    ctx.strokeStyle = "red"
    ctx.fillStyle = "red"

    for (let i = 0; i < state.star_coords.length; i++) {
        let coord = state.star_coords[i]
        ctx.beginPath()
        ctx.arc(coord[0] * ui_zoom, coord[1] * ui_zoom, ui_zoom, 0, 2 * Math.PI)
        ctx.stroke()
    }

    ctx.save()
    ctx.strokeStyle = "blue"
    const w = 10
    const h = 10
    for (let ns of state.north_south) {
        ctx.translate(ns[0] * ui_zoom, ns[1] * ui_zoom)
        ctx.beginPath()
        ctx.moveTo(-w, 0)
        ctx.lineTo(w, 0)
        ctx.stroke()
        ctx.moveTo(0, -h)
        ctx.lineTo(0, h)
        ctx.stroke()

        ctx.translate(0, -h)
        ctx.rotate(Math.PI / 2)
        ctx.fillText("North", 0, 0)
        ctx.restore()
    }

    ctx.restore()
}


function parseSize(size) {
    let unit
    for (unit of ["B", "kB", "MB", "GB"]) {
        if (size < 10) return size.toFixed(1) + unit
        if (size < 100) return size.toFixed(0) + unit
        size /= 1024
    }
}

let url = 'ws://' + window.location.host + "/state"
var ws = new WebSocket(url);
ws.onmessage = response => {
    state = JSON.parse(response.data)
    redraw()
    document.getElementById("trackedStars").innerHTML = `Tracked stars: ${state.n_matches}`
    document.getElementById("alignmentError").innerHTML = `Alignment error: ${state.alignment_error.toFixed(3)}°`
    document.getElementById("processingTime").innerHTML = `Processing time: ${state.processing_time}ms`
    document.getElementById("packetSize").innerHTML = `Packet size: ${parseSize(response.data.length)}`
}

document.getElementById('canvas').onclick = () => {
    payload = {
        action: "dunno"
    }
    fetch('/test', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    }).then(response => response.json()).then((data) => {
    }).catch(error => {
        console.error('Error:', error);
    });
}
