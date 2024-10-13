
import { ZoomHandler, api } from './util.js';
import { ref } from './vue.esm-browser.prod.min.js';

export default {
    setup() {
        return {
            stream: ref({
                quat: [],
                obs_pix: [],
                n_matches: 0,
                pre_processing_time: 0,
                processing_time: 0,
                post_processing_time: 0,
                alignment_error: 0,
                image_size: [960, 540],
                frame_points: [],
            }),
            packet_size: ref("??"),
            ui_zoom: 0.8,
            calibrating: false,
            calibration_orientations: 0,
            history: [],
        }
    },
    methods: {
        onmessage(response) {
            this.stream = JSON.parse(response.data)
            this.packet_size = parseSize(response.data.length),
                this.history.push({
                    n_matches: this.stream.n_matches,
                    processing_time: this.stream.processing_time,
                })
            if (this.history.length > 20) this.history.shift();
            this.redraw()
        },
        connectWebSocket() {
            let url = 'ws://' + window.location.host + "/api/stream";
            let ws = new WebSocket(url);
            ws.onmessage = this.onmessage.bind(this);
        },
        resize() {
            let canvas = document.getElementById('canvas')
            canvas.width = document.body.clientWidth * window.devicePixelRatio
            canvas.height = document.body.clientHeight * window.devicePixelRatio
            this.redraw()
        },
        redraw() {
            const scale = 50

            let state = this.stream
            let ui_zoom = this.ui_zoom

            let canvas = document.getElementById('canvas')

            if (state === null) return

            let ctx = canvas.getContext("2d")

            ctx.setTransform(1, 0, 0, 1, 0, 0)
            ctx.clearRect(0, 0, canvas.width, canvas.height)
            ctx.save()
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
            drawRings(ctx, ui_zoom)
            drawCameraFrame(ctx, state, ui_zoom)

            ctx.lineWidth = 2 / scale
            drawStars(ctx, state, ui_zoom)

            ctx.restore()

            drawPlot(ctx, this.history)

        },
        addToCalibration() {
            let self = this
            api('/api/axis_calibration', { command: "put" }, data => {
                self.calibration_orientations = data.calibration_orientations
            });
        },
        resetCalibration() {
            let self = this
            api('/api/axis_calibration', { command: "reset" }, data => {
                self.calibration_orientations = data.calibration_orientations
            });
        },
        calibrate() {
            if (this.calibration_orientations < 1) return;
            this.calibrating = true;
            let self = this
            api(
                '/api/axis_calibration', { command: "calibrate" },
                data => { self.calibrating = false; },
                error => { self.calibrating = true; },
            );
        },

    },
    mounted() {
        this.connectWebSocket();

        window.onresize = this.resize
        this.resize()

        const zoomHandler = new ZoomHandler(0.25, 5, (zoom) => {
            this.ui_zoom = zoom
            this.redraw()
        });
        zoomHandler.attachEventListeners(document.getElementById('canvas'))
    }
}

const R_MARGIN = 0.5

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

function drawRings(ctx, ui_zoom) {
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

function drawCameraFrame(ctx, state, ui_zoom) {

    ctx.save()
    if (state.n_matches > 0) {
        ctx.strokeStyle = "red"
        ctx.fillStyle = "red"
    } else {
        ctx.strokeStyle = "#f004"
        ctx.fillStyle = "#f004"
    }

    if (state.frame_points.length >= 2) {
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
    }

    ctx.restore()
}

function drawStars(ctx, state, ui_zoom) {

    if (state.cat_xyz === undefined) return

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
    const w = 10
    const h = 10
    for (let i of [0, 1]) {
        let pos = state.north_south[i]
        let name = ["north", "south"][i]
        let color = ["red", "blue"][i]

        let x = pos[0]
        let y = pos[1]

        if (x * x + y * y > 90 * 90) continue;

        ctx.save()

        ctx.strokeStyle = color
        ctx.fillStyle = color

        ctx.translate(x * ui_zoom, y * ui_zoom)
        ctx.beginPath()
        ctx.moveTo(-w, 0)
        ctx.lineTo(w, 0)
        ctx.stroke()
        ctx.moveTo(0, -h)
        ctx.lineTo(0, h)
        ctx.stroke()

        ctx.translate(0, -h)
        ctx.rotate(Math.PI / 2)
        ctx.fillText(name, 0, 0)

        ctx.restore()
    }

    ctx.restore()
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

function drawPlot(ctx, history) {

    let width = 300
    let height = 200

    ctx.strokeStyle = "red"
    ctx.fillStyle = "red"

    ctx.save()

    ctx.translate(20, 20)

    ctx.lineWidth = 3

    ctx.save()
    ctx.beginPath();
    ctx.rect(0, 0, width, height);
    ctx.clip();
    for (let [label, color, lw] of [["n_matches", "red", 3]]) {

        if (history.length < 1) break;

        ctx.save()
        ctx.lineWidth = lw
        ctx.strokeStyle = color
        ctx.fillStyle = color
        ctx.globalAlpha = 0.4;
        ctx.save()

        ctx.translate(0, height)
        ctx.scale(width / (history.length - 1), -height / 40)
        ctx.beginPath();
        for (let i = 0; i < history.length; i++) {
            ctx.lineTo(i, history[i][label])
        }
        ctx.lineTo(history.length - 1, 0)
        ctx.lineTo(0, 0)
        ctx.restore()
        ctx.fill()
        ctx.globalAlpha = 1;
        ctx.stroke()
        ctx.restore()
    }
    ctx.restore()

    ctx.lineWidth = 4
    ctx.strokeStyle = "red"
    ctx.beginPath();
    ctx.rect(0, 0, width, height);
    ctx.stroke()

    ctx.restore()
}
