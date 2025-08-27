
import { ZoomHandler, api, HelpDisplay, parseSize, toF32Array } from './util.js';
import { ref } from './vue.esm-browser.prod.min.js';
import { pack, unpack } from './msgpackr.js'

export default {
    setup() {
        return {
            stream: ref(undefined),
            packet_size: ref("??"),
            ui_zoom: 0.8,
            calibration_orientations: 0,
            history: [],
            helpDisplay: null,
            shutdownCalls: ref([]),
            viewSettings: ref(false),
            cameraMode: ref(undefined),
            fullscreen: ref(undefined),
        }
    },
    methods: {
        onmessage(response) {
            this.stream = unpack(response.data);
            this.packet_size = parseSize(response.data.byteLength);

            let attitude_estimation = this.stream.attitude_estimation;
            if (attitude_estimation) {
                this.history.push({
                    n_matches: attitude_estimation.n_matches,
                    processing_time: attitude_estimation.processing_time,
                })
                if (this.history.length > 20) this.history.shift();
            }

            this.redraw()
            document.getElementById('footer-bar').style.display = "flex"
        },
        connectWebSocket() {
            let url = 'ws://' + window.location.host + "/api/stream";
            let ws = new WebSocket(url);
            ws.binaryType = "arraybuffer"
            ws.onmessage = this.onmessage.bind(this);
            window.addEventListener("beforeunload", () => {
                ws.close(1000, "Page is unloading");
            });
        },
        updateState(data) {
            this.calibration_orientations = data.axis_calibration.orientations;
            this.cameraMode = data.camera_mode;

            const el = document.getElementById("toggle_cam");
            el.classList.remove("pending");
        },
        resize() {
            let canvas = document.getElementById('canvas')
            canvas.width = document.body.clientWidth * window.devicePixelRatio
            canvas.height = document.body.clientHeight * window.devicePixelRatio
            this.redraw()
        },
        redraw() {
            const scale = 50

            let ui_zoom = this.ui_zoom
            let canvas = document.getElementById('canvas')
            let ctx = canvas.getContext("2d")

            let font_size = parseFloat(window.getComputedStyle(document.body).fontSize) * window.devicePixelRatio;
            ctx.font = `${font_size}px Consolas, monospace`;

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
            ctx.font = `${font_size / s}px Consolas, monospace`;
            drawRings(ctx, ui_zoom);

            if (this.stream?.attitude_estimation) {
                drawCameraFrame(ctx, this.stream.attitude_estimation, ui_zoom)

                ctx.lineWidth = 2 / scale
                drawStars(ctx, this.stream.attitude_estimation, ui_zoom)
            }

            ctx.restore()

            drawPlot(ctx, this.history)
        },
        addToCalibration() {
            api('/api/axis_calibration', "Put", this.updateState);
        },
        resetCalibration() {
            api('/api/axis_calibration', "Reset", this.updateState);
        },
        calibrate() {
            if (this.calibration_orientations < 1) return;
            document.getElementById("calibrate").classList.add("pending");
            api('/api/axis_calibration', "Calibrate", (state) => {
                this.updateState(state);
                document.getElementById("calibrate").classList.remove("pending");
            });
        },
        capture(mode) {
            document.getElementById("toggle_cam").classList.add("pending");
            api('/api/capture', mode, this.updateState);
        },
        showHelp() {
            this.helpDisplay = new HelpDisplay(document.getElementById('footer-bar'))
            this.helpDisplay.toggleHelp()
        },
        triggerShutdown() {
            const now = Date.now();
            this.shutdownCalls = this.shutdownCalls.filter(ts => now - ts < 5000);
            this.shutdownCalls.push(now);
            if (this.shutdownCalls.length >= 3) {
                this.shutdownCalls = [];
                api('/api/shutdown', { shutdown: "shutdown" }, () => { });
            }
        },
        toggleSettings() {
            this.viewSettings = this.viewSettings ? false : true;
        },
        toggleFullscreen() {
            const el = document.documentElement
            if (!document.fullscreenElement) {
                el.requestFullscreen()
            } else {
                document.exitFullscreen()
            }
            this.fullscreen = !document.fullscreenElement;
        },
    },
    mounted() {
        this.connectWebSocket();

        api('/api/set_settings', {send_image: false}, null);
        api('/api/get_state', null, this.updateState);

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

    let frame_points = toF32Array(state.frame_points_radial);

    if (frame_points.length >= 2) {
        ctx.beginPath()
        ctx.moveTo(frame_points[0] * ui_zoom, frame_points[1] * ui_zoom)
        for (let i2 = 2; i2 < frame_points.length; i2 += 2) {
            ctx.lineTo(frame_points[i2] * ui_zoom, frame_points[i2 + 1] * ui_zoom)
        }
        ctx.closePath()
        ctx.stroke()
    }

    ctx.restore()
}

function drawStars(ctx, state, ui_zoom) {

    if (state.cat_radial === undefined) return

    ctx.save()

    let cat_radial = toF32Array(state.cat_radial);
    let cat_mags = toF32Array(state.cat_mags);

    for (let i = 0; i < cat_mags.length; i++) {
        let mag = cat_mags[i]
        mag = Math.pow(100, (-mag / 5 / 2)) * ui_zoom * 0.7
        ctx.beginPath()
        ctx.arc(cat_radial[i*2] * ui_zoom, cat_radial[i*2 + 1] * ui_zoom, mag, 0, 2 * Math.PI)
        ctx.fill()
    }

    // Draw matched stars
    ctx.strokeStyle = "red"
    ctx.fillStyle = "red"

    let matched_obs_radial = toF32Array(state.matched_obs_radial);

    for (let i2 = 0; i2 < matched_obs_radial.length; i2 += 2) {
        ctx.beginPath()
        ctx.arc(matched_obs_radial[i2] * ui_zoom, matched_obs_radial[i2 + 1] * ui_zoom, ui_zoom, 0, 2 * Math.PI)
        ctx.stroke()
    }

    // Draw poles
    const w = 10
    const h = 10

    let north_south = toF32Array(state.north_south);
    for (let i of [0, 1]) {
        let name = ["North", "South"][i]

        let x = north_south[i*2 + 0]
        let y = north_south[i*2 + 1]

        if (x * x + y * y > 90 * 90) continue;

        ctx.save()

        ctx.translate(x * ui_zoom, y * ui_zoom)
        ctx.beginPath()
        ctx.moveTo(-w, 0)
        ctx.lineTo(w, 0)
        ctx.stroke()
        ctx.moveTo(0, -h)
        ctx.lineTo(0, h)
        ctx.stroke()

        ctx.translate(0.2, -h)
        ctx.rotate(Math.PI / 2)
        ctx.fillText(name, 0, 0)

        ctx.restore()
    }

    ctx.restore()
}

function drawPlot(ctx, history) {

    if (history.length == 0) {
        return
    }

    let width = 300
    let height = 200

    ctx.strokeStyle = "red"
    ctx.fillStyle = "red"

    ctx.save()

    let em = parseFloat(ctx.font) / 0.8;
    ctx.translate(em, em)

    ctx.save()
    ctx.lineWidth = 3
    ctx.beginPath();
    ctx.rect(0, 0, width, height);
    ctx.clip();



    ctx.lineWidth = 3
    ctx.strokeStyle = "red"
    ctx.fillStyle = "red"
    ctx.save()
    ctx.translate(0, height)
    ctx.scale(width / (history.length - 1), -height)
    ctx.scale(1, 1 / ( Math.log10(100) - Math.log10(1)))
    ctx.translate(0, -Math.log10(1))
    ctx.beginPath();
    for (let i = 0; i < history.length; i++) {
        ctx.lineTo(i, Math.max(Math.log10(history[i]["n_matches"]), 1))
    }
    ctx.restore()
    ctx.globalAlpha = 1;
    ctx.stroke()
    ctx.globalAlpha = 0.2;
    ctx.lineTo(width, height)
    ctx.lineTo(0, height)
    ctx.fill()

    ctx.globalAlpha = 1;
    ctx.strokeStyle = "gray"
    for (let base of [1, 10, 100]) {
        for (let digit of [1, 2, 3, 4, 5, 6, 7, 8, 9]) {
            let log_y = Math.log10(base * digit);
            if (Math.abs(Math.round(log_y) - log_y) < 0.0001) {
                ctx.lineWidth = 1.5
            } else {
                ctx.lineWidth = 0.5
            }
            ctx.save()
            ctx.translate(0, height)
            ctx.scale(width, -height)
            ctx.scale(1, 1 / ( Math.log10(100) - Math.log10(1)))
            ctx.translate(0, -Math.log10(1))
            ctx.beginPath();
            ctx.moveTo(0, log_y);
            ctx.lineTo(1, log_y);
            ctx.restore()
            ctx.stroke()
        }
    }
    ctx.restore()

    ctx.lineWidth = 4
    ctx.strokeStyle = "red"
    ctx.beginPath();
    ctx.rect(0, 0, width, height);
    ctx.stroke()

    let font_size = parseFloat(ctx.font);
    ctx.fillText(`Matches: ${history[history.length - 1]["n_matches"]}`, font_size / 2, height - font_size / 2)

    ctx.restore()
}
