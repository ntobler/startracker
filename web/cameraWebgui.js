
import { api, HelpDisplay, matmul3x3, matToLaTeX, vecToLaTeX } from './util.js';
import { ref } from './vue.esm-browser.prod.min.js';

let katexPromise;

function getKatex(callback) {
    if (!katexPromise) {
        katexPromise = import('./katex.mjs');
    }
    katexPromise.then((katex) => {
        callback(katex.default);
    });
}

export default {
    setup() {
        return {
            camera_settings: ref({
                exposure_ms: 1,
                analog_gain: 2,
                digital_gain: 3,
                binning: 1,
            }),
            intrinsic_calibrator: ref({
                index: 1,
                pattern_width: 1,
                pattern_height: 1,
                pattern_size: 1,
            }),
            attitude: ref({
                min_matches: 12,
                pixel_tolerance: 3.0,
                timeout_secs: 3.0,
            }),
            stream: ref({
                pre_processing_time: 0,
                attitude_estimation: {
                    quat: [],
                    rotm: [],
                    obs_pix: [],
                    cat_pix: [],
                    n_matches: 0,
                    processing_time: 0,
                    post_processing_time: 0,
                    image_size: [960, 540],
                    extrinsic: [],
                    intrinsic: [],
                    dist_coeffs: [],
                },
                auto_calibrator: {},
            }),
            celestial_coordinate_frame_overlay: false,
            brightness: ref(1),
            helpDisplay: null,
        }
    },
    methods: {
        connectImageWebSocket() {
            let url = 'ws://' + window.location.host + "/api/image"
            var ws = new WebSocket(url);
            var img = document.getElementById('image');
            ws.onmessage = function (event) {
                var blob = new Blob([event.data], { type: 'image/png' });
                img.src = URL.createObjectURL(blob);
                img.style.display = "block"
            };
        },
        onmessage(response) {
            this.stream = JSON.parse(response.data);

            this.redraw()
            document.getElementById('footer-bar').style.display = "flex"
        },
        connectStreamWebSocket() {
            let url = 'ws://' + window.location.host + "/api/stream";
            let ws = new WebSocket(url);
            ws.onmessage = this.onmessage.bind(this);
        },
        setSettings() {
            let payload = {
                exposure_ms: this.camera_settings.exposure_ms,
                analog_gain: this.camera_settings.analog_gain,
                digital_gain: this.camera_settings.digital_gain,
                binning: this.camera_settings.binning,
                pattern_width: this.intrinsic_calibrator.pattern_width,
                pattern_height: this.intrinsic_calibrator.pattern_height,
                pattern_size: this.intrinsic_calibrator.pattern_size,
                min_matches: this.attitude.min_matches,
                pixel_tolerance: this.attitude.pixel_tolerance,
                timeout_secs: this.attitude.timeout_secs,
            }
            api('/api/set_settings', payload, this.updateState);
        },
        updateState(data) {
            this.camera_settings = data.camera_settings
            this.intrinsic_calibrator = data.intrinsic_calibrator
            this.attitude = data.attitude

            const el = document.getElementById("toggle_cam");
            el.classList.remove("pending");
            if (data.camera_mode == "continuous") {
                el.innerHTML = "Stop"
            } else {
                el.innerHTML = "Start"
            }

            this.redraw()
        },
        toggleCelestialCoordinateFrameOverlay() {
            let el = document.getElementById("overlay")
            this.celestial_coordinate_frame_overlay = this.celestial_coordinate_frame_overlay ? false : true
            el.classList = this.celestial_coordinate_frame_overlay ? ["active"] : []
            this.setSettings()
        },
        toggleBrightness() {
            let brightness = {
                1: 2, 2: 4, 4: 1,
            }[Number(this.brightness)];
            let img = document.getElementById('image');
            img.style.filter = `brightness(${brightness})`
            this.brightness = brightness
        },
        capture(mode) {
            document.getElementById("toggle_cam").classList.add("pending");
            let payload = { mode: mode };
            api('/api/capture', payload, this.updateState);
        },
        calibrationPut() {
            api('/api/camera_calibration', { command: "put" }, this.updateState);
        },
        calibrationReset() {
            api('/api/camera_calibration', { command: "reset" }, this.updateState);
        },
        calibrationCalibrate() {
            if (this.intrinsic_calibrator.index < 1) return;
            document.getElementById("calibrate").classList.add("pending");
            api('/api/camera_calibration', { command: "calibrate" }, (state) => {
                this.updateState(state);
                document.getElementById("calibrate").classList.remove("pending");
            });
        },
        calibrationAccept() {
            api('/api/camera_calibration', { command: "accept" }, this.updateState);
        },
        autoCalibration(cmd) {
            api('/api/auto_calibration', { command: cmd }, this.updateState);
        },
        resize() {
            let canvas = document.getElementById('canvas')
            canvas.width = document.body.clientWidth * window.devicePixelRatio
            canvas.height = document.body.clientHeight * window.devicePixelRatio
            this.redraw()
        },
        redraw() {
            let state = this.stream.attitude_estimation
            if (!state) return

            let canvas = document.getElementById('canvas')
            let ctx = canvas.getContext("2d")

            ctx.setTransform(1, 0, 0, 1, 0, 0)
            ctx.clearRect(0, 0, canvas.width, canvas.height)
            ctx.save()
            ctx.lineWidth = 1
            ctx.fillStyle = "white";
            ctx.strokeStyle = "white";

            ctx.translate(canvas.width / 2, canvas.height / 2)

            let width = state.image_size[0]
            let height = state.image_size[1]

            let s = Math.min(canvas.width / width, canvas.height / height)
            ctx.scale(s, s)
            ctx.translate(0.5 - width / 2, 0.5 - height / 2)

            ctx.lineWidth = 1
            ctx.strokeStyle = "red"
            ctx.save()

            ctx.beginPath();
            ctx.rect(-0.5, -0.5, width, height);
            ctx.clip();

            this.drawStars(ctx, state)

            this.showAutoCalibrationInfo(this.stream.auto_calibrator);

            if (this.stream.auto_calibrator != {} && this.stream.auto_calibrator.active) {
                this.drawCelestialCoordinateFrame(ctx, this.stream.auto_calibrator);
            } else if (this.celestial_coordinate_frame_overlay && this.stream.attitude_estimation.n_matches > 0) {
                this.drawCelestialCoordinateFrame(ctx, this.stream.attitude_estimation);
            }

            ctx.restore()

            ctx.beginPath();
            ctx.rect(-1, -1, width + 1, height + 1);
            ctx.stroke()

            ctx.restore()
        },
        drawStars(ctx, state) {
            if (state.obs_pix === undefined) return
            let obs_pix = state.obs_pix;
            let cat_pix = state.cat_pix;
            ctx.save()

            ctx.lineCap = "round"

            for (let i = 0; i < Math.min(obs_pix.length, cat_pix.length); i++) {
                let coord = obs_pix[i]
                ctx.beginPath()
                ctx.arc(coord[0], coord[1], 5, 0, 2 * Math.PI)
                ctx.stroke()

                ctx.beginPath()
                ctx.moveTo(coord[0], coord[1])
                coord = cat_pix[i]
                ctx.lineTo(coord[0], coord[1])
                ctx.stroke()
            }
            ctx.restore()
        },
        showAutoCalibrationInfo(state) {
            const el = document.getElementById("matrix");
            if (state == {} || !state.active) {
                el.style.display = "none"
                return;
            }
            el.style.display = "flex"

            getKatex((katex) => {
                katex.render(String.raw`
                    \begin{aligned}
                        \text{State} = \text{${state.state}} \\
                        \text{RMS error} = \mathrm{${state.rms_error}} \\
                        \text{Max error} = \mathrm{${state.max_error}} \\
                        \text{Intrinsic} = ${matToLaTeX(state.intrinsic, 2)} \\
                        \text{Dist coeffs} = ${vecToLaTeX(state.dist_coeffs, 3)}
                    \end{aligned}
                `, el, {
                    throwOnError: true,
                    displayMode: true,
                    output: "mathml",
                });
            });
        },
        drawCelestialCoordinateFrame(ctx, state) {

            const extrinsic = state.extrinsic;
            const intrinsic = state.intrinsic;
            const dist_coeffs = state.dist_coeffs;

            // Return early if data is missing
            if (extrinsic.length == 0 || intrinsic.length == 0 || dist_coeffs.length == 0) {
                return;
            }

            ctx.save()
            ctx.strokeStyle = "#333F"
            ctx.setLineDash([6, 6]);
            ctx.lineWidth = 1;

            // Extract intrinsic parameters
            const projection_matrix = matmul3x3(intrinsic, extrinsic);
            const fx = intrinsic[0][0];
            const fy = intrinsic[1][1];
            const tx = intrinsic[0][2];
            const ty = intrinsic[1][2];
            const k1 = dist_coeffs[0];
            const k2 = dist_coeffs[1];
            const p1 = dist_coeffs[2];
            const p2 = dist_coeffs[3];
            const k3 = dist_coeffs[4];

            // Pre-calculate variable used for culling
            const width = state.image_size[0];
            const height = state.image_size[1];
            const diagonal = Math.sqrt(width * width + height * height);
            const angle_margin_factor = 1.4;
            const cos_phi = Math.cos(angle_margin_factor * Math.atan(diagonal / intrinsic[0][0] / 2));
            const target_vector = extrinsic[2];

            // Keep track of line drawing state (whether a line has been stared or not)
            let drawing = false;

            // Define plot function for a point
            function plot_point(lat_rad, lon_rad) {
                const cos_lat = Math.cos(lat_rad);
                let x = Math.cos(lon_rad) * cos_lat;
                let y = Math.sin(lon_rad) * cos_lat;
                const z = Math.sin(lat_rad);

                // Check if point is roughly in frame
                if ((target_vector[0] * x + target_vector[1] * y + target_vector[2] * z) < cos_phi) {
                    if (drawing) {
                        ctx.stroke()
                        drawing = false;
                    }
                    return true;
                }

                // Perspective projection
                const p = projection_matrix
                const diviser = p[2][0] * x + p[2][1] * y + p[2][2] * z;
                const img_x = (p[0][0] * x + p[0][1] * y + p[0][2] * z) / diviser;
                const img_y = (p[1][0] * x + p[1][1] * y + p[1][2] * z) / diviser;

                // // Distortion
                x = (img_x - tx) / fx
                y = (img_y - ty) / fy
                const r2 = x ** 2 + y ** 2
                const r4 = r2 * r2
                const r6 = r2 * r4
                const d = 1 + k1 * r2 + k2 * r4 + k3 * r6
                let x_dist = x * d + (2 * p1 * x * y + p2 * (r2 + 2 * x ** 2))
                let y_dist = y * d + (2 * p2 * x * y + p1 * (r2 + 2 * y ** 2))
                x_dist = (x_dist * fx) + tx
                y_dist = (y_dist * fy) + ty

                // Draw line
                if (!drawing) {
                    ctx.beginPath();
                    ctx.moveTo(x_dist, y_dist);
                    drawing = true;
                } else {
                    ctx.lineTo(x_dist, y_dist);
                }
                return false
            }

            // Plot latitudinal lines
            for (let lat = -85; lat <= 85; lat += 5) {
                const lat_rad = lat * (Math.PI / 180);
                if (lat % 15 == 0) ctx.setLineDash([]); else ctx.setLineDash([6, 6]);
                drawing = false;
                for (let lon = 0; lon <= 360; lon += 2) {
                    const lon_rad = lon * (Math.PI / 180);
                    if (plot_point(lat_rad, lon_rad)) continue;
                }
                if (drawing) {
                    ctx.stroke()
                }
            }
            // Plot longitudinal lines
            for (let lon = 0; lon < 360; lon += 5) {
                const lon_rad = lon * (Math.PI / 180);
                let min, max;
                if (lon % 15 == 0) {
                    ctx.setLineDash([]);
                    min = -88;
                    max = 88;
                } else {
                    ctx.setLineDash([6, 6]);
                    min = -75;
                    max = 75;
                }
                drawing = false;
                for (let lat = min; lat <= max; lat += 2) {
                    const lat_rad = lat * (Math.PI / 180);
                    if (plot_point(lat_rad, lon_rad)) continue;
                }
                if (drawing) {
                    ctx.stroke()
                }
            }

            ctx.restore()
        },
        showHelp() {
            if (this.helpDisplay === null) return
            this.helpDisplay.toggleHelp()
        },
    },
    mounted() {
        this.connectImageWebSocket();
        this.connectStreamWebSocket();

        api('/api/get_state', null, this.updateState);

        getKatex(() => { });

        window.onresize = this.resize
        this.resize()
        this.helpDisplay = new HelpDisplay(document.getElementById('footer-bar'))
    }
}
