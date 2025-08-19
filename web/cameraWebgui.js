
import { api, HelpDisplay, matmul3x3, matToLaTeX, vecToLaTeX, parseSize, toF32Array } from './util.js';
import { ref } from './vue.esm-browser.prod.min.js';
import { pack, unpack } from './msgpackr.js'


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
            camera_settings: ref(undefined),
            attitude: ref(undefined),
            stream: ref(undefined),
            view_settings: ref(undefined),
            packet_size: ref("??"),
            image_size: ref("??"),
            image_quality: ref("??"),
            helpDisplay: null,
            shutdownCalls: ref([]),
            viewSettings: ref(false),
            cameraMode: ref(undefined),
            fullscreen: ref(undefined),
        }
    },
    computed: {
        exposureMs: {
            get() {
                return this.camera_settings.exposure_us / 1000;
            },
            set(newValue) {
                this.camera_settings.exposure_us = newValue * 1000;
            }
        }
    },
    methods: {
        showImage(data) {
            let img = document.getElementById('image');

            this.image_size = parseSize(data.length);

            // clear image if data not present
            if (data.length == 0) {
                img.style.display = "none"
                return;
            }

            // Find out mime type from the image header
            let mimeType = '';
            if (data[0] === 0x89 && data[1] === 0x50) {
                mimeType = 'image/png';
            } else if (data[0] === 0xFF && data[1] === 0xD8) {
                mimeType = 'image/jpeg';
            } else {
                console.error("Unknown image type");
                return;
            }

            // Free old image if present
            if (img.src) {
                URL.revokeObjectURL(img.src);
            }

            // Display image
            const blob = new Blob([data], { type: mimeType });
            img.src = URL.createObjectURL(blob);
            img.style.display = "block"
        },
        onmessage(response) {
            this.stream = unpack(response.data);
            this.packet_size = parseSize(response.data.byteLength);

            this.image_quality = this.stream.image_quality

            this.redraw()
            document.getElementById('footer-bar').style.display = "flex"

            this.showImage(this.stream.encoded_frame);
        },
        connectStreamWebSocket() {
            let url = 'ws://' + window.location.host + "/api/stream";
            let ws = new WebSocket(url);
            ws.binaryType = "arraybuffer"
            ws.onmessage = this.onmessage.bind(this);
            ws.onclose = function (event) {
                ws.close()
            };
            ws.onerror = function (event) {
                ws.close()
            };
            window.addEventListener("beforeunload", () => {
                ws.close(1000, "Page is unloading");
            });
        },
        setSettings() {
            let payload = {
                camera_config: this.camera_settings,
                view_settings: this.view_settings,
                attitude_est_config: this.attitude,
            }
            api('/api/set_settings', payload, this.updateState);
        },
        updateState(data) {
            this.camera_settings = data.persistent.camera_config
            this.attitude = data.persistent.attitude_est_config
            this.view_settings = data.persistent.view_settings
            this.cameraMode = data.camera_mode;

            const el = document.getElementById("toggle_cam");
            el.classList.remove("pending");

            this.redraw()
        },
        toggleCoordinateFrame() {
            this.view_settings.coordinate_frame = this.view_settings.coordinate_frame ? false : true
            this.setSettings()
        },
        toggleBrightness() {
            this.view_settings.brightness = {
                1: 2, 2: 4, 4: 1,
            }[Number(this.view_settings.brightness)];
            let img = document.getElementById('image');
            img.style.filter = `brightness(${this.view_settings.brightness})`
            this.setSettings()
        },
        toggleImageType() {
            this.view_settings.image_type = {
                "Raw": "Processed", "Processed": "Crop2x", "Crop2x": "Raw",
            }[this.view_settings.image_type];
            this.setSettings()
        },
        toggleImageTargetQuality() {
            this.view_settings.target_quality_kb = {
                20: 50, 50: 100, 100: 200, 200: 500, 500: 0, 0: 20
            }[this.view_settings.target_quality_kb];
            this.setSettings()
        },
        capture(mode) {
            document.getElementById("toggle_cam").classList.add("pending");
            let payload = mode;
            api('/api/capture', payload, this.updateState);
        },
        autoCalibration(cmd) {
            api('/api/auto_calibration', cmd, this.updateState);
        },
        resize() {
            let canvas = document.getElementById('canvas')
            canvas.width = document.body.clientWidth * window.devicePixelRatio
            canvas.height = document.body.clientHeight * window.devicePixelRatio
            this.redraw()
        },
        redraw() {

            if (!this.stream) return;

            let width = this.stream.image_size[0]
            let height = this.stream.image_size[1]

            let canvas = document.getElementById('canvas')
            let ctx = canvas.getContext("2d")

            ctx.setTransform(1, 0, 0, 1, 0, 0)
            ctx.clearRect(0, 0, canvas.width, canvas.height)
            ctx.save()
            ctx.lineWidth = 1
            ctx.fillStyle = "white";
            ctx.strokeStyle = "white";

            ctx.translate(canvas.width / 2, canvas.height / 2)

            let s = Math.min(canvas.width / width, canvas.height / height)
            ctx.scale(s, s)
            ctx.translate(0.5 - width / 2, 0.5 - height / 2)

            ctx.lineWidth = 1
            ctx.strokeStyle = "red"
            ctx.save()

            ctx.beginPath();
            ctx.rect(-0.5, -0.5, width, height);
            ctx.clip();

            if (this.view_settings && (this.view_settings?.image_type != "crop2x")) {

                if (this.stream.auto_calibrator != {} && this.stream.auto_calibrator.active) {
                    this.drawCelestialCoordinateFrame(ctx, this.stream.auto_calibrator);
                } else if (this.view_settings.coordinate_frame && this.stream.attitude_estimation?.n_matches > 0) {
                    this.drawCelestialCoordinateFrame(ctx, this.stream.attitude_estimation);
                }

                if (this.stream.attitude_estimation) {
                    this.drawStars(ctx, this.stream.attitude_estimation)
                }

                this.showAutoCalibrationInfo(this.stream.auto_calibrator);

            }

            ctx.restore()

            ctx.beginPath();
            ctx.rect(-1, -1, width + 1, height + 1);
            ctx.stroke()

            ctx.restore()
        },
        drawStars(ctx, state) {
            if (state.obs_xy === undefined) return

            let obs_matched_mask = state.obs_matched_mask;
            let obs_xy = toF32Array(state.obs_xy);
            let cat_xy = toF32Array(state.cat_xy);

            ctx.save()

            ctx.lineCap = "round"

            let cat_i2 = 0;
            for (let i = 0; i < obs_matched_mask.length; i++) {
                let obs_i2 = i * 2;

                if (obs_matched_mask[i] != 0) {
                    // Draw circle over observation
                    ctx.beginPath()
                    ctx.arc(obs_xy[obs_i2], obs_xy[obs_i2+1], 5, 0, 2 * Math.PI)
                    ctx.stroke()

                    // Connect observation with catalog position
                    ctx.beginPath()
                    ctx.moveTo(obs_xy[obs_i2], obs_xy[obs_i2+1])
                    ctx.lineTo(cat_xy[cat_i2], cat_xy[cat_i2+1])
                    ctx.stroke()

                    // Increase cat index
                    cat_i2 += 2;
                } else {
                    // Draw cross over observation
                    ctx.beginPath()
                    ctx.moveTo(obs_xy[obs_i2] - 2, obs_xy[obs_i2+1] - 2)
                    ctx.lineTo(obs_xy[obs_i2] + 2, obs_xy[obs_i2+1] + 2)
                    ctx.stroke()
                    ctx.beginPath()
                    ctx.moveTo(obs_xy[obs_i2] + 2, obs_xy[obs_i2+1] - 2)
                    ctx.lineTo(obs_xy[obs_i2] - 2, obs_xy[obs_i2+1] + 2)
                    ctx.stroke()
                }
            }
            ctx.restore()
        },
        showAutoCalibrationInfo(state) {
            if (state == {} || !state.active) return;

            const el = document.getElementById("matrix");
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

                // Distortion
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
        this.connectStreamWebSocket();

        api('/api/set_settings', {send_image: true}, null);
        api('/api/get_state', null, this.updateState);

        getKatex(() => { });

        window.onresize = this.resize
        this.resize()
    }
}
