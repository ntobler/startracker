
import { api, HelpDisplay, matToLaTeX, vecToLaTeX, parseSize, toF32Array } from './util.js';
import { mat_apply_vec, quat_to_mat, create_fast_params, fast_obj_to_pix } from './geom.js';


import { ref } from './vue.esm-browser.prod.min.js';
import { unpack } from './msgpackr.js'


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
                "Raw": "Processed", "Processed": "Crop2x", "Crop2x": "Motion", "Motion": "Raw",
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

            if (this.stream.image_type == "Crop2x") {
                ctx.scale(2, 2)
            }

            let s = Math.min(canvas.width / width, canvas.height / height)
            ctx.scale(s, s)
            ctx.translate(0.5 - width / 2, 0.5 - height / 2)

            ctx.lineWidth = 1
            ctx.strokeStyle = "red"
            ctx.save()

            ctx.beginPath();
            if (this.stream.image_type == "Crop2x") {
                ctx.rect(-0.5 + width / 4, -0.5 + height / 4, width / 2, height / 2);
            } else {
                ctx.rect(-0.5, -0.5, width, height);
            }
            ctx.clip();

            if (this.stream.auto_calibrator != {} && this.stream.auto_calibrator.active) {
                this.drawCelestialCoordinateFrame(ctx, this.stream.auto_calibrator);
            } else if (this.view_settings.coordinate_frame && this.stream.attitude_estimation?.n_matches > 0) {
                this.drawCelestialCoordinateFrame(ctx, this.stream.attitude_estimation);
            }

            if (this.stream.attitude_estimation) {
                this.drawStars(ctx, this.stream.attitude_estimation)
            }

            // if (this.stream.motion_xy) {
            //     this.drawMotion(ctx, this.stream.motion_quat)
            // }

            this.showAutoCalibrationInfo(this.stream.auto_calibrator);


            ctx.restore()

            ctx.beginPath();
            if (this.stream.image_type == "Crop2x") {
                ctx.rect(-1 + width / 4, -1 + height / 4, width / 2 + 1, height / 2 + 1);
            } else {
                ctx.rect(-1, -1, width + 1, height + 1);
            }
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
                    ctx.arc(obs_xy[obs_i2], obs_xy[obs_i2 + 1], 5, 0, 2 * Math.PI)
                    ctx.stroke()

                    // Connect observation with catalog position
                    ctx.beginPath()
                    ctx.moveTo(obs_xy[obs_i2], obs_xy[obs_i2 + 1])
                    ctx.lineTo(cat_xy[cat_i2], cat_xy[cat_i2 + 1])
                    ctx.stroke()

                    // Increase cat index
                    cat_i2 += 2;
                } else {
                    // Draw cross over observation
                    ctx.beginPath()
                    ctx.moveTo(obs_xy[obs_i2] - 2, obs_xy[obs_i2 + 1] - 2)
                    ctx.lineTo(obs_xy[obs_i2] + 2, obs_xy[obs_i2 + 1] + 2)
                    ctx.stroke()
                    ctx.beginPath()
                    ctx.moveTo(obs_xy[obs_i2] + 2, obs_xy[obs_i2 + 1] - 2)
                    ctx.lineTo(obs_xy[obs_i2] - 2, obs_xy[obs_i2 + 1] + 2)
                    ctx.stroke()
                }
            }
            ctx.restore()
        },
        drawMotion(ctx, motion_quat_raw) {

            const extrinsic = state.extrinsic;
            const center_vec = [extrinsic[7], extrinsic[8], extrinsic[9]]
            const fast_params = create_fast_params(state.intrinsic, state.dist_coeffs)


            if (motion_quat_raw === undefined) return
            let motion_quat = toF32Array(motion_quat_raw);
            ctx.save()
            ctx.lineCap = "round"
            ctx.beginPath()
            for (let i = 0; i < motion_quat.length; i += 4) {
                let quat = motion_quat.subarray(i, i + 4);
                let mat = quat_to_mat(quat);
                const camera_xyz = mat_apply_vec(mat, center_vec)
                const xy_dist = fast_obj_to_pix(camera_xyz, fast_params);
                ctx.lineTo(xy_dist[0], xy_dist[0])
            }
            ctx.stroke()
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

            // Pre-calculate variable used for culling
            const width = state.image_size[0];
            const height = state.image_size[1];
            const diagonal = Math.sqrt(width * width + height * height);
            const angle_margin_factor = 1.4;
            const cos_phi = Math.cos(angle_margin_factor * Math.atan(diagonal / intrinsic[0] / 2));
            const target_vector = [extrinsic[6], extrinsic[7], extrinsic[8]];
            const fast_params = create_fast_params(state.intrinsic, state.dist_coeffs)

            ctx.save()
            ctx.strokeStyle = "#333F"
            ctx.setLineDash([6, 6]);
            ctx.lineWidth = 1;

            // Keep track of line drawing state (whether a line has been stared or not)
            let drawing = false;

            // Define plot function for a point
            function plot_point(lat_rad, lon_rad) {
                const cos_lat = Math.cos(lat_rad);
                const x = Math.cos(lon_rad) * cos_lat;
                const y = Math.sin(lon_rad) * cos_lat;
                const z = Math.sin(lat_rad);

                // Check if point is roughly in frame
                if ((target_vector[0] * x + target_vector[1] * y + target_vector[2] * z) < cos_phi) {
                    if (drawing) {
                        ctx.stroke()
                        drawing = false;
                    }
                    return true;
                }

                const camera_xyz = mat_apply_vec(extrinsic, [x, y, z])
                const xy_dist = fast_obj_to_pix(camera_xyz, fast_params);

                // Draw line
                if (!drawing) {
                    ctx.beginPath();
                    ctx.moveTo(xy_dist[0], xy_dist[1]);
                    drawing = true;
                } else {
                    ctx.lineTo(xy_dist[0], xy_dist[1]);
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

        api('/api/set_settings', { send_image: true }, null);
        api('/api/get_state', null, this.updateState);

        getKatex(() => { });

        window.onresize = this.resize
        this.resize()
    }
}
