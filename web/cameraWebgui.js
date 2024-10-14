
import { api } from './util.js';
import { ref } from './vue.esm-browser.prod.min.js';

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
                overlay: 1,
                min_matches: 12,
                pixel_tolerance: 3.0,
                timeout_secs: 3.0,
            }),
            stream: ref({
                quat: [],
                obs_pix: [],
                cat_pix: [],
                n_matches: 0,
                pre_processing_time: 0,
                processing_time: 0,
                post_processing_time: 0,
                image_size: [960, 540],
            }),
            brightness: ref(1),
            calibrating: false,
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
            this.stream = JSON.parse(response.data)
            this.redraw()
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
                overlay: this.attitude.overlay,
                min_matches: this.attitude.min_matches,
                pixel_tolerance: this.attitude.pixel_tolerance,
                timeout_secs: this.attitude.timeout_secs,
            }
            console.log(payload)
            api('/api/set_settings', payload, this.updateState);
        },
        updateState(data) {
            this.camera_settings = data.camera_settings
            this.intrinsic_calibrator = data.intrinsic_calibrator
            this.attitude = data.attitude
            this.redraw()
        },
        toggleOverlay() {
            let el = document.getElementById("overlay")
            this.attitude.overlay = this.attitude.overlay ? false : true
            el.classList = this.attitude.overlay ? ["active"] : []
            this.setSettings()
        },
        toggleBrightness() {
            let brightness = {
                1: 2, 2: 4, 4: 1,
            }[Number(this.brightness)];
            console.log(brightness)
            let img = document.getElementById('image');
            img.style.filter = `brightness(${brightness})`
            this.brightness = brightness
        },
        capture(mode) {
            let payload = { mode: mode };
            api('/api/capture', payload, this.updateState);
        },
        putCalibrationImage() {
            api('/api/camera_calibration', { command: "put" }, this.updateState);
        },
        resetCalibration() {
            api('/api/camera_calibration', { command: "reset" }, this.updateState);
        },
        calibrate() {
            if (this.intrinsic_calibrator.index < 1) return;
            this.calibrating = true;
            api(
                '/api/camera_calibration', { command: "calibrate" },
                data => { this.calibrating = false; },
                error => { this.calibrating = true; },
            );
        },
        resize() {
            let canvas = document.getElementById('canvas')
            canvas.width = document.body.clientWidth * window.devicePixelRatio
            canvas.height = document.body.clientHeight * window.devicePixelRatio
            this.redraw()
        },
        redraw() {
            let canvas = document.getElementById('canvas')
            let ctx = canvas.getContext("2d")

            ctx.setTransform(1, 0, 0, 1, 0, 0)
            ctx.clearRect(0, 0, canvas.width, canvas.height)
            ctx.save()
            ctx.lineWidth = 1
            ctx.fillStyle = "white";
            ctx.strokeStyle = "white";

            ctx.translate(canvas.width / 2, canvas.height / 2)

            let width = this.stream.image_size[0]
            let height = this.stream.image_size[1]

            let s = Math.min(canvas.width / width, canvas.height / height)
            ctx.scale(s, s)
            ctx.translate(0.5 - width / 2, 0.5 - height / 2)

            ctx.lineWidth = 1
            ctx.strokeStyle = "red"
            ctx.save()

            ctx.beginPath();
            ctx.rect(-0.5, -0.5, width, height);
            ctx.clip();

            this.drawStars(ctx)

            ctx.restore()

            ctx.beginPath();
            ctx.rect(-1, -1, width + 1, height + 1);
            ctx.stroke()

            ctx.restore()
        },
        drawStars(ctx) {
            if (this.stream.obs_pix === undefined) return
            let obs_pix = this.stream.obs_pix;
            let cat_pix = this.stream.cat_pix;
            ctx.save()
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
    },
    mounted() {
        this.connectImageWebSocket();
        this.connectStreamWebSocket();

        api('/api/get_state', null, this.updateState);

        window.onresize = this.resize
        this.resize()
    }
}
