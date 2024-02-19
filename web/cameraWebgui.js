let url = 'ws://' + window.location.host + "/image"
var ws = new WebSocket(url);

var img = document.getElementById('image');

ws.onmessage = function (event) {
    var blob = new Blob([event.data], { type: 'image/png' });
    img.src = URL.createObjectURL(blob);
};

function updateState(state) {
    console.log(state)
}

document.getElementById('capture').onclick = () => {
    let payload = {
        exposure_ms: document.getElementById('exposure').value,
        gain: document.getElementById('gain').value,
        digital_gain: document.getElementById('digital_gain').value,
        binning: document.getElementById('binning').value,
    }
    fetch('/capture', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    }).then(response => response.json()).then((data) => {
        updateState(data)
    }).catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById('put_calibration_image').onclick = () => {
    fetch('/put_calibration_image', {
        method: 'POST',
    }).then(response => response.json()).then((data) => {
        updateState(data)
    }).catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById('reset_calibration').onclick = () => {
    fetch('/reset_calibration', {
        method: 'POST',
    }).then(response => response.json()).then((data) => {
        updateState(data)
    }).catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById('calibrate').onclick = () => {

    let interval = setInterval(function () {
        let el = document.getElementById('calibrate')
        el.innerHTML += "."
    }, 1000)

    fetch('/calibrate', {
        method: 'POST',
    }).then(response => response.json()).then((data) => {
        document.getElementById('calibrate').innerHTML = "Calibrate"
        clearInterval(interval)
        updateState(data)
    }).catch(error => {
        document.getElementById('calibrate').innerHTML = "Calibrate"
        clearInterval(interval)
        console.error('Error:', error);
    });
}
