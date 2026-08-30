#pragma once
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Chicken Coop Gauges</title>
  <script src="https://bernii.github.io/gauge.js/dist/gauge.min.js"></script>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 24px;
      background: #f4f6f8;
      color: #222;
      text-align: center;
    }

    h1 {
      margin-bottom: 24px;
    }

    .gauges {
      display: flex;
      flex-wrap: wrap;
      gap: 24px;
      justify-content: center;
    }

    .card {
      background: white;
      padding: 16px;
      border-radius: 12px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.08);
      width: 280px;
    }

    canvas {
      width: 220px;
      height: 140px;
    }

    .value {
      margin-top: 12px;
      font-size: 20px;
      font-weight: bold;
    }

    .status {
      margin-top: 20px;
      color: #555;
    }
  </style>
</head>
<body>
  <h1>Kurnik monitorr</h1>

  <div class="gauges">
    <div class="card">
      <h3>Temperatura</h3>
      <canvas id="tempGauge"></canvas>
      <div class="value" id="tempValue">--</div>
    </div>

    <div class="card">
      <h3>Wilgotność</h3>
      <canvas id="humGauge"></canvas>
      <div class="value" id="humValue">--</div>
    </div>

    <div class="card">
      <h3>Ciśnienie</h3>
      <canvas id="pressGauge"></canvas>
      <div class="value" id="pressValue">--</div>
    </div>

    <div class="card">
      <h3>Wysokość</h3>
      <canvas id="altGauge"></canvas>
      <div class="value" id="altValue">--</div>
    </div>
  </div>

  <div class="status" id="status">Loading...</div>

  <script>
    function createGauge(targetId, min, max) {
      const target = document.getElementById(targetId);
      const gauge = new Gauge(target).setOptions({
        angle: 0.15,
        lineWidth: 0.2,
        radiusScale: 1,
        pointer: {
          length: 0.6,
          strokeWidth: 0.035,
          color: "#000"
        },
        staticLabels: {
          font: "10px sans-serif",
          color: "#333",
          labels: [min, (min + max) / 2, max],
          fractionDigits: 0
        },
        staticZones: [
          { strokeStyle: "#30B32D", min: min, max: min + (max - min) * 0.5 },
          { strokeStyle: "#FFDD00", min: min + (max - min) * 0.5, max: min + (max - min) * 0.8 },
          { strokeStyle: "#F03E3E", min: min + (max - min) * 0.8, max: max }
        ],
        limitMin: false,
        limitMax: false,
        highDpiSupport: true
      });

      gauge.maxValue = max;
      gauge.setMinValue(min);
      gauge.animationSpeed = 32;
      gauge.set(min);
      return gauge;
    }

    const tempGauge = createGauge("tempGauge", -20, 50);
    const humGauge = createGauge("humGauge", 0, 100);
    const pressGauge = createGauge("pressGauge", 950, 1050);
    const altGauge = createGauge("altGauge", 0, 500);

    async function loadData() {
      try {
        const response = await fetch("/api/v1/");
        const data = await response.json();

        const temperature = parseFloat(data.temperature);
        const humidity = parseFloat(data.humidity);
        const pressure = parseFloat(data.pressure) / 100.0;
        const altitude = parseFloat(data.altitude);

        tempGauge.set(temperature);
        humGauge.set(humidity);
        pressGauge.set(pressure);
        altGauge.set(altitude);

        document.getElementById("tempValue").textContent = temperature.toFixed(2) + " C";
        document.getElementById("humValue").textContent = humidity.toFixed(2) + " %";
        document.getElementById("pressValue").textContent = pressure.toFixed(2) + " hPa";
        document.getElementById("altValue").textContent = altitude.toFixed(2) + " m";
        document.getElementById("status").textContent = "Aktualizacja Live";
      } catch (err) {
        document.getElementById("status").textContent = "Problem z ladowaniem danych...";
        console.error(err);
      }
    }

    loadData();
    setInterval(loadData, 3000);
  </script>
</body>
</html>
)rawliteral";
