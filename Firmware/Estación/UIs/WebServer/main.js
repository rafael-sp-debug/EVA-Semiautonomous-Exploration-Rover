document.addEventListener('DOMContentLoaded', () => {
  // ---------------------------------------------------------------------------
  // GRID SETUP
  const rows = 10;
  const cols = 10;

  const grid = document.getElementById("grid");

  // Generar celdas dinámicas
  for (let i = 0; i < rows * cols; i++) {
      const cell = document.createElement("div");
      cell.className = "cell";
      cell.id = "cell-" + i;
      grid.appendChild(cell);
  }

  // ---------------------------------------------------------------------------
  // // BASE64 IMAGE
  const base64String = "";
  document.getElementById('camImage').src = "data:image/jpeg;base64," + base64String;
  
  const ws = new WebSocket('ws://192.168.4.1:81');

  let myClientId = null;
  let currentController = 255;

  const attemptC = document.getElementById('attemptC');
  const releaseC = document.getElementById('releaseC');
  const buttons = document.querySelectorAll('button');
  const cancelImage = document.getElementById('cancelImage');

  // ---------------------------------------------------------------------------
  // TEL BUFFERS FOR CHARTS

  const telemetry = {
    rssiCurr: [], rssiAvg: [],
    tempInt: [], tempExt: [],
    humInt: [], humExt: [],
    voltEsp: [], currEsp: [], powEsp: [],
    voltM1: [], currM1: [], powM1: [],
    voltM2: [], currM2: [], powM2: [],
    accX: [], accY: [], accZ: [],
    angX: [], angY: [], angZ: [],
    dist1: [], dist2: [], dist3: []
  };

  function appendTelemetry(arr, val) {
    arr.push(val);
    if (arr.length > 20) arr.shift();
  }

  // ---------------------------------------------------------------------------
  // CHART MANAGEMENT

  const activeCharts = {
    chartBoxRSSI: null,
    chartBoxTEMP: null,
    chartBoxPOWER: null,
    chartBoxGYRO: null,
    chartBoxDIST: null
  };

  function createChart(canvasId, label, dataArray) {
    const ctx = document.getElementById(canvasId);

    return new Chart(ctx, {
      type: "line",
      data: {
        labels: Array(dataArray.length).fill(""),
        datasets: [{
          label: label,
          data: dataArray,
          borderWidth: 1
        }]
      },
      options: {
        responsive: true,
        animation: false,
        scales: { x: { display: false } }
      }
    });
  }

  function refreshVisibleCharts() {
    Object.values(activeCharts).forEach(chart => {
      if (chart) {
        chart.data.labels = Array(chart.data.datasets[0].data.length).fill("");
        chart.update();
      }
    });
  }

  // ---------------------------------------------------------------------------
  // CHART DROPDOWN

  document.querySelectorAll(".chart-select").forEach(select => {
    select.addEventListener("change", () => {
      const telemetryKey = select.value;
      const canvas = select.nextElementSibling;
      const canvasId = canvas.id;

      if (activeCharts[canvasId])
        activeCharts[canvasId].destroy();

      activeCharts[canvasId] =
        createChart(canvasId, telemetryKey, telemetry[telemetryKey]);
    });
  });

  // ---------------------------------------------------------------------------
  // AUTO START CHARTS

  window.addEventListener("load", () => {
    document.querySelectorAll(".chart-select").forEach(select => {
      const telemetryKey = select.value;
      const canvasId = select.nextElementSibling.id;

      activeCharts[canvasId] =
        createChart(canvasId, telemetryKey, telemetry[telemetryKey]);
    });
  });

  // ---------------------------------------------------------------------------
  // WEBSOCKET

  ws.onopen = () => {
    console.log("WebSocket connected");
    ws.send("HELLO_" + myClientId);
  };

  // ---------------------------------------------------------------------------
  // UPDATE GRID AND OBJECT COUNT
  function updateObjectCount() {
    const cells = document.querySelectorAll(".cell");
    let count = 0;

    cells.forEach(c => {
      if (c.style.backgroundColor === "red") {
        count++;
      }
    });

    document.getElementById("currObj").textContent = count;
  }

  function updateGridFromMessage(msg) {
    // msg = "M,0000000000,0000000000,..."
    const parts = msg.split(",");
    parts.shift(); // remove "M"

    if (parts.length !== 10) return; // safety

    // parts[0] is bottom row, parts[9] is top row
    // But our grid is cell-0 at TOP LEFT
    // So we reverse the rows to map correctly.
    const rowsFromBottom = parts.reverse();

    for (let row = 0; row < 10; row++) {
      const rowStr = rowsFromBottom[row]; // e.g. "0012000345"

      for (let col = 0; col < 10; col++) {
        const val = rowStr[col];
        const index = row * 10 + col;
        const cell = document.getElementById("cell-" + index);

        // Reset cell
        cell.style.backgroundColor = "gray";
        cell.textContent = "";

        if (val === "0") {
          // empty (do nothing)
        }
        else if (val === "1") {
          cell.textContent = "X";
          cell.style.backgroundColor = "red";
        }
        else if (val === "2") {
          cell.textContent = "↑";
          cell.style.backgroundColor = "blue";
        }
        else if (val === "3") {
          cell.textContent = "→";
          cell.style.backgroundColor = "blue";
        }
        else if (val === "4") {
          cell.textContent = "←";
          cell.style.backgroundColor = "blue";
        }
        else if (val === "5") {
          cell.textContent = "↓";
          cell.style.backgroundColor = "blue";
        }
        else if (val === "6") {
          cell.textContent = "O";
          cell.style.backgroundColor = "yellow";
        }
      }
    }

    updateObjectCount();
  }


  ws.onmessage = (event) => {
    const msg = event.data;
    console.log("Message received:", msg);

    // ---- ID assignment ----
    if (msg.startsWith("ASSIGN_ID_")) {
      myClientId = parseInt(msg.split("_")[2]);
      return;
    }

    // ---- Control state ----
    if (msg.startsWith("CTRL_")) {
      currentController = parseInt(msg.split("_")[1]);
      updateControlState(currentController, myClientId);
      return;
    }

    // ---- Image lockout ----
    if (msg === "IMG_START") {
      buttons.forEach(btn => {
        if (btn !== attemptC && btn !== releaseC && btn !== cancelImage) {
          btn.classList.add("gray");
          btn.disabled = true;
        }
      });
      return;
    }

    if (msg === "IMG_DONE") {
      buttons.forEach(btn => {
        if (btn !== attemptC && btn !== releaseC && btn !== cancelImage) {
          btn.classList.remove("gray");
          btn.disabled = false;
        }
      });
      return;
    }

    // ---- Image frame ----
    if (msg.startsWith("IMG_")) {
      const b64 = msg.substring(4);
      document.getElementById('camImage').src =
        "data:image/jpeg;base64," + b64;
      return;
    }

    // ---- GRID UPDATE ----
    if (msg.startsWith("M,")) {
        updateGridFromMessage(msg);
        return;
    }

    // Chunk size update
    if (msg.startsWith("CK_")) {
        const newChunks = parseInt(msg.substring(3));
        if (!isNaN(newChunks)) {
            document.getElementById("currChunks").textContent = newChunks;
        }
        return;
    }

    // NEMA step update
    if (msg.startsWith("NEMA_")) {
        const newSteps = parseInt(msg.substring(5));
        if (!isNaN(newSteps)) {
            document.getElementById("currSteps").textContent = newSteps;
        }
        return;
    }

    // CSV interval update
    if (msg.startsWith("CSVINT_")) {
        const newInterval = parseFloat(msg.substring(7));
        if (!isNaN(newInterval)) {
            document.getElementById("currInterval").textContent = newInterval;
        }
        return;
    }

    if (msg.startsWith("GOAL_")) {
        const newGoal = msg.substring(5);
        document.getElementById("currGoal").textContent = newGoal;
        return;
    }

    // -----------------------------------------------------------------------
    // TEL PARSING

    const dt = msg.split(',');

    if (dt.length === 24) {

      // Update text
      document.getElementById('rssiCurr').textContent = dt[0];
      document.getElementById('rssiAvg').textContent = dt[1];
      document.getElementById('tempInt').textContent = (dt[2] / 100).toFixed(2);
      document.getElementById('humInt').textContent = (dt[3] / 100).toFixed(2);
      document.getElementById('tempExt').textContent = (dt[4] / 100).toFixed(2);
      document.getElementById('humExt').textContent = (dt[5] / 100).toFixed(2);
      document.getElementById('voltEsp').textContent = (dt[6] / 100).toFixed(2);
      document.getElementById('currEsp').textContent = (dt[7] / 10).toFixed(1);
      document.getElementById('powEsp').textContent = dt[8];
      document.getElementById('voltM1').textContent = (dt[9] / 100).toFixed(2);
      document.getElementById('currM1').textContent = (dt[10] / 10).toFixed(1);
      document.getElementById('powM1').textContent = dt[11];
      document.getElementById('voltM2').textContent = (dt[12] / 100).toFixed(2);
      document.getElementById('currM2').textContent = (dt[13] / 10).toFixed(1);
      document.getElementById('powM2').textContent = dt[14];
      document.getElementById('accX').textContent = (dt[15] / 100).toFixed(2);
      document.getElementById('accY').textContent = (dt[16] / 100).toFixed(2);
      document.getElementById('accZ').textContent = (dt[17] / 100).toFixed(2);
      document.getElementById('angX').textContent = (dt[18] / 100).toFixed(2);
      document.getElementById('angY').textContent = (dt[19] / 100).toFixed(2);
      document.getElementById('angZ').textContent = (dt[20] / 100).toFixed(2);
      document.getElementById('dist1').textContent = dt[21];
      document.getElementById('dist2').textContent = dt[22];
      document.getElementById('dist3').textContent = dt[23];

      // Append to graph buffers
      appendTelemetry(telemetry.rssiCurr, parseInt(dt[0]));
      appendTelemetry(telemetry.rssiAvg, parseInt(dt[1]));
      appendTelemetry(telemetry.tempInt, dt[2] / 100);
      appendTelemetry(telemetry.humInt, dt[3] / 100);
      appendTelemetry(telemetry.tempExt, dt[4] / 100);
      appendTelemetry(telemetry.humExt, dt[5] / 100);
      appendTelemetry(telemetry.voltEsp, dt[6] / 100);
      appendTelemetry(telemetry.currEsp, dt[7] / 10);
      appendTelemetry(telemetry.powEsp, dt[8]);
      appendTelemetry(telemetry.voltM1, dt[9] / 100);
      appendTelemetry(telemetry.currM1, dt[10] / 10);
      appendTelemetry(telemetry.powM1, dt[11]);
      appendTelemetry(telemetry.voltM2, dt[12] / 100);
      appendTelemetry(telemetry.currM2, dt[13] / 10);
      appendTelemetry(telemetry.powM2, dt[14]);
      appendTelemetry(telemetry.accX, dt[15] / 100);
      appendTelemetry(telemetry.accY, dt[16] / 100);
      appendTelemetry(telemetry.accZ, dt[17] / 100);
      appendTelemetry(telemetry.angX, dt[18] / 100);
      appendTelemetry(telemetry.angY, dt[19] / 100);
      appendTelemetry(telemetry.angZ, dt[20] / 100);
      appendTelemetry(telemetry.dist1, parseInt(dt[21]));
      appendTelemetry(telemetry.dist2, parseInt(dt[22]));
      appendTelemetry(telemetry.dist3, parseInt(dt[23]));

      // Update current charts
      refreshVisibleCharts();
    }
  };

  // ---------------------------------------------------------------------------
  // SENDING COMMAND FUNCTION

  function sendCommand(cmd) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(cmd);
    }
  }

  // ---------------------------------------------------------------------------
  // CONTROL LOGIC

  attemptC.onclick = () => sendCommand("REQUEST_CONTROL");
  releaseC.onclick = () => sendCommand("RELEASE_CONTROL");

  function updateControlState(currentController, myClientId) {
    attemptC.classList.remove("gray");
    releaseC.classList.remove("gray");
    attemptC.disabled = false;
    releaseC.disabled = false;

    if (currentController === myClientId) {
      attemptC.classList.add("gray");
      attemptC.disabled = true;
    } 
    else if (currentController === 255) {
      releaseC.classList.add("gray");
      releaseC.disabled = true;
    } 
    else {
      attemptC.classList.add("gray");
      releaseC.classList.add("gray");
      attemptC.disabled = true;
      releaseC.disabled = true;
    }
  }

  // ---------------------------------------------------------------------------
  // BUTTON BINDS

  document.getElementById('recvTel').onclick = () => sendCommand('START_TEL');
  document.getElementById('stopTel').onclick = () => sendCommand('STOP_TEL');

  document.getElementById('btnUp').onclick = () => sendCommand('UP');
  document.getElementById('btnDown').onclick = () => sendCommand('DOWN');
  document.getElementById('btnLeft').onclick = () => sendCommand('LEFT');
  document.getElementById('btnRight').onclick = () => sendCommand('RIGHT');

  document.getElementById('requestImage').onclick = () => sendCommand('WEB_IMG');
  document.getElementById('captureImage').onclick = () => sendCommand('CAPTURE_IMG');
  document.getElementById('cancelImage').onclick = () => sendCommand('CANCEL_IMG');
    document.getElementById('resetCam').onclick = () => sendCommand('RESET_CAM');


  document.getElementById('startAuto').onclick = () => sendCommand('START_AUTO');
  document.getElementById('resumeAuto').onclick = () => sendCommand('RESUME_AUTO');
  document.getElementById('reverseAuto').onclick = () => sendCommand('REVERSE_AUTO');
  document.getElementById('stopAuto').onclick = () => sendCommand('STOP_AUTO');
  document.getElementById('pauseAuto').onclick = () => sendCommand('PAUSE_AUTO');
  document.getElementById('resetRover').onclick = () => sendCommand('RESET_ROVER');
  document.getElementById('clearMap').onclick = () => sendCommand('CLEAR_MAP');
  document.getElementById('clearAll').onclick = () => sendCommand('CLEAR_ALL');
  document.getElementById('party').onclick = () => sendCommand('PARTY_MODE');
  document.getElementById('forceShort').onclick = () => sendCommand('LORA_SHORT');
  document.getElementById('forceMid').onclick = () => sendCommand('LORA_MEDIUM');
  document.getElementById('forceLong').onclick = () => sendCommand('LORA_LONG');

  document.getElementById('setGoalBtn').onclick = () => {
    const coords = goalCoords.value.trim();
    sendCommand(`GOAL_${coords}`);
    goalCoords.value = "";
  };

  document.getElementById('setObjBtn').onclick = () => {
    const coords = objectCoords.value.trim();
    sendCommand(`OBJECT_${coords}`);
    objectCoords.value = "";
  };

  document.getElementById('setIntervalBtn').onclick = () => {
    sendCommand(`CSV_${intervalInput.value}`);
    intervalInput.value = "";
  };

  document.getElementById('setChunkBtn').onclick = () => {
    sendCommand(`CHUNK_${chunkSize.value}`);
    chunkSize.value = "";
  };

  document.getElementById('setStepBtn').onclick = () => {
    sendCommand(`STEP_${stepSize.value}`);
    stepSize.value = "";
  };
});
