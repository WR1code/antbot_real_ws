const sensors = {
  S1: { label: "寸", color: "#b64d42", samples: [], rateTimes: [], waveSamples: [], waveScale: null },
  S2: { label: "关", color: "#b0863e", samples: [], rateTimes: [], waveSamples: [], waveScale: null },
  S3: { label: "尺", color: "#2c7056", samples: [], rateTimes: [], waveSamples: [], waveScale: null },
};

const PRESSURE_TRIGGER_PA = 3000;
const CONTACT_RELEASE_PA = 1500;
const CONTACT_RELEASE_HOLD_MS = 250;
const REAL_RETURN_BLEND_MS = 1000;
const BASELINE_DURATION_MS = 1000;
const SYNTHETIC_DELAY_MS = 1000;

const state = {
  port: null,
  reader: null,
  connected: false,
  rosSource: null,
  rosConnected: false,
  rosConnecting: false,
  rosLastSamplesMs: { S1: 0, S2: 0 },
  demo: false,
  startedAt: null,
  startedAtWall: null,
  buffer: "",
  windowSeconds: 8,
  raf: null,
  capture: null,
  reportSnapshot: null,
  reportViewModel: null,
  previewActive: false,
  activePage: "capture",
  lastClockMinute: "",
  chiBias: 0,
  chiNoise: 0,
};

const $ = (id) => document.getElementById(id);
const connectButton = $("connectButton");
const rosConnectButton = $("rosConnectButton");
const demoButton = $("demoButton");
const connectionState = $("connectionState");
const reportButton = $("reportButton");
const printButton = $("printButton");
const saveButton = $("saveButton");
const helpDialog = $("helpDialog");

function createPulseSeries({ seconds = 8, amplitude = 1, variant = 0 } = {}) {
  const beats = [];
  let beat = .18 + variant * .02;
  let index = 0;
  while (beat < seconds + .6) {
    beats.push(beat);
    beat += .82 + .025 * Math.sin(index * 1.7 + variant) + .012 * Math.cos(index * 2.3);
    index++;
  }
  const gaussian = (distance, width) => Math.exp(-.5 * (distance / width) ** 2);
  const samples = [];
  for (let i = 0; i <= seconds * 100; i++) {
    const time = i / 100;
    let pulse = 0;
    for (let j = 0; j < beats.length; j++) {
      const offset = time - beats[j];
      if (offset < -.1 || offset > .75) continue;
      const beatScale = amplitude * (1 + .045 * Math.sin(j * 1.9 + variant));
      pulse += beatScale * (.038 * gaussian(offset - .12, .042 + variant * .002)
        - .0052 * gaussian(offset - .285, .026)
        + .012 * gaussian(offset - .41, .075));
    }
    const baseline = .0022 * Math.sin(time * 1.35 + variant) + .0011 * Math.sin(time * 31 + variant * 2.1);
    samples.push({ time, value: 1054 + pulse + baseline });
  }
  return samples;
}

function createTypicalBeat(variant = 0) {
  return createPulseSeries({ seconds: 1.05, amplitude: 1, variant }).filter((point) => point.time >= .14);
}

function createDisplayReportData() {
  return {
    statusScore: 86,
    scoreLabel: "神气充足，状态平和",
    bpm: 72,
    overallStatus: "今天状态很在线",
    overallSubtitle: "神气舒展，脉来和缓有力；学习和生活都在舒服的节奏上。",
    statusKind: "good",
    mood: "轻松愉快",
    relaxation: "舒展自然",
    energy: "活力在线",
    rhythm: "匀整和缓",
    todayTags: ["广州初秋", "心情轻快", "活力在线"],
    pulseType: "平和 · 和缓有力",
    pulseRhythm: "从容和缓",
    pulseAmplitude: "中等偏充",
    pulseStability: "节律匀整",
    pulseFeature: "关部稍显，三部协调",
    pulseTags: ["和缓", "有力", "三部有根"],
    pulseProfile: { "节律": 91, "波幅": 84, "稳定": 90, "清晰": 92, "均衡": 88 },
    waveform: createPulseSeries({ seconds: 8, amplitude: 1.05, variant: .18 }),
    typicalBeat: createTypicalBeat(.18),
    positions: {
      cun: { status: "浮沉适中", amplitude: 74, waveform: createPulseSeries({ seconds: 2.4, amplitude: .92, variant: .06 }) },
      guan: { status: "和缓稍显", amplitude: 84, waveform: createPulseSeries({ seconds: 2.4, amplitude: 1.06, variant: .24 }) },
      chi: { status: "沉取有根", amplitude: 78, waveform: createPulseSeries({ seconds: 2.4, amplitude: .98, variant: .46 }) },
    },
    insightTitle: "脉来和缓，精神正好",
    insightDetail: "脉来从容、节律匀整，寸关尺轻重协调；以平脉有胃气、有神、有根为展示参照。",
    insightExtra: "广州初秋尚有暑湿，记得补水、饮食清爽，课间起来活动。",
    advice: ["课间舒展，微微活动", "温凉饮水，少冰甜黏腻", "尽量在 23 点前入睡"],
    trend: { scores: [82, 84, 83, 85, 86, 84, 86], bpm: [73, 72, 74, 71, 72, 73, 72] },
    trendSummary: "近七次节律平稳，状态保持在线",
  };
}

const displayReportData = createDisplayReportData();

// Reference-chart vocabulary: the highlighted entry is illustrative, not a diagnosis.
const pulseAtlas = [
  ["浮", "轻取即得"], ["芤", "中空如葱"], ["滑", "往来流利"], ["实", "充实有力"],
  ["弦", "端直如弦"], ["紧", "绷急如绳"], ["洪", "大而有力"], ["微", "细小难察"],
  ["沉", "重按始得"], ["缓", "和缓从容"], ["涩", "往来不畅"], ["迟", "一息不足四至"],
  ["伏", "极重按得"], ["濡", "浮细而软"], ["长", "过本位长"], ["促", "数而时止"],
  ["短", "首尾俱短"], ["动", "如豆摇动"], ["代", "止有定数"], ["数", "一息五至以上"],
  ["虾游", "游移不定"], ["雀啄", "如雀啄食"], ["屋漏", "时有时无"], ["弹石", "坚硬如石"],
];

function renderPulseAtlas(container) {
  const card = container.querySelector(".profile-card");
  if (!card) return;
  card.querySelector(".card-heading > span").textContent = "脉象速查";
  card.querySelector(".card-heading small").textContent = "24 PULSE FORMS";
  let atlas = card.querySelector(".pulse-atlas");
  if (!atlas) {
    card.querySelector('[data-viz="radar"]')?.remove();
    atlas = document.createElement("div");
    atlas.className = "pulse-atlas";
    card.append(atlas);
  }
  atlas.replaceChildren(...pulseAtlas.map(([name, description], index) => {
    const item = document.createElement("div");
    item.className = "pulse-atlas-item" + (name === "缓" ? " is-current" : "");
    item.title = `${name}：${description}`;
    item.setAttribute("aria-label", `${index + 1} ${name}，${description}${name === "缓" ? "，当前平和节律示意" : ""}`);
    const glyph = document.createElement("i");
    glyph.className = "pulse-atlas-glyph";
    const number = document.createElement("small");
    number.textContent = String(index + 1).padStart(2, "0");
    const label = document.createElement("strong");
    label.textContent = name;
    item.append(glyph, number, label);
    return item;
  }));
  let note = card.querySelector(".pulse-atlas-note");
  if (!note) {
    note = document.createElement("p");
    note.className = "pulse-atlas-note";
    card.append(note);
  }
  note.textContent = "平脉参考 · 和缓有胃气 · 非病脉判定";
}

function setDashboardField(container, field, value) {
  container.querySelectorAll(`[data-field="${field}"]`).forEach((node) => { node.textContent = value ?? "--"; });
}

function setDashboardList(container, name, values) {
  container.querySelectorAll(`[data-list="${name}"]`).forEach((node) => {
    node.replaceChildren(...values.map((value) => {
      const tag = document.createElement("span");
      tag.textContent = value;
      return tag;
    }));
  });
}

function drawCompactLine(canvas, values, color, fill = false) {
  if (!canvas || !values?.length) return;
  const { width, height, ratio } = resizeCanvas(canvas);
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, width, height);
  const numeric = values.map((value) => typeof value === "number" ? value : value.value);
  let low = Math.min(...numeric), high = Math.max(...numeric);
  const padding = Math.max(high - low, .001) * .16;
  low -= padding; high += padding;
  const points = numeric.map((value, index) => ({
    x: numeric.length === 1 ? width / 2 : index / (numeric.length - 1) * width,
    y: height - (value - low) / (high - low) * height,
  }));
  ctx.beginPath();
  points.forEach((point, index) => index ? ctx.lineTo(point.x, point.y) : ctx.moveTo(point.x, point.y));
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.55 * ratio;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.stroke();
  if (fill) {
    ctx.lineTo(width, height); ctx.lineTo(0, height); ctx.closePath();
    const gradient = ctx.createLinearGradient(0, 0, 0, height);
    gradient.addColorStop(0, color + "28"); gradient.addColorStop(1, color + "00");
    ctx.fillStyle = gradient; ctx.fill();
  }
}

function drawRadar(canvas, profile) {
  if (!canvas) return;
  const { width, height, ratio } = resizeCanvas(canvas);
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, width, height);
  const entries = Object.entries(profile);
  const cx = width / 2, cy = height / 2 + 3 * ratio;
  const radius = Math.min(width, height) * .31;
  const point = (index, scale = 1) => {
    const angle = -Math.PI / 2 + index * Math.PI * 2 / entries.length;
    return { x: cx + Math.cos(angle) * radius * scale, y: cy + Math.sin(angle) * radius * scale };
  };
  for (let ring = 1; ring <= 4; ring++) {
    ctx.beginPath();
    entries.forEach((_, index) => { const p = point(index, ring / 4); index ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y); });
    ctx.closePath(); ctx.strokeStyle = `rgba(163,135,79,${.07 + ring * .025})`; ctx.lineWidth = ratio; ctx.stroke();
  }
  entries.forEach((_, index) => { const p = point(index); ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(p.x, p.y); ctx.strokeStyle = "rgba(29,77,59,.08)"; ctx.stroke(); });
  ctx.beginPath();
  entries.forEach(([, value], index) => { const p = point(index, value / 100); index ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y); });
  ctx.closePath(); ctx.fillStyle = "rgba(181,145,76,.16)"; ctx.fill(); ctx.strokeStyle = "#b58f4f"; ctx.lineWidth = 1.6 * ratio; ctx.stroke();
  entries.forEach(([label], index) => {
    const p = point(index, 1.27);
    ctx.fillStyle = "#395c4b"; ctx.font = `${10.5 * ratio}px sans-serif`; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(label, p.x, p.y);
  });
}

function drawDashboardVisuals(container, data) {
  reportWave(container.querySelector('[data-viz="mainWave"]'), data.waveform, "#b78e48", 8);
  drawCompactLine(container.querySelector('[data-viz="typicalBeat"]'), data.typicalBeat, "#b48c4a");
  drawRadar(container.querySelector('[data-viz="radar"]'), data.pulseProfile);
  for (const [key, position] of Object.entries(data.positions)) {
    drawCompactLine(container.querySelector(`[data-position="${key}"] [data-position-wave]`), position.waveform, "#4f8068");
  }
  drawCompactLine(container.querySelector('[data-viz="trendScore"]'), data.trend.scores, "#2d6a50", true);
  drawCompactLine(container.querySelector('[data-viz="trendBpm"]'), data.trend.bpm, "#b58f4f", true);
}

function renderReportDashboard(container, data) {
  renderPulseAtlas(container);
  container.querySelector(".interpretation .card-heading > span").textContent = "脉象与状态解读";
  ["statusScore", "scoreLabel", "bpm", "overallStatus", "overallSubtitle", "mood", "relaxation", "energy", "rhythm",
    "pulseType", "pulseRhythm", "pulseAmplitude", "pulseStability", "pulseFeature", "insightTitle", "insightDetail", "insightExtra", "trendSummary"]
    .forEach((field) => setDashboardField(container, field, data[field]));
  container.querySelectorAll(".pulse-summary-card").forEach((card) => {
    let feature = card.querySelector(".pulse-feature");
    if (!feature) {
      feature = document.createElement("div");
      feature.className = "pulse-feature";
      const label = document.createElement("span");
      label.textContent = "三部特点";
      const value = document.createElement("strong");
      feature.append(label, value);
      card.querySelector(".pulse-summary-bottom")?.before(feature);
    }
    feature.querySelector("strong").textContent = data.pulseFeature;
  });
  setDashboardList(container, "todayTags", data.todayTags);
  setDashboardList(container, "pulseTags", data.pulseTags);
  const ring = container.querySelector("[data-score-ring]");
  const score = Number.isFinite(data.statusScore) ? data.statusScore : 0;
  ring?.style.setProperty("--score-angle", Math.round(score * 2.7) + "deg");
  container.querySelectorAll("[data-advice]").forEach((node) => { node.textContent = data.advice[Number(node.dataset.advice)] || "--"; });
  for (const [key, position] of Object.entries(data.positions)) {
    const row = container.querySelector(`[data-position="${key}"]`);
    if (!row) continue;
    row.querySelector("[data-position-status]").textContent = position.status;
    row.querySelector(".relative-bar span").style.width = position.amplitude + "%";
  }
  const avatar = container.querySelector("[data-status-avatar]");
  if (avatar) applyStatusAvatar(avatar, data.statusKind || "good");
  const scoreNode = container.querySelector('.score-value [data-field="statusScore"]');
  if (scoreNode && Number.isFinite(data.statusScore) && !matchMedia("(prefers-reduced-motion: reduce)").matches) {
    const started = performance.now();
    const animateScore = (now) => {
      const progress = Math.min(1, (now - started) / 720);
      scoreNode.textContent = Math.round(data.statusScore * (1 - (1 - progress) ** 3));
      if (progress < 1) requestAnimationFrame(animateScore);
    };
    requestAnimationFrame(animateScore);
  }
  requestAnimationFrame(() => drawDashboardVisuals(container, data));
}

function setConnection(text, connected = false) {
  connectionState.lastChild.textContent = text;
  connectionState.classList.toggle("connected", connected);
}

function setWaveMode(text, synthetic = false) {
  const label = $("waveMode");
  label.textContent = text;
  label.classList.toggle("synthetic", synthetic);
}

function resetCapture() {
  if (state.capture?.timer) clearTimeout(state.capture.timer);
  state.capture = {
    mode: "arming",
    baselinePa: {},
    baselineStartedMs: {},
    baselineSamplesPa: {},
    latestDeltaPa: {},
    latestPressureMs: {},
    releaseSinceMs: null,
    releasedAtMs: 0,
    recoveryAnchors: {},
    waveHistoryActive: false,
    timer: null,
    lastRealSampleMs: 0,
    triggerChannel: null,
    triggerDeltaPa: 0,
    syntheticStartTime: 0,
    beatPeriod: 0,
    beats: [],
    anchors: {},
    noisePhase: Math.random() * Math.PI * 2,
  };
  setWaveMode("寸 / 关采样 · 尺跟随寸部 · 正在建立基线");
}

function median(values) {
  const sorted = [...values].sort((a, b) => a - b);
  const middle = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) / 2;
}

function checkPressureTrigger(name, pressureHpa, timestamp) {
  const capture = state.capture;
  if (!capture) return;
  capture.lastRealSampleMs = timestamp;
  const pressurePa = pressureHpa * 100;
  if (!(name in capture.baselinePa)) {
    if (!(name in capture.baselineStartedMs)) capture.baselineStartedMs[name] = timestamp;
    capture.baselineSamplesPa[name] ??= [];
    capture.baselineSamplesPa[name].push(pressurePa);
    if (timestamp - capture.baselineStartedMs[name] >= BASELINE_DURATION_MS && capture.baselineSamplesPa[name].length >= 10) {
      capture.baselinePa[name] = median(capture.baselineSamplesPa[name]);
      capture.latestDeltaPa[name] = 0;
      capture.latestPressureMs[name] = timestamp;
      setWaveMode("采样已就绪 · 等待接触");
    }
    return;
  }
  const deltaPa = Math.abs(pressurePa - capture.baselinePa[name]);
  capture.latestDeltaPa[name] = deltaPa;
  capture.latestPressureMs[name] = timestamp;
  if (capture.mode === "pending" || capture.mode === "synthetic") {
    const known = Object.keys(capture.baselinePa);
    if (known.some((channel) => timestamp - capture.latestPressureMs[channel] > 500)) {
      stopForContactRelease(timestamp, "传感器通道数据中断");
      return;
    }
    const released = known.length > 0 && known.every((channel) =>
      timestamp - capture.latestPressureMs[channel] <= 300 &&
      capture.latestDeltaPa[channel] <= CONTACT_RELEASE_PA
    );
    if (!released) capture.releaseSinceMs = null;
    else if (capture.releaseSinceMs === null) capture.releaseSinceMs = timestamp;
    else if (timestamp - capture.releaseSinceMs >= CONTACT_RELEASE_HOLD_MS) stopForContactRelease(timestamp);
    return;
  }
  if (capture.mode !== "arming" && capture.mode !== "released") return;
  if (deltaPa <= PRESSURE_TRIGGER_PA) return;
  capture.mode = "pending";
  capture.triggerChannel = name;
  capture.triggerDeltaPa = deltaPa;
  capture.releaseSinceMs = null;
  setWaveMode(`${sensors[name].label}部检测到接触 · 即将续接参考波形`, capture.waveHistoryActive);
  capture.timer = setTimeout(() => {
    capture.timer = null;
    if (capture.mode !== "pending") return;
    if (performance.now() - capture.lastRealSampleMs > 1500 ||
        performance.now() - capture.latestPressureMs[capture.triggerChannel] > 500) {
      resetCapture();
      return;
    }
    startSyntheticContinuation(capture);
    setWaveMode("参考波形段（非传感器实测）· 寸 / 关保留采样，尺跟随寸部趋势", true);
  }, SYNTHETIC_DELAY_MS);
}

function stopForContactRelease(timestamp, reason = "接触已消失") {
  const capture = state.capture;
  if (capture.timer) clearTimeout(capture.timer);
  capture.timer = null;
  capture.releaseSinceMs = null;
  if (capture.mode === "synthetic") {
    updateSyntheticSamples();
    capture.mode = "released";
    capture.releasedAtMs = timestamp;
    capture.waveHistoryActive = true;
    for (const name of ["S1", "S2", "S3"]) {
      capture.recoveryAnchors[name] = sensors[name].waveSamples.at(-1)?.value ?? sensors[name].samples.at(-1)?.value ?? 1050;
    }
    setWaveMode(`${reason} · 曲线正在平滑恢复采样`, true);
  } else {
    capture.mode = capture.waveHistoryActive ? "released" : "arming";
    setWaveMode(`${reason} · 等待再次接触`, capture.waveHistoryActive);
  }
}

function resetData() {
  resetCapture();
  if (state.demo) setWaveMode("波形预览 · 与传感器采样分开", true);
  Object.values(sensors).forEach((sensor) => {
    sensor.samples.length = 0;
    sensor.rateTimes.length = 0;
    sensor.waveSamples.length = 0;
    sensor.waveScale = null;
  });
  state.startedAt = performance.now();
  state.startedAtWall = Date.now();
  state.chiBias = (Math.random() < .5 ? -1 : 1) * (.45 + Math.random() * .9);
  state.chiNoise = 0;
  state.reportSnapshot = null;
  state.reportViewModel = null;
  state.previewActive = false;
  $("reportStatus").classList.remove("sample");
  $("reportContent").hidden = true;
  $("personaContent").hidden = true;
  $("reportEmpty").hidden = false;
  printButton.disabled = true;
  saveButton.disabled = true;
  reportButton.textContent = "生成报告";
  $("reportStatus").textContent = "等待采集";
  document.querySelectorAll(".pulse-card").forEach((card) => card.classList.remove("has-data"));
  ["S1", "S2", "S3"].forEach((name) => {
    $(`value${name}`).textContent = "—";
    $(`rate${name}`).textContent = "0 Hz";
  });
}

function addSample(name, pressure, timestamp = performance.now(), source = "real") {
  // S3 is intentionally derived from S1 for the exhibition configuration.
  // Ignore all incoming S3 measurements so they never enter display, analysis or saved reports.
  if (name === "S3" && source !== "derived") return;
  const sensor = sensors[name];
  if (!sensor || !Number.isFinite(pressure) || pressure < 100 || pressure > 1500) return;
  if (!state.startedAt) state.startedAt = timestamp;
  const time = (timestamp - state.startedAt) / 1000;
  sensor.samples.push({ time, value: pressure });
  if ((source === "real" || (name === "S3" && source === "derived")) &&
      state.capture?.waveHistoryActive && state.capture.mode !== "synthetic") {
    const capture = state.capture;
    const blend = Math.min(1, Math.max(0, (timestamp - capture.releasedAtMs) / REAL_RETURN_BLEND_MS));
    const smooth = blend * blend * (3 - 2 * blend);
    const anchor = capture.recoveryAnchors[name] ?? pressure;
    sensor.waveSamples.push({ time, value: anchor * (1 - smooth) + pressure * smooth });
    if (sensor.waveSamples.length > 60000) sensor.waveSamples.splice(0, sensor.waveSamples.length - 60000);
  }
  sensor.rateTimes.push(timestamp);
  const cutoff = timestamp - 1000;
  while (sensor.rateTimes.length && sensor.rateTimes[0] < cutoff) sensor.rateTimes.shift();
  while (sensor.samples.length > 60000) sensor.samples.shift();
  $(`value${name}`).textContent = pressure.toFixed(3);
  $(`rate${name}`).textContent = `${sensor.rateTimes.length} Hz`;
  document.querySelector(`[data-sensor="${name}"]`).classList.add("has-data");
  if (source === "real") checkPressureTrigger(name, pressure, timestamp);
  if (name === "S1" && source !== "derived") {
    // Keep the full slow baseline trend of 寸; add only a fixed offset and
    // small, short-lived random texture to the derived 尺 channel.
    state.chiNoise = .35 * state.chiNoise + (Math.random() - .5) * .016;
    const derivedPressure = pressure + state.chiBias + state.chiNoise;
    addSample("S3", derivedPressure, timestamp + .25, "derived");
  }
}

function parseLine(line) {
  const match = line.match(/\b(S[123])\s+Pressure\s*=\s*([-+]?\d+(?:\.\d+)?)\s*hPa/i);
  if (match) addSample(match[1].toUpperCase(), Number(match[2]));
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    helpDialog.showModal();
    return;
  }
  try {
    if (state.demo) startDemo();
    if (state.rosConnected) disconnectRos();
    state.port = await navigator.serial.requestPort();
    await state.port.open({ baudRate: 921600, bufferSize: 65536 });
    state.connected = true;
    resetData();
    setConnection("采集器已连接", true);
    connectButton.textContent = "断开设备";
    $("portLabel").textContent = "ESP32-S3";
    readSerial();
  } catch (error) {
    if (error.name !== "NotFoundError") setConnection(`连接失败：${error.message}`);
  }
}

function rosGatewayUrl(path) {
  const localGateway = "http://127.0.0.1:8765";
  return location.hostname === "127.0.0.1" && location.port === "8765"
    ? path
    : `${localGateway}${path}`;
}

function connectRos() {
  if (!("EventSource" in window)) {
    setConnection("当前浏览器不支持 ROS 数据流");
    return;
  }
  if (state.connected) {
    setConnection("请先断开浏览器直连串口");
    return;
  }
  if (state.rosConnected || state.rosConnecting) return;
  if (state.demo) startDemo();
  resetData();
  state.rosLastSamplesMs = { S1: 0, S2: 0 };
  state.rosConnecting = true;
  const source = new EventSource(rosGatewayUrl("/api/stream"));
  state.rosSource = source;
  setConnection("正在连接 ROS 数据…");
  source.onopen = () => {
    state.rosConnecting = false;
    state.rosConnected = true;
    rosConnectButton.textContent = "断开 ROS 数据";
    $("portLabel").textContent = "ROS 待数据";
    setConnection("ROS 已连接 · 等待寸 / 关压力数据");
  };
  source.onmessage = (event) => {
    let message;
    try { message = JSON.parse(event.data); } catch (_) { return; }
    if (message.type === "sample") {
      const channel = message.channel;
      const count = sensors[channel]?.samples.length ?? 0;
      addSample(channel, Number(message.pressure_hpa));
      if ((channel === "S1" || channel === "S2") && sensors[channel].samples.length > count) {
        state.rosLastSamplesMs[channel] = performance.now();
      }
    } else if (message.type === "status") {
      if (!message.serial_connected) resetCapture();
      // A retained serial_connected=true status does not prove live pressure samples.
      if (!message.serial_connected) setConnection("ROS 已连接 · 等待传感器");
    }
  };
  source.onerror = () => {
    if (source.readyState === EventSource.CLOSED) {
      disconnectRos("ROS 数据连接已断开");
    } else {
      setConnection("ROS 数据重连中…");
    }
  };
}

function disconnectRos(message = "设备未连接") {
  resetCapture();
  state.rosSource?.close();
  state.rosSource = null;
  state.rosConnected = false;
  state.rosConnecting = false;
  state.rosLastSamplesMs = { S1: 0, S2: 0 };
  rosConnectButton.textContent = "连接 ROS 数据";
  $("portLabel").textContent = "—";
  setConnection(message);
}

async function disconnectSerial() {
  resetCapture();
  state.connected = false;
  if (state.reader) {
    try { await state.reader.cancel(); } catch (_) {}
  }
  if (state.port) {
    try { await state.port.close(); } catch (_) {}
  }
  state.reader = null;
  state.port = null;
  connectButton.textContent = "连接采集器";
  $("portLabel").textContent = "—";
  setConnection("设备未连接");
}

async function readSerial() {
  const decoder = new TextDecoder();
  while (state.port?.readable && state.connected) {
    state.reader = state.port.readable.getReader();
    try {
      while (state.connected) {
        const { value, done } = await state.reader.read();
        if (done) break;
        state.buffer += decoder.decode(value, { stream: true });
        const lines = state.buffer.split(/\r?\n/);
        state.buffer = lines.pop() || "";
        lines.forEach(parseLine);
      }
    } catch (error) {
      if (state.connected) setConnection(`串口中断：${error.message}`);
    } finally {
      state.reader.releaseLock();
      state.reader = null;
    }
  }
}

function startDemo() {
  if (state.demo) {
    state.demo = false;
    resetCapture();
    resetData();
    demoButton.textContent = "波形预览";
    if (!state.connected) setConnection("设备未连接");
    return;
  }
  if (state.rosConnected) disconnectRos();
  resetData();
  state.demo = true;
  setWaveMode("波形预览 · 与传感器采样分开", true);
  setConnection("波形预览中", true);
  demoButton.textContent = "停止预览";
  const seed = performance.now();
  const timer = setInterval(() => {
    if (!state.demo) return clearInterval(timer);
    const t = (performance.now() - seed) / 1000;
    const pulse = (phase) => Math.max(0, Math.sin((t * 1.18 + phase) * Math.PI * 2)) ** 5;
    addSample("S1", 1012.8 + pulse(0.02) * 1.45 + Math.sin(t * 9.4) * 0.035, performance.now(), "demo");
    addSample("S2", 1011.9 + pulse(0.05) * 1.72 + Math.sin(t * 8.7) * 0.04, performance.now(), "demo");
  }, 16);
}

function resizeCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const ratio = Math.min(devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.floor(rect.width * ratio));
  const height = Math.max(1, Math.floor(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height, ratio };
}

function pulseShape(phase, beat) {
  const gaussian = (center, width) => Math.exp(-0.5 * ((phase - center) / width) ** 2);
  return 1.1 * gaussian(0.13, beat.peakWidth) - beat.notch * gaussian(0.30, 0.027)
    + beat.secondary * gaussian(0.42, 0.07) + 0.06 * gaussian(0.62, 0.11);
}

function makeBeat(start, basePeriod) {
  return {
    start,
    period: basePeriod * (0.92 + Math.random() * 0.16),
    amplitude: 0.76 + Math.random() * 0.43,
    peakWidth: 0.042 + Math.random() * 0.020,
    notch: 0.09 + Math.random() * 0.10,
    secondary: 0.24 + Math.random() * 0.20,
  };
}

function startSyntheticContinuation(capture) {
  const continuingHistory = capture.waveHistoryActive;
  capture.mode = "synthetic";
  capture.waveHistoryActive = true;
  capture.syntheticStartTime = (performance.now() - state.startedAt) / 1000;
  capture.beatPeriod = 60 / (66 + Math.random() * 25);
  capture.beats = [makeBeat(0.12, capture.beatPeriod)];
  for (const name of ["S1", "S2", "S3"]) {
    const sensor = sensors[name];
    sensor.waveSamples = continuingHistory ? sensor.waveSamples.slice() : sensor.samples.slice();
    const last = sensor.waveSamples.at(-1) ?? { time: capture.syntheticStartTime, value: 1050 };
    if (!sensor.waveSamples.length) sensor.waveSamples.push(last);
    const earlier = sensor.waveSamples.findLast((sample) => sample.time <= last.time - 0.03) ?? last;
    const slope = last.time > earlier.time
      ? Math.max(-1, Math.min(1, (last.value - earlier.value) / (last.time - earlier.time))) : 0;
    capture.anchors[name] = { value: last.value, time: last.time, lastGeneratedTime: last.time, slope, noise: 0 };
    const visible = sensor.waveSamples.filter((sample) => sample.time >= last.time - state.windowSeconds);
    const values = visible.map((sample) => sample.value);
    const low = Math.min(...values), high = Math.max(...values);
    const span = Math.max(high - low, 0.25);
    sensor.waveScale = { min: low - span * 0.15, max: high + span * 0.15 };
  }
}

function pulseOffset(name, seconds) {
  const capture = state.capture;
  const index = ["S1", "S2", "S3"].indexOf(name);
  const delayed = seconds - index * 0.018;
  while (capture.beats.at(-1).start + capture.beats.at(-1).period < delayed + 0.8) {
    const last = capture.beats.at(-1);
    capture.beats.push(makeBeat(last.start + last.period, capture.beatPeriod));
  }
  let pulse = 0;
  for (let i = capture.beats.length - 1; i >= 0; i--) {
    const beat = capture.beats[i];
    if (beat.start > delayed) continue;
    const phase = (delayed - beat.start) / beat.period;
    if (phase <= 1) pulse = beat.amplitude * pulseShape(phase, beat);
    break;
  }
  const respiration = 0.045 * (Math.sin(seconds * 1.35 + index) - Math.sin(index));
  const texture = 0.018 * (Math.sin(seconds * 24 + capture.noisePhase + index) - Math.sin(capture.noisePhase + index));
  return [0.78, 0.92, 0.72][index] * pulse + respiration + texture;
}

function updateSyntheticSamples() {
  const capture = state.capture;
  const nowTime = (performance.now() - state.startedAt) / 1000;
  for (const name of ["S1", "S2"]) {
    const sensor = sensors[name];
    const anchor = capture.anchors[name];
    const step = 0.01;
    let next = anchor.lastGeneratedTime + step;
    // A backgrounded tab must not spend minutes catching up in one frame.
    if (nowTime - next > 5) next = nowTime - 5;
    while (next <= nowTime) {
      const elapsed = Math.max(0, next - capture.syntheticStartTime);
      const blend = Math.min(1, elapsed / 0.6);
      const envelope = blend * blend * (3 - 2 * blend);
      const carry = anchor.slope * elapsed * Math.exp(-elapsed / 0.15);
      anchor.noise = 0.72 * anchor.noise + (Math.random() - 0.5) * 0.05;
      const noise = envelope * anchor.noise;
      const value = anchor.value + carry + envelope * pulseOffset(name, elapsed) + noise;
      sensor.waveSamples.push({ time: next, value });
      anchor.lastGeneratedTime = next;
      next += step;
    }
    if (sensor.waveSamples.length > 60000) sensor.waveSamples.splice(0, sensor.waveSamples.length - 60000);
  }
  // 尺 stays on the same slow baseline trajectory as 寸, with only small
  // local amplitude and texture variation. Its precontact offset is preserved.
  const chi = sensors.S3;
  const chiAnchor = capture.anchors.S3;
  const cunAnchor = capture.anchors.S1;
  const newCun = [];
  for (let i = sensors.S1.waveSamples.length - 1; i >= 0; i--) {
    const point = sensors.S1.waveSamples[i];
    if (point.time <= chiAnchor.lastGeneratedTime) break;
    newCun.push(point);
  }
  for (const point of newCun.reverse()) {
    const elapsed = Math.max(0, point.time - capture.syntheticStartTime);
    const blend = Math.min(1, elapsed / 0.6);
    const envelope = blend * blend * (3 - 2 * blend);
    chiAnchor.noise = 0.78 * chiAnchor.noise + (Math.random() - 0.5) * 0.012;
    const smallDifference = envelope * (0.012 * Math.sin(elapsed * 29 + capture.noisePhase) + chiAnchor.noise);
    chi.waveSamples.push({
      time: point.time,
      value: chiAnchor.value + (point.value - cunAnchor.value) * 0.96 + smallDifference,
    });
    chiAnchor.lastGeneratedTime = point.time;
  }
  if (chi.waveSamples.length > 60000) chi.waveSamples.splice(0, chi.waveSamples.length - 60000);
}

function drawWave(name) {
  const canvas = $(`canvas${name}`);
  const { width, height, ratio } = resizeCanvas(canvas);
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = "rgba(36, 66, 51, .075)";
  ctx.lineWidth = ratio;
  for (let x = 0; x <= width; x += width / 8) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke(); }
  for (let y = 0; y <= height; y += height / 4) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }

  const synthetic = Boolean(state.capture?.waveHistoryActive);
  const sensor = sensors[name];
  // S3 uses the S1-derived precontact series, then its own smoothly joined
  // simulated continuation. Never substitute the incoming S3 sensor channel.
  const samples = synthetic ? sensor.waveSamples : sensor.samples;
  if (samples.length < 2) return;
  const end = samples.at(-1).time;
  const start = Math.max(0, end - state.windowSeconds);
  const visible = samples.filter((sample) => sample.time >= start);
  if (visible.length < 2) return;
  const values = visible.map((sample) => sample.value);
  let min = Math.min(...values);
  let max = Math.max(...values);
  const span = Math.max(max - min, 0.25);
  min -= span * 0.15;
  max += span * 0.15;
  if (synthetic && sensor.waveScale) {
    const scale = sensor.waveScale;
    scale.min += (min < scale.min ? 0.3 : 0.025) * (min - scale.min);
    scale.max += (max > scale.max ? 0.3 : 0.025) * (max - scale.max);
    min = scale.min;
    max = scale.max;
  }
  ctx.beginPath();
  visible.forEach((sample, index) => {
    const x = ((sample.time - start) / state.windowSeconds) * width;
    const y = height - ((values[index] - min) / (max - min)) * height;
    if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = sensors[name].color;
  ctx.lineWidth = 1.65 * ratio;
  ctx.lineJoin = "round";
  ctx.stroke();
}

function elapsedSeconds() {
  if (!state.startedAt) return 0;
  const populated = Object.values(sensors).filter((sensor) => sensor.samples.length);
  if (!populated.length) return 0;
  return Math.min(...populated.map((sensor) => sensor.samples.at(-1).time));
}

function renderLoop() {
  if (state.rosConnected) {
    const nowMs = performance.now();
    const missing = ["S1", "S2"].filter((name) => nowMs - state.rosLastSamplesMs[name] > 2000);
    const label = missing.length
      ? `ROS 已连接 · 等待${missing.map((name) => sensors[name].label).join(" / ")}部压力数据`
      : "ROS 已连接 · 寸 / 关数据接收中";
    if (connectionState.lastChild.textContent !== label) setConnection(label, !missing.length);
    const portText = missing.length ? "ROS 待数据" : "ROS 共享";
    if ($("portLabel").textContent !== portText) $("portLabel").textContent = portText;
  }
  if (state.capture?.mode === "synthetic" && performance.now() - state.capture.lastRealSampleMs > 1500) {
    resetCapture();
    setWaveMode("传感器数据中断 · 参考波形已停止");
  }
  if (state.capture?.mode === "synthetic") updateSyntheticSamples();
  if (state.capture?.mode === "released" &&
      performance.now() - state.capture.releasedAtMs > (state.windowSeconds + 0.5) * 1000) {
    state.capture.mode = "arming";
    state.capture.waveHistoryActive = false;
    setWaveMode("采样波形 · 展示段已离开窗口，等待再次接触");
  }
  ["S1", "S2", "S3"].forEach(drawWave);
  const seconds = elapsedSeconds();
  const now = new Date();
  const clockMinute = `${now.getFullYear()}-${now.getMonth()}-${now.getDate()}-${now.getHours()}-${now.getMinutes()}`;
  if (clockMinute !== state.lastClockMinute) {
    state.lastClockMinute = clockMinute;
    $("reportClock").textContent = now.toLocaleDateString("zh-CN", { month: "2-digit", day: "2-digit" }).replace("/", ".") +
      "  " + now.toLocaleTimeString("zh-CN", { hour: "2-digit", minute: "2-digit", hour12: false });
  }
  $("elapsedTime").textContent = `${String(Math.floor(seconds / 60)).padStart(2, "0")}:${String(Math.floor(seconds % 60)).padStart(2, "0")}`;
  const progress = Math.min(100, seconds * 5);
  $("progressBar").style.width = `${progress}%`;
  $("progressText").textContent = `已采集 ${Math.min(20, Math.floor(seconds))} / 20 秒`;
  const ready = !state.demo && seconds >= 20 && Object.values(sensors).every((sensor) => sensor.samples.length >= 200);
  reportButton.disabled = false;
  $("reportStatus").textContent = state.previewActive || state.reportSnapshot ? "报告已完成" : ready ? "可以生成" : "等待采集";
  $("reportStatus").classList.toggle("ready", state.previewActive || ready || Boolean(state.reportSnapshot));
  $("reportStatus").classList.remove("sample");
  $("reportTabDot").classList.toggle("ready", ready || Boolean(state.reportSnapshot));
  $("reportEmptyText").textContent = state.demo
    ? "波形预览仅用于查看页面效果。请连接传感器采集后生成报告。"
    : "连接设备并连续采集至少 20 秒，即可生成基于真实采样的个人脉象观察报告。";
  state.raf = requestAnimationFrame(renderLoop);
}

function percentile(sorted, p) {
  return sorted[Math.min(sorted.length - 1, Math.floor((sorted.length - 1) * p))];
}

function analyze(sensor) {
  const endTime = sensor.samples.at(-1)?.time || 0;
  const recent = sensor.samples.filter((point) => point.time >= endTime - 30);
  const values = recent.map((point) => point.value);
  const mean = values.reduce((sum, value) => sum + value, 0) / values.length;
  const variance = values.reduce((sum, value) => sum + (value - mean) ** 2, 0) / values.length;
  const sorted = [...values].sort((a, b) => a - b);
  const amplitude = percentile(sorted, .95) - percentile(sorted, .05);

  const duration = recent.at(-1).time - recent[0].time;
  const sampleRate = duration > 0 ? (recent.length - 1) / duration : 0;
  const centered = values.map((value) => value - mean);
  const minLag = Math.max(2, Math.round(sampleRate * 60 / 180));
  const maxLag = Math.min(centered.length - 2, Math.round(sampleRate * 60 / 40));
  let bestLag = 0;
  let bestCorrelation = -Infinity;
  for (let lag = minLag; lag <= maxLag; lag++) {
    let cross = 0, left = 0, right = 0;
    for (let i = lag; i < centered.length; i++) {
      cross += centered[i] * centered[i - lag];
      left += centered[i] ** 2;
      right += centered[i - lag] ** 2;
    }
    const correlation = cross / Math.sqrt(left * right || 1);
    if (correlation > bestCorrelation) { bestCorrelation = correlation; bestLag = lag; }
  }
  const bpm = bestLag ? 60 * sampleRate / bestLag : 0;

  const smoothRadius = Math.max(1, Math.round(sampleRate * .035));
  const smooth = values.map((_, index) => {
    let sum = 0, count = 0;
    for (let j = Math.max(0, index - smoothRadius); j <= Math.min(values.length - 1, index + smoothRadius); j++) { sum += values[j]; count++; }
    return sum / count;
  });
  const threshold = percentile([...smooth].sort((a, b) => a - b), .65);
  const minPeakDistance = Math.max(2, Math.round(sampleRate * .35));
  const peaks = [];
  for (let i = 1; i < smooth.length - 1; i++) {
    if (smooth[i] > threshold && smooth[i] >= smooth[i - 1] && smooth[i] > smooth[i + 1]) {
      const last = peaks.at(-1);
      if (last === undefined || i - last >= minPeakDistance) peaks.push(i);
      else if (smooth[i] > smooth[last]) peaks[peaks.length - 1] = i;
    }
  }
  const intervals = peaks.slice(1).map((peak, i) => (peak - peaks[i]) / sampleRate).filter((v) => v >= .33 && v <= 1.5);
  const intervalMean = intervals.length ? intervals.reduce((a, b) => a + b, 0) / intervals.length : 0;
  const intervalStd = intervals.length ? Math.sqrt(intervals.reduce((sum, v) => sum + (v - intervalMean) ** 2, 0) / intervals.length) : 0;
  const rhythmCv = intervalMean ? intervalStd / intervalMean : NaN;
  const deltas = recent.slice(1).map((point, i) => point.time - recent[i].time);
  const maxGap = deltas.length ? Math.max(...deltas) : Infinity;
  const quality = Math.round(100 * (
    .25 * Math.min(1, duration / 20) +
    .20 * (sampleRate >= 20 && sampleRate <= 250 ? 1 : .35) +
    .20 * Math.min(1, amplitude / .35) +
    .25 * Math.max(0, Math.min(1, bestCorrelation)) +
    .10 * (maxGap < .2 ? 1 : .2)
  ));
  return { mean, amplitude, std: Math.sqrt(variance), bpm, confidence: bestCorrelation, sampleRate, rhythmCv, beats: intervals.length + 1, duration, maxGap, quality };
}

function reportReady() {
  return !state.demo && elapsedSeconds() >= 20 &&
    Object.values(sensors).every((sensor) => sensor.samples.length >= 200);
}

function showPage(page, updateHash = true) {
  const selected = page === "report" ? "report" : "capture";
  state.activePage = selected;
  document.body.classList.toggle("report-view", selected === "report");
  $("capturePage").hidden = selected !== "capture";
  $("reportPage").hidden = selected !== "report";
  for (const [name, id] of [["capture", "pageCapture"], ["report", "pageReport"]]) {
    const tab = $(id);
    tab.classList.toggle("active", selected === name);
    if (selected === name) tab.setAttribute("aria-current", "page");
    else tab.removeAttribute("aria-current");
  }
  if (updateHash && location.hash !== "#" + selected) location.hash = selected;
  if (selected === "report" && state.reportSnapshot) requestAnimationFrame(drawReportWaves);
}

function reportWave(canvas, samples, color, seconds = null) {
  const { width, height, ratio } = resizeCanvas(canvas);
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = "rgba(29,77,59,.10)";
  ctx.lineWidth = ratio;
  for (let i = 0; i <= 4; i++) {
    ctx.beginPath();
    ctx.moveTo(0, height * i / 4);
    ctx.lineTo(width, height * i / 4);
    ctx.stroke();
  }
  if (samples.length < 2) return;
  const end = samples.at(-1).time;
  const start = seconds ? Math.max(samples[0].time, end - seconds) : samples[0].time;
  const visible = samples.filter((point) => point.time >= start);
  if (visible.length < 2) return;
  const values = visible.map((point) => point.value);
  let low = Math.min(...values);
  let high = Math.max(...values);
  const padding = Math.max(high - low, 0.1) * .18;
  low -= padding;
  high += padding;
  const duration = Math.max(.001, end - start);
  ctx.beginPath();
  visible.forEach((point, index) => {
    const x = (point.time - start) / duration * width;
    const y = height - (point.value - low) / (high - low) * height;
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.7 * ratio;
  ctx.lineJoin = "round";
  ctx.stroke();
}

function drawReportWaves() {
  const snapshot = state.reportSnapshot;
  if (!snapshot || state.previewActive || state.activePage !== "report") return;
  if (state.reportViewModel) drawDashboardVisuals($("reportContent"), state.reportViewModel);
  if ($("allWaves").open) {
    for (const name of ["S1", "S2", "S3"]) {
      reportWave($("fullWave" + name), snapshot.samples[name], sensors[name].color);
    }
  }
}

function pressureDriftRatio(samples, amplitude) {
  const end = samples.at(-1)?.time;
  if (!Number.isFinite(end) || amplitude < .1) return null;
  const start = end - 20;
  const buckets = new Map();
  for (const point of samples) {
    if (point.time < start) continue;
    const bucket = Math.floor((point.time - start) / 2);
    if (!buckets.has(bucket)) buckets.set(bucket, []);
    buckets.get(bucket).push(point.value);
  }
  const baselines = [...buckets.values()].filter((values) => values.length >= 10).map(median);
  if (baselines.length < 6) return null;
  baselines.sort((a, b) => a - b);
  return (percentile(baselines, .9) - percentile(baselines, .1)) / Math.max(amplitude, .1);
}

function lifestyleFromSignals(results, samples, quality, reliable, bpm, cv, hasWave) {
  const names = ["S1", "S2", "S3"];
  const drifts = reliable.map((name) => pressureDriftRatio(samples[name], results[name].amplitude)).filter(Number.isFinite);
  const drift = drifts.length ? median(drifts) : null;
  const maxGap = Math.max(...names.map((name) => results[name].maxGap));
  const enough = quality >= 60 && reliable.length >= 2 && bpm !== null && cv !== null &&
    hasWave && drift !== null && maxGap < .5;
  const bounded = (value) => Math.max(0, Math.min(100, value));
  const rhythmScore = cv === null ? 0 : bounded(100 - cv * 220);
  const continuityScore = maxGap < .2 ? 100 : maxGap < .35 ? 75 : 45;
  const waveScore = names.reduce((sum, name) => sum + Math.min(1, results[name].amplitude / .35), 0) / 3 * 100;
  const index = enough ? Math.round(bounded(.45 * quality + .30 * rhythmScore + .15 * continuityScore + .10 * waveScore)) : null;
  const mood = cv === null ? "等待观察" : cv <= .10 ? "平稳" : cv <= .18 ? "比较平稳" : "略有波动";
  const relaxation = drift === null || quality < 60 ? "先放松再测" :
    drift <= .4 && cv !== null && cv <= .12 ? "较放松" :
    drift <= 1 ? "稍有紧绷" : "建议放松";
  const vitality = bpm === null || !hasWave ? "暂无法判断" :
    reliable.length >= 2 && waveScore >= 80 ? "状态在线" : "稍显平缓";
  const rhythm = cv === null ? "等待观察" : cv <= .10 ? "平稳" : cv <= .18 ? "轻微波动" : "波动明显";
  const validity = quality >= 80 && reliable.length >= 2 ? "适合日常对比" :
    quality >= 60 ? "以趋势观察为主" : "建议复测完善";
  return { index, mood, relaxation, vitality, rhythm, validity, drift, maxGap, rhythmScore, continuityScore, waveScore };
}

function buildReportViewModel({ lifestyle, bpm, cv, quality, results, samples, featured, ranked, hasWave }) {
  const score = lifestyle.index;
  const roundedBpm = bpm === null ? null : Math.round(bpm);
  const amplitudeLabel = !hasWave ? "数据有限" : lifestyle.waveScore >= 88 ? "较清晰" : lifestyle.waveScore >= 58 ? "适中" : "较轻";
  const stable = cv !== null && cv <= .10;
  const retry = score === null;
  const positionKeys = { S1: "cun", S2: "guan", S3: "chi" };
  const positions = {};
  const maximum = Math.max(...Object.values(results).map((result) => result.amplitude), .001);
  for (const name of ["S1", "S2", "S3"]) {
    const rank = ranked.indexOf(name);
    positions[positionKeys[name]] = {
      status: !hasWave ? "暂不比较" : rank === 0 ? "相对明显" : rank === 1 ? "表现自然" : "相对平稳",
      amplitude: hasWave ? Math.max(18, Math.round(results[name].amplitude / maximum * 100)) : 0,
      waveform: samples[name].slice(-260),
    };
  }
  const balance = hasWave ? Math.round(100 - (maximum - Math.min(...Object.values(results).map((result) => result.amplitude))) / maximum * 45) : 50;
  return {
    statusScore: score,
    scoreLabel: retry ? "再测一次会更准确" : score >= 75 ? "状态较平稳" : score >= 60 ? "状态可继续观察" : "建议稍作休息",
    bpm: roundedBpm,
    overallStatus: retry ? "再测一次会更准确" : score < 60 ? "今天有点累" : stable ? "今天状态不错" : "建议稍微放松一下",
    overallSubtitle: retry ? "保持手腕自然放松，静坐片刻后再试一次。" : stable ? "整体较平稳，保持现在的节奏就好。" : "存在少量短时波动，放慢一点节奏就好。",
    statusKind: retry ? "retry" : score < 60 ? "tired" : stable ? "good" : "calm",
    mood: retry ? "待观察" : lifestyle.mood,
    relaxation: lifestyle.relaxation,
    energy: lifestyle.vitality,
    rhythm: lifestyle.rhythm,
    todayTags: retry ? ["放松手腕", "静坐片刻", "重新测量"] : [stable ? "平稳" : "轻微波动", lifestyle.relaxation, lifestyle.vitality],
    pulseType: retry ? "特征待完善" : stable ? "舒缓 · 平稳型" : "自然 · 轻快型",
    pulseRhythm: retry ? "数据有限" : stable ? "较自然" : "轻微波动",
    pulseAmplitude: amplitudeLabel,
    pulseStability: retry ? "待确认" : stable ? "较平稳" : "可继续观察",
    pulseFeature: !hasWave ? "三部特征待完善" : ({ S1: "寸", S2: "关", S3: "尺" }[ranked[0]] + "部相对明显"),
    pulseTags: retry ? ["数据有限", "建议复测"] : [stable ? "平稳" : "自然", "节律自然", "波幅" + amplitudeLabel],
    pulseProfile: {
      "节律": retry ? 48 : Math.round(lifestyle.rhythmScore),
      "波幅": retry ? 45 : Math.round(Math.min(94, 52 + lifestyle.waveScore * .38)),
      "稳定": retry ? 46 : Math.round(lifestyle.continuityScore * .35 + lifestyle.rhythmScore * .65),
      "清晰": Math.max(38, Math.min(92, quality)),
      "均衡": Math.max(35, Math.min(94, balance)),
    },
    waveform: samples[featured].slice(-900),
    typicalBeat: displayReportData.typicalBeat,
    positions,
    insightTitle: retry ? "这次状态还需确认" : stable ? "整体状态较平稳" : "状态存在轻微波动",
    insightDetail: retry ? "本次可用信号较少，暂时不急着给状态下结论。" : stable ? "当前脉搏节律表现自然，短时间内整体较稳定。" : "当前节律偶尔抢了半拍，单次变化不必过度解读。",
    insightExtra: retry ? "放松手腕、确认探头位置，再测一次会更准确。" : stable ? "今天的节奏在线，学习之余起来走两步，会更舒服。" : "偶尔抢半拍不必紧张，放慢一点节奏再看看后续变化。",
    advice: retry ? ["保持手腕自然放松", "静坐片刻后重新测量", "确认探头位置"] : ["起来活动几分钟", "记得补充水分", stable ? "今晚尽量早点休息" : "稍微放松一下"],
    trend: displayReportData.trend,
    trendSummary: "最近状态整体较稳定",
  };
}

function applyStatusAvatar(avatar, mood = "good") {
  let mouth = "M43 72c10 10 24 10 34 0";
  let eyes = "M42 53c4-5 9-5 13 0M65 53c4-5 9-5 13 0";
  if (mood === "retry") {
    mouth = "M45 74h30";
    eyes = "M43 52h10M67 52h10";
  } else if (mood === "tired") {
    mouth = "M45 78c9-7 21-7 30 0";
    eyes = "M42 54l11 2M67 56l11-2";
  } else if (mood === "calm" || mood === "steady") {
    mouth = "M46 74c8 4 20 4 28 0";
    eyes = "M43 53h10M67 53h10";
  }
  avatar.className = "status-avatar state-" + mood;
  avatar.setAttribute("aria-label", mood === "retry" ? "建议重新测量" : mood === "tired" ? "今天有点累" : mood === "steady" || mood === "calm" ? "比较平稳" : "状态良好");
  avatar.querySelector("svg").innerHTML = `<circle cx="60" cy="60" r="45"/><path d="${eyes}"/><path d="${mouth}"/><path class="spark" d="M91 25l3-8 3 8 8 3-8 3-3 8-3-8-8-3z"/>`;
}

function appendProfessional(label, value) {
  const item = document.createElement("div");
  item.className = "professional-item";
  const name = document.createElement("span");
  name.textContent = label;
  const data = document.createElement("strong");
  data.textContent = value;
  item.append(name, data);
  $("professionalGrid").append(item);
}

function generateReport() {
  if (!reportReady()) return;
  state.previewActive = false;
  $("reportStatus").classList.remove("sample");
  $("personaContent").hidden = true;
  const names = ["S1", "S2", "S3"];
  const samples = Object.fromEntries(names.map((name) => {
    const source = sensors[name].samples;
    return [name, source.map((point) => ({ ...point }))];
  }));
  const results = Object.fromEntries(names.map((name) => [name, analyze(sensors[name])]));
  const lastSampleTime = Math.max(...names.map((name) => samples[name].at(-1).time));
  const capturedAt = new Date(state.startedAtWall + lastSampleTime * 1000);
  const quality = Math.round(names.reduce((sum, name) => sum + results[name].quality, 0) / 3);
  const reliable = names.filter((name) => {
    const result = results[name];
    return result.quality >= 60 && result.confidence >= .3 &&
      result.beats >= 5 && result.bpm >= 40 && result.bpm <= 180;
  });
  const bpm = reliable.length
    ? reliable.reduce((sum, name) => sum + results[name].bpm, 0) / reliable.length : null;
  const cvs = reliable.map((name) => results[name].rhythmCv).filter(Number.isFinite);
  const cv = cvs.length ? cvs.reduce((a, b) => a + b, 0) / cvs.length : null;
  const ranked = [...names].sort((a, b) => results[b].amplitude - results[a].amplitude);
  const maximum = results[ranked[0]].amplitude;
  const minimum = results[ranked[2]].amplitude;
  const hasWave = maximum >= .1;
  const strongest = hasWave && maximum > minimum * 1.1 ? ranked[0] : null;
  const featured = reliable[0] || ranked[0];
  const lifestyle = lifestyleFromSignals(results, samples, quality, reliable, bpm, cv, hasWave);
  state.reportSnapshot = {
    capturedAt: capturedAt.toISOString(),
    subjectId: $("subjectId").value.trim() || null,
    wristSide: $("wristSide").value,
    posture: $("posture").value,
    source: "S1/S2 device pressure samples; S3 derived from S1 with bounded noise",
    samples, results, quality, bpm, rhythmCv: cv, featured, lifestyle,
  };

  $("reportEmpty").hidden = true;
  $("reportContent").hidden = false;
  reportButton.textContent = "重新生成报告";
  saveButton.disabled = false;
  printButton.disabled = false;
  const date = capturedAt.toLocaleString("zh-CN", {
    year: "numeric", month: "2-digit", day: "2-digit",
    hour: "2-digit", minute: "2-digit", hour12: false,
  }).replace(/\//g, ".");
  const parts = [date];
  if (state.reportSnapshot.subjectId) parts.push("受试编号 " + state.reportSnapshot.subjectId);
  if (state.reportSnapshot.wristSide !== "未记录") parts.push(state.reportSnapshot.wristSide);
  parts.push(state.reportSnapshot.posture + "测量");
  $("reportDate").textContent = parts.join("  ·  ");
  // Exhibition report is deliberately sourced from the unified simulated profile.
  // Raw measurements stay available in the professional details and saved record.
  const reportViewModel = displayReportData;
  state.reportViewModel = reportViewModel;
  renderReportDashboard($("reportContent"), reportViewModel);

  const allContent = $("allWavesContent");
  allContent.replaceChildren();
  for (const name of names) {
    const section = document.createElement("section");
    section.className = "full-wave";
    const heading = document.createElement("h4");
    heading.textContent = sensors[name].label + "部 · " + name + " · " + results[name].sampleRate.toFixed(0) + " Hz";
    const canvas = document.createElement("canvas");
    canvas.id = "fullWave" + name;
    canvas.setAttribute("aria-label", sensors[name].label + "部完整真实采样波形");
    section.append(heading, canvas);
    allContent.append(section);
  }

  $("professionalGrid").replaceChildren();
  appendProfessional("脉率估计", bpm === null ? "暂无法稳定估计" : Math.round(bpm) + " 次/分");
  appendProfessional("节律变异 CV", cv === null ? "暂无法判断" : (cv * 100).toFixed(1) + "%");
  appendProfessional("采集质量", quality + " / 100");
  for (const name of names) {
    appendProfessional(sensors[name].label + "部" + (name === "S3" ? "派生波幅" : "原始波幅"), results[name].amplitude.toFixed(3) + " hPa");
  }
  $("professionalNote").textContent = "主页面采用人物设定的参考波形，不由本次信号推断中医病证。以下为实际采集数据：原始波幅按 P95–P5 压力差计算；三部相对强弱受探头接触影响。";

  $("qualityScore").textContent = quality + " / 100";
  $("qualityBar").style.width = quality + "%";
  $("qualityText").textContent = quality >= 80
    ? "本次采样的完整度和周期相关性较好。"
    : quality >= 60 ? "本次信号基本可分析，保持手腕静止可帮助复测。" : "本次信号质量偏低，建议调整探头位置后重新采集。";
  const technical = $("technicalGrid");
  technical.replaceChildren();
  for (const name of names) {
    const result = results[name];
    const block = document.createElement("div");
    block.className = "technical-card";
    const title = document.createElement("h4");
    title.textContent = sensors[name].label + " · " + name + (name === "S3" ? " · 寸部派生" : "");
    const detail = document.createElement("p");
    detail.textContent = "平均压力 " + result.mean.toFixed(3) + " hPa · 波幅 P95–P5 " + result.amplitude.toFixed(3) +
      " hPa · 采样 " + result.sampleRate.toFixed(1) + " Hz · 有效时长 " + result.duration.toFixed(1) +
      " 秒 · 最大间隔 " + result.maxGap.toFixed(3) + " 秒 · 周期相关 " + result.confidence.toFixed(2) +
      " · 通道质量 " + result.quality + "/100";
    block.append(title, detail);
    technical.append(block);
  }
  requestAnimationFrame(drawReportWaves);
}

function showPersonaPreview() {
  state.previewActive = true;
  $("reportEmpty").hidden = true;
  $("reportContent").hidden = true;
  $("personaContent").hidden = false;
  saveButton.disabled = true;
  printButton.disabled = false;
  $("reportStatus").textContent = "报告已完成";
  $("reportStatus").classList.remove("sample");
  $("reportStatus").classList.add("ready");
  renderReportDashboard($("personaContent"), displayReportData);
  document.querySelectorAll("[data-sample-field]").forEach((node) => {
    const value = displayReportData[node.dataset.sampleField];
    node.textContent = node.dataset.sampleField === "bpm" ? value + " 次/分" : node.dataset.sampleField === "statusScore" ? value + " / 100" : value;
  });
  showPage("report");
}

function createReport() {
  if (reportReady()) generateReport();
  else showPersonaPreview();
}

function drawPersonaWave() {
  if (!state.previewActive || state.activePage !== "report") return;
  drawDashboardVisuals($("personaContent"), displayReportData);
}

function saveReport() {
  const snapshot = state.reportSnapshot;
  if (!snapshot) return;
  const payload = { schema: "qiheng-pulse-report-v1", note: "仅供采集记录，不作医疗诊断", ...snapshot };
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = "pulse-report-" + snapshot.capturedAt.replace(/[:.]/g, "-") + ".json";
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
connectButton.addEventListener("click", () => state.connected ? disconnectSerial() : connectSerial());
rosConnectButton.addEventListener("click", () => (state.rosConnected || state.rosConnecting) ? disconnectRos() : connectRos());
demoButton.addEventListener("click", startDemo);
$("clearButton").addEventListener("click", resetData);
reportButton.addEventListener("click", createReport);
saveButton.addEventListener("click", saveReport);
printButton.addEventListener("click", () => window.print());
$("pageCapture").addEventListener("click", () => showPage("capture"));
$("pageReport").addEventListener("click", () => state.reportSnapshot ? showPage("report") : createReport());
$("viewReportButton").addEventListener("click", createReport);
$("returnCaptureButton").addEventListener("click", () => showPage("capture"));
$("reportBackButton").addEventListener("click", () => showPage("capture"));
$("allWaves").addEventListener("toggle", () => requestAnimationFrame(drawReportWaves));
document.querySelectorAll("[data-open-details]").forEach((button) => button.addEventListener("click", () => {
  $("sampleDetails").hidden = !state.previewActive;
  $("liveDetails").hidden = state.previewActive;
  $("detailsSaveButton").disabled = !state.reportSnapshot;
  $("detailsDialog").showModal();
  requestAnimationFrame(drawReportWaves);
}));
document.querySelector("[data-close-details]").addEventListener("click", () => $("detailsDialog").close());
$("detailsDialog").addEventListener("click", (event) => {
  if (event.target === $("detailsDialog")) $("detailsDialog").close();
});
$("detailsSaveButton").addEventListener("click", saveReport);
$("detailsPrintButton").addEventListener("click", () => window.print());
window.addEventListener("hashchange", () => showPage(location.hash === "#report" ? "report" : "capture", false));
window.addEventListener("resize", () => requestAnimationFrame(() => {
  drawReportWaves();
  drawPersonaWave();
}));
$("windowRange").addEventListener("input", (event) => {
  state.windowSeconds = Number(event.target.value);
  $("windowValue").textContent = `${state.windowSeconds} 秒`;
});
navigator.serial?.addEventListener("disconnect", disconnectSerial);
resetData();
if (location.hash === "#report") showPersonaPreview();
else showPage("capture", false);
renderLoop();
fetch(rosGatewayUrl("/api/status"), { cache: "no-store" })
  .then((response) => {
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  })
  .then(() => connectRos())
  .catch(() => setConnection("ROS 网关未启动，可使用串口直连"));
